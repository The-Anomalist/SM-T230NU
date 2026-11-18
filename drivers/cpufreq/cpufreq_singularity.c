/*
 * Singularity shared-policy CPU governor for PXA1088.
 * Copyright (C) 2026 Singularity contributors
 * Licensed under the GNU General Public License version 2.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "cpufreq_governor.h"

struct singularity_cpu {
	u64 idle, wall;
	bool valid;
};

struct singularity_policy {
	struct cpufreq_policy *policy;
	struct delayed_work work;
	struct mutex lock;
	struct singularity_cpu cpu[NR_CPUS];
	unsigned int sampling_rate, target_load, hispeed_freq;
	unsigned int hispeed_load, down_rate_us, idle_rate_us;
	unsigned int load, smooth_load, last_target;
	unsigned long last_up, boost_until, last_input;
	unsigned int input_boost_freq, input_boost_us;
	struct input_handler input_handler;
	int last_error;
	bool running;
};

static unsigned int singularity_smooth(unsigned int previous, unsigned int load)
{
	return max(load, (3 * previous + load) / 4);
}

static unsigned int singularity_target(unsigned int cur_freq, unsigned int load,
	unsigned int smooth, unsigned int target_load,
	unsigned int hispeed_freq, unsigned int hispeed_load)
{
	unsigned int target = div_u64((u64)cur_freq * smooth, target_load);

	if (load >= hispeed_load) {
		target = max(target, hispeed_freq);
		if (load >= 95)
			target = max(target, cur_freq + cur_freq / 4);
	}
	return target;
}

/* Caller serializes against LIMITS and sysfs with sp->lock. */
static void singularity_sample(struct work_struct *work)
{
	struct singularity_policy *sp = container_of(work,
		struct singularity_policy, work.work);
	struct cpufreq_policy *p = sp->policy;
	unsigned int cpu, load = 0, samples = 0, target, index, delay;
	struct cpufreq_frequency_table *table;

	mutex_lock(&sp->lock);
	if (!sp->running)
		goto out;
	for_each_cpu(cpu, p->cpus) {
		struct singularity_cpu *sc = &sp->cpu[cpu];
		u64 idle, wall, di, dw;

		if (!cpu_online(cpu)) {
			sc->valid = false;
			continue;
		}
		/* I/O wait is idle: storage stalls must not pin a Cortex-A7. */
		idle = get_cpu_idle_time(cpu, &wall, 0);
		di = idle - sc->idle;
		dw = wall - sc->wall;
		if (sc->valid && wall > sc->wall && idle >= sc->idle &&
		    dw && di <= dw) {
			load = max_t(unsigned int, load, div64_u64(
				100 * (dw - di), dw));
			samples++;
		}
		sc->idle = idle;
		sc->wall = wall;
		sc->valid = true;
	}
	if (!samples) {
		target = p->cur;
		goto boost;
	}
	sp->load = load;
	/* Immediate rise, EWMA decay to bridge short gaps between UI work. */
	sp->smooth_load = singularity_smooth(sp->smooth_load, load);
	target = singularity_target(p->cur, load, sp->smooth_load,
		sp->target_load, sp->hispeed_freq, sp->hispeed_load);
boost:
	if (sp->input_boost_us &&
	    time_before(jiffies, ACCESS_ONCE(sp->boost_until)))
		target = max(target, sp->input_boost_freq);
	target = clamp(target, p->min, p->max);
	/* Resolve before comparing: avoid rounding a down request back up. */
	table = cpufreq_frequency_get_table(p->cpu);
	if (!table) {
		sp->last_error = -ENODEV;
		goto requeue;
	}
	sp->last_error = cpufreq_frequency_table_target(p, table, target,
		CPUFREQ_RELATION_L, &index);
	if (sp->last_error)
		goto requeue;
	target = table[index].frequency;
	if (target < p->cur && time_before(jiffies,
	    sp->last_up + usecs_to_jiffies(sp->down_rate_us)))
		goto requeue;
	sp->last_target = target;
	if (target != p->cur) {
		bool up = target > p->cur;

		sp->last_error = __cpufreq_driver_target(p, target,
			CPUFREQ_RELATION_L);
		if (!sp->last_error && up)
			sp->last_up = jiffies;
	}
requeue:
	/* Deferrable work does not wake an otherwise idle CPU. */
	delay = sp->load < 10 && sp->smooth_load < 10 && p->cur == p->min ?
		sp->idle_rate_us : sp->sampling_rate;
	queue_delayed_work(system_freezable_wq, &sp->work,
		max(1UL, usecs_to_jiffies(delay)));
out:
	mutex_unlock(&sp->lock);
}

#define SINGULARITY_TUNABLE(name, low, high) \
static ssize_t show_##name(struct cpufreq_policy *p, char *buf) \
{ \
	struct singularity_policy *sp = p->governor_data; \
	return sprintf(buf, "%u\n", sp->name); \
} \
static ssize_t store_##name(struct cpufreq_policy *p, const char *buf, \
			    size_t count) \
{ \
	struct singularity_policy *sp = p->governor_data; \
	unsigned int value; \
	if (kstrtouint(buf, 10, &value) || value < (low) || value > (high)) \
		return -EINVAL; \
	mutex_lock(&sp->lock); \
	sp->name = value; \
	mutex_unlock(&sp->lock); \
	return count; \
} \
static struct freq_attr name##_attr = \
	__ATTR(name, 0644, show_##name, store_##name)

SINGULARITY_TUNABLE(sampling_rate, 10000, 100000);
SINGULARITY_TUNABLE(target_load, 50, 95);
SINGULARITY_TUNABLE(hispeed_freq, 1, 2000000);
SINGULARITY_TUNABLE(hispeed_load, 50, 100);
SINGULARITY_TUNABLE(down_rate_us, 10000, 500000);
SINGULARITY_TUNABLE(idle_rate_us, 100000, 1000000);
SINGULARITY_TUNABLE(input_boost_freq, 1, 2000000);
SINGULARITY_TUNABLE(input_boost_us, 0, 200000);

static ssize_t show_status(struct cpufreq_policy *p, char *buf)
{
	struct singularity_policy *sp = p->governor_data;
	ssize_t ret;

	mutex_lock(&sp->lock);
	ret = sprintf(buf, "load=%u smooth_load=%u target_khz=%u error=%d\n",
		sp->load, sp->smooth_load, sp->last_target, sp->last_error);
	mutex_unlock(&sp->lock);
	return ret;
}
static struct freq_attr status_attr = __ATTR(status, 0444, show_status, NULL);
static struct attribute *singularity_attrs[] = {
	&sampling_rate_attr.attr, &target_load_attr.attr,
	&hispeed_freq_attr.attr, &hispeed_load_attr.attr,
	&down_rate_us_attr.attr, &idle_rate_us_attr.attr,
	&input_boost_freq_attr.attr, &input_boost_us_attr.attr,
	&status_attr.attr, NULL,
};
static struct attribute_group singularity_group = {
	.name = "singularity",
	.attrs = singularity_attrs,
};

/* Input callbacks run under the input device spinlock: never sleep here. */
static void singularity_input_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	struct singularity_policy *sp = handle->handler->private;
	unsigned long now = jiffies;
	unsigned int duration = ACCESS_ONCE(sp->input_boost_us);

	if (!duration || type != EV_SYN || code != SYN_REPORT)
		return;
	/* Bound queue churn while dragging; this is a floor, not a max vote. */
	if (time_before(now, ACCESS_ONCE(sp->last_input) +
	    max(1UL, usecs_to_jiffies(20000))))
		return;
	ACCESS_ONCE(sp->last_input) = now;
	ACCESS_ONCE(sp->boost_until) = now + usecs_to_jiffies(duration);
	mod_delayed_work(system_freezable_wq, &sp->work, 0);
}

static int singularity_input_connect(struct input_handler *handler,
	struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "singularity";
	ret = input_register_handle(handle);
	if (ret)
		goto free;
	ret = input_open_device(handle);
	if (!ret)
		return 0;
	input_unregister_handle(handle);
free:
	kfree(handle);
	return ret;
}

static void singularity_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/* Match touchscreens, including the tablet's single-touch protocol. */
static const struct input_device_id singularity_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_ABSBIT,
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{ },
};

static int singularity_governor(struct cpufreq_policy *p, unsigned int event)
{
	struct singularity_policy *sp = p->governor_data;
	unsigned int cpu;
	int ret;

	switch (event) {
	case CPUFREQ_GOV_POLICY_INIT:
		sp = kzalloc(sizeof(*sp), GFP_KERNEL);
		if (!sp)
			return -ENOMEM;
		sp->policy = p;
		sp->sampling_rate = 20000;
		sp->target_load = 80;
		sp->hispeed_freq = 624000;
		sp->hispeed_load = 85;
		sp->down_rate_us = 60000;
		sp->idle_rate_us = 100000;
		sp->input_boost_freq = 624000;
		sp->input_boost_us = 60000;
		sp->input_handler.private = sp;
		sp->input_handler.event = singularity_input_event;
		sp->input_handler.connect = singularity_input_connect;
		sp->input_handler.disconnect = singularity_input_disconnect;
		sp->input_handler.name = "singularity";
		sp->input_handler.id_table = singularity_input_ids;
		mutex_init(&sp->lock);
		INIT_DEFERRABLE_WORK(&sp->work, singularity_sample);
		p->governor_data = sp;
		ret = sysfs_create_group(&p->kobj, &singularity_group);
		if (ret) {
			p->governor_data = NULL;
			kfree(sp);
		}
		return ret;
	case CPUFREQ_GOV_START:
		if (!p->cur || !cpu_online(p->cpu))
			return -EINVAL;
		mutex_lock(&sp->lock);
		for_each_possible_cpu(cpu) {
			sp->cpu[cpu].idle = get_cpu_idle_time(cpu,
				&sp->cpu[cpu].wall, 0);
			sp->cpu[cpu].valid = cpu_online(cpu);
		}
		sp->load = sp->smooth_load = 0;
		sp->last_up = jiffies;
		sp->boost_until = jiffies;
		sp->last_input = jiffies - usecs_to_jiffies(20000);
		sp->running = true;
		queue_delayed_work(system_freezable_wq, &sp->work,
			max(1UL, usecs_to_jiffies(sp->sampling_rate)));
		mutex_unlock(&sp->lock);
		ret = input_register_handler(&sp->input_handler);
		if (ret) {
			mutex_lock(&sp->lock);
			sp->running = false;
			mutex_unlock(&sp->lock);
			cancel_delayed_work_sync(&sp->work);
			return ret;
		}
		break;
	case CPUFREQ_GOV_STOP:
		/* Drain all input callbacks before cancelling delayed work. */
		input_unregister_handler(&sp->input_handler);
		mutex_lock(&sp->lock);
		sp->running = false;
		mutex_unlock(&sp->lock);
		/* Never hold lock while draining a worker which acquires it. */
		cancel_delayed_work_sync(&sp->work);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&sp->lock);
		if (p->cur > p->max)
			sp->last_error = __cpufreq_driver_target(p, p->max,
				CPUFREQ_RELATION_H);
		else if (p->cur < p->min)
			sp->last_error = __cpufreq_driver_target(p, p->min,
				CPUFREQ_RELATION_L);
		mutex_unlock(&sp->lock);
		break;
	case CPUFREQ_GOV_POLICY_EXIT:
		sysfs_remove_group(&p->kobj, &singularity_group);
		cancel_delayed_work_sync(&sp->work);
		p->governor_data = NULL;
		kfree(sp);
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SINGULARITY
static
#endif
struct cpufreq_governor cpufreq_gov_singularity = {
	.name = "singularity",
	.governor = singularity_governor,
	.max_transition_latency = 1000000,
	.owner = THIS_MODULE,
};
static int __init singularity_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_singularity);
}
static void __exit singularity_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_singularity);
}
#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SINGULARITY
fs_initcall(singularity_init);
#else
module_init(singularity_init);
#endif
module_exit(singularity_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Singularity PXA1088 shared-policy CPU governor");

#!/usr/bin/env python3
"""Execute the governor's actual arithmetic on the host; no kernel emulation."""
import ctypes
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'drivers/cpufreq/cpufreq_singularity.c').read_text()
start = source.index('static unsigned int singularity_smooth(')
end = source.index('/* Caller serializes', start)
functions = source[start:end].replace('static unsigned int', 'unsigned int')
with tempfile.TemporaryDirectory(prefix='singularity-test-') as temp:
    temp = Path(temp)
    c = temp / 'logic.c'
    c.write_text('#include <stdint.h>\n'
                 'typedef uint64_t u64;\n'
                 '#define max(a,b) ((a) > (b) ? (a) : (b))\n'
                 '#define div_u64(a,b) ((a)/(b))\n' + functions)
    subprocess.run(['cc', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', str(c), '-o', str(temp / 'logic.so')],
                   check=True)
    lib = ctypes.CDLL(str(temp / 'logic.so'))
    smooth, target = lib.singularity_smooth, lib.singularity_target
    smooth.argtypes = [ctypes.c_uint] * 2
    target.argtypes = [ctypes.c_uint] * 6
    smooth.restype = target.restype = ctypes.c_uint
    checks = 0
    for old in range(101):
        for load in range(101):
            value = smooth(old, load)
            assert load <= value <= 100
            if load >= old:
                assert value == load
            else:
                assert load <= value <= old
            checks += 1
    value = 100
    for _ in range(20):
        value = smooth(value, 0)
    assert value == 0, 'idle history must drain fully'
    tables = [[312000, 624000, 1066000, 1183000],
              [156000, 208000, 312000, 416000, 533000, 624000,
               800000, 1066000, 1183000, 1248000, 1482000]]
    for table in tables:
        for current in table:
            for load in range(101):
                for history in range(load, 101):
                    for tl in (50, 80, 95):
                        value = target(current, load, history, tl, 624000, 85)
                        assert value >= current * history // tl
                        if load >= 85:
                            assert value >= 624000
                        if load >= 95:
                            assert value >= current + current // 4
                        bounded = max(table[0], min(value, table[-1]))
                        resolved = next(f for f in table if f >= bounded)
                        assert table[0] <= resolved <= table[-1]
                        checks += 1
        # A new sustained saturated task must escape the lowest OPP promptly.
        current, history = table[0], 0
        for _ in range(6):
            history = smooth(history, 100)
            value = min(table[-1], target(current, 100, history, 80, 624000, 85))
            current = next(f for f in table if f >= value)
        assert current == table[-1]
    # High clock range must not overflow the 32-bit load product.
    assert target(2000000, 100, 100, 50, 624000, 85) == 4000000
    print('PASS:', checks, 'arithmetic/table-bound cases; burst ramp and idle decay')

#!/usr/bin/env python3
"""Generate a CAM-like 3D surfacing program (ball-end raster over a wavy surface) for benchmarks.

    python examples/bench/make_surface.py [stepover_mm] > surface.nc

Typical CAM finishing output: thousands of short G1 moves with a tiny chord tolerance.
"""
import math
import sys

step = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
out = ["%", "O04000 (BENCH: 3D RASTER FINISH, 6 MM BALL)", "G21 G17 G40 G49 G80 G90 G94",
       "T5 M06", "G00 G90 G54 B0. C0. X-40. Y-40. S12000 M03", "G43 H5 Z10. M08", "G01 Z2. F2000."]
n = 0
y = -40.0
direction = 1
while y <= 40.0 + 1e-9:
    xs = [i * 0.4 - 40.0 for i in range(201)]
    if direction < 0:
        xs.reverse()
    for x in xs:
        r = math.hypot(x, y)
        z = -1.5 + 1.2 * math.cos(r / 6.0) * math.exp(-r / 40.0)
        out.append(f"X{x:.3f} Y{y:.3f} Z{z:.4f}")
        n += 1
    y += step
    direction = -direction
out += ["G00 Z25. M09", "G91 G28 Z0.", "G90", "M30", "%"]
sys.stdout.write("\n".join(out) + "\n")
print(f"{n} moves", file=sys.stderr)

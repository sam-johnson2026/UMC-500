#!/usr/bin/env python3
"""Generate examples/demo_5axis.nc: facing, a helical pocket, 3+2 cross holes (G254) and a
simultaneous 5-axis chamfer around the top edge (G234 TCPC).

The chamfer's B/C angles are computed with the twin's own kinematics so the tool leans
radially outward at every point -- if the B/C sign convention in config/umc500.yaml changes,
re-run this script.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from umc_twin import Kinematics, load_machine  # noqa: E402

R_STOCK = 50.0
kin = Kinematics(load_machine())


def xyz(p) -> str:
    p = np.round(p, 3) + 0.0  # no "-0.000"
    return f"X{p[0]:.3f} Y{p[1]:.3f} Z{p[2]:.3f}"


def c_for_outward_tilt(phi_deg: float, b: float) -> float:
    """C angle that makes the tool axis (as seen from the part) lean toward polar angle phi."""
    q = np.array([0, 0, 0, b, 0.0])
    up = -kin.tool_axis_in_table(q)  # tip -> spindle, as seen from the part
    lean = math.degrees(math.atan2(up[1], up[0]))
    return (phi_deg - lean) % 360.0


out = ["%", "O01000 (UMC-500 TWIN DEMO)", "(STOCK: 100 DIA X 50 CYLINDER, G54 = TOP CENTRE)",
       "G21 G17 G40 G49 G80 G90 G94",
       "",
       "(--- T1 FACE ---)",
       "T1 M06", "G00 G90 G54 B0. C0. X-80. Y-35. S4000 M03", "G43 H1 Z25. M08", "G01 Z-0.5 F800."]
for k, y in enumerate(np.arange(-35, 36, 35)):
    out.append(f"G01 X{80 if k % 2 == 0 else -80:.0f}. Y{y:.1f} F1500.")
    out.append(f"Y{y + 35:.1f}" if y < 35 else "")
out = [x for x in out if x]
out += ["G00 Z25. M09", "G91 G28 Z0.", "G90", "",
        "(--- T2 HELICAL POCKET 30 DIA X 10 DEEP ---)",
        "T2 M06", "G00 G90 G54 X10. Y0. S9000 M03", "G43 H2 Z5. M08", "G01 Z0.5 F500."]
z = 0.5
while z > -10:
    z = max(z - 2.0, -10.0)
    out.append(f"G03 X10. Y0. I-10. J0. Z{z:.1f} F1200.")
out += ["G03 X10. Y0. I-10. J0.", "G01 X0. Y0.", "G00 Z25. M09", "G91 G28 Z0.", "G90", "",
        "(--- T3 3+2 ANGLED HOLES AT B45 WITH DWO ---)",
        "(G254 KEEPS XYZ IN THE PART FRAME AS IT SITS AT B0 C0, SO EACH HOLE IS)",
        "(DRILLED ALONG THE TOOL AXIS AS SEEN FROM THE PART)",
        "(RADIAL HOLES AT B90 ARE NOT POSSIBLE HERE: THE HEAD IS ~370 WIDE AT NOSE LEVEL)",
        "(AND THE TILTED PLATTER REACHES Z+200 -- TRY IT, THE SIM CATCHES IT)",
        "T3 M06", "G00 G90 G54 B0. C0. S2500 M03"]
for c in (0, 90, 180, 270):
    down = kin.tool_axis_in_table(np.array([0, 0, 0, 45.0, c]))  # spindle -> tip, part frame
    up = -down
    radial = np.array([up[0], up[1], 0.0]) / np.hypot(up[0], up[1])
    entry = radial * 30.0  # on the top face, 30 mm out from the centre
    start_pt, clear = entry + up * 20.0, entry + up * 5.0
    out += [f"G00 B45. C{c}.", "G254", "G43 H3 " + xyz(start_pt) + " M08", "G00 " + xyz(clear)]
    for depth in (6, 12, 18):  # pecks along the tool axis
        out += ["G01 " + xyz(entry + down * depth) + " F250.", "G00 " + xyz(clear)]
    out += ["G00 " + xyz(start_pt), "G255", "G00 G53 Z0. M09"]
out += ["G49", "G90", ""]

# Simultaneous 5-axis chamfer: tip rides the top edge, tool leans 45 deg outward.
B = 45.0
out += ["(--- T4 5-AXIS CHAMFER, TCPC ---)", "T4 M06", "G00 G90 G54 B0. C0. S8000 M03"]
c0 = c_for_outward_tilt(0.0, B)
out += [f"G00 B{B:.1f} C{c0:.3f}", "G234 H4", f"G00 X{R_STOCK + 20:.3f} Y0. Z20.", "M08",
        f"G01 X{R_STOCK - 1.5:.3f} Y0. Z-1.5 F600."]
prev_c = c0
for deg in range(5, 361, 5):
    phi = math.radians(deg)
    c = c_for_outward_tilt(deg, B)
    while c < prev_c - 180:  # keep C monotonic -- no unwinding mid-cut
        c += 360
    while c > prev_c + 180:
        c -= 360
    prev_c = c
    out.append(f"X{(R_STOCK - 1.5) * math.cos(phi):.3f} Y{(R_STOCK - 1.5) * math.sin(phi):.3f} C{c:.3f} F1200.")
out += [f"G01 X{R_STOCK + 20:.3f} Y0. Z20.", "M09", "G49", "G91 G28 Z0.", "G28 X0. Y0. B0. C0.", "G90", "M30", "%"]

path = Path(__file__).with_name("demo_5axis.nc")
path.write_text("\n".join(out) + "\n")
print(f"wrote {path} ({len(out)} lines)")

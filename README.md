# UMC-500 digital twin

A kinematic simulation of our **Haas UMC-500SS** (5-axis trunnion, 120-pocket side-mount tool changer),
built from Haas's own solid model. It is the first layer of a digital twin: the geometry and motion
model that live machine data and, later, a physics simulation will plug into.

What it does today:

- **Moves like the machine.** X/Y/Z on the spindle, B (tilt) and C (rotary) on the table, in Haas machine
  coordinates, with travel limits.
- **Runs G-code.** Haas-flavoured interpreter: G0/G1/G2/G3, work offsets, tool length, canned drill cycles,
  G28/G53, and 5-axis **TCPC (G234)** and **DWO (G254)**. Produces a time-stamped trajectory and a
  cycle-time estimate.
- **Catches crashes before the machine does.** Checks every pose against the real CAD geometry (spindle
  head, tool and holder against trunnion, platter, base, stock and fixtures), plus travel limits and
  rapids into the stock.
- **Shows it in 3D.** A browser viewer with jog sliders, program playback, a toolpath, issue list, and a
  live mode.

![demo](docs/demo.png)

## Quick start

```bash
pip install -e ".[collision]"          # numpy, pyyaml, trimesh, python-fcl

# simulate a program: cycle time, over-travel, collisions
python -m umc_twin simulate examples/demo_5axis.nc --setup examples/setup_demo.yaml --out demo.json
python -m umc_twin simulate examples/crash_demo.nc --setup examples/setup_demo.yaml

# 3D viewer + simulation API on http://127.0.0.1:8000
python -m umc_twin serve
# ...optionally streaming a program as if it were the live machine (Live tab)
python -m umc_twin serve --replay examples/demo_5axis.nc --setup examples/setup_demo.yaml

# one-off questions
python -m umc_twin pose -252.5 -204.7 -300 0 0 --tool 1 --setup examples/setup_demo.yaml
python -m umc_twin info --option spindle=hsk

pytest                                  # 32 tests
```

The viewer also works as plain static files (`web/`), for example on GitHub Pages. In that mode it can
jog, play the bundled examples, and open result `.json` files from the CLI; simulating new G-code
needs the server.

## How it fits together

```
Haas STEP ──tools/build_assets.py──▶ web/assets/parts/*.glb + machine.json (mesh info, mass props)
                                          │
config/umc500.yaml  (kinematics, limits,  │           job setup YAML (tools, offsets, stock, fixtures)
 options, colours — single source of truth)                          │
          │                               │                          │
          ▼                               ▼                          ▼
   umc_twin.kinematics ◀── umc_twin.gcode (G-code ➜ trajectory) ◀────┘
          │                        │
          │                umc_twin.collision (trimesh + FCL on the CAD meshes)
          │                        │
          └──── umc_twin.sim ──────┴──▶ JSON result ──▶ web viewer (three.js)
                                                     ▲
              umc_twin.live (LiveSource) ── /api/live ┘   ← real machine data plugs in here
```

| Path | What |
|---|---|
| `config/umc500.yaml` | Machine definition: frames, joints, limits, rates, part → link map, options. **Start here.** |
| `umc_twin/kinematics.py` | Forward kinematics, TCP inverse, work-offset helpers |
| `umc_twin/gcode.py` | G-code interpreter → `Trajectory` |
| `umc_twin/collision.py` | Mesh collision checks along a trajectory |
| `umc_twin/sim.py` | `run_job()`: G-code in, checked trajectory and report out |
| `umc_twin/live.py` | `MachineState` + `LiveSource` interface; `ReplaySource` fakes a live machine |
| `umc_twin/server.py` | Serves the viewer, `POST /api/simulate`, `GET /api/live` (SSE). Standard library only |
| `web/` | Viewer (three.js vendored in `web/vendor`, so it runs offline on a shop PC) |
| `tools/build_assets.py` | STEP → GLB meshes. Needs `pip install cadquery-ocp`; run only when the CAD changes |
| `examples/` | Demo program (facing, helical pocket, 3+2 holes, TCPC chamfer), crash demo, job setup |

## Frames and conventions

- **World = machine frame**: Z up, +X right, +Y toward the back, mm. The origin is where the B and C axes
  intersect (the pivot, like Haas's MRZP). The platter top is at Z −50.8 (2″ below the B axis).
- **Joints = Haas MACHINE coordinates**: X/Y/Z are 0 at home and travel negative. B is −35° to +110°;
  C is continuous.
- **Tool length** is measured from the spindle gauge line (nose face). At home the gauge point sits at
  X 252.5, Y 204.7, Z 210.48.
- **Work offsets** in a job setup can be given as machine values (`X/Y/Z`, like the offsets page) or as a
  `table_point` in the table frame (e.g. `[0, 0, -50.8]` is the platter centre, top face).

Everything above was derived from the CAD: bearing bores for B, the platter centre for C, and the spindle
taper for the gauge line.

## What still needs checking on the real machine

These values are marked `VERIFY` in `config/umc500.yaml`. Until they're checked, treat the sim's
collision results as a strong hint, not a guarantee.

1. **Z at home.** The model assumes the CAD shows Z at home: spindle nose 261.3 mm (10.29″) above the
   platter at B0. Home the machine and measure. If it differs, set `cad.pose.Z`.
2. **B direction.** At home, jog B+: the platter face should turn toward the right (+X). The CAD strongly
   suggests this, because the opposite direction would crash the cradle into the spindle at home.
3. **C direction.** Jog C+ and look down: the config assumes the table turns clockwise.
4. **Rotary rapid rates**, the tool-change position and the chip-to-chip time (placeholders now; take them from
   the machine parameters). Linear rapid (1000 ipm) and max feed (650 ipm) are spec-sheet values.
5. **Options.** The Haas model includes every variant. Set `spindle`, `platter` etc. under `options:`
   (or in the viewer's Machine tab). The defaults are 40-taper and T-slot platter.
6. **MRZP.** The control's settings 255–257 hold the calibrated rotary centre. Once live data is
   flowing, compare them with the CAD-derived pivot.

## Limitations (on purpose, for now)

- No acceleration or jerk, so cycle times are optimistic on short moves. The machine's
  look-ahead and G187 settings aren't modelled.
- No material removal. The stock stays a full solid, so the cutter touching the stock during feed moves is
  treated as cutting, and only rapids *entering* the stock are flagged.
- Tools and holders are cylinders from the tool table.
- Not supported yet (the interpreter warns and skips them): macros (`#` variables), subprograms (M97/M98),
  cutter compensation (G41/G42), G68 rotation, and G10 offset setting.
- Mass properties in `machine.json` assume solid cast iron. They're reasonable for the castings, but
  meaningless for sheet-metal parts (enclosure, tanks, conveyors), which are modelled as solid blocks.

## Roadmap to the digital twin

1. **Live data.** Add a `LiveSource` that polls the machine. Haas NGC controls can serve MTConnect;
   Haas also offers other data-collection options. Map actual X/Y/Z/B/C, the program line, tool and
   spindle into `MachineState`, and the viewer's Live tab works unchanged.
2. **Calibrate** the model against the live machine: MRZP, Z home, and axis signs (see the checklist above).
3. **Physics.** Per-link mass and inertia are already exported. Next come axis dynamics (servo models and
   drive limits, which also give realistic cycle times), then cutting forces from material-removal
   simulation (dexel/voxel stock), then thermal growth.
4. **Twin loops**: compare commanded against actual, flag divergence, and predict crashes ahead of the
   running program.

## Rebuilding the meshes

The raw CAD isn't committed (80 MB). Download the UMC-500 solid model from Haas and run:

```bash
pip install cadquery-ocp
python tools/build_assets.py --zip path/to/umc-500_ss_smtc120_solid_models_04_2026.zip
python tools/build_examples.py     # refresh the pre-computed viewer examples
```

This takes about 40 s and writes `web/assets/`. After editing only `config/umc500.yaml`, run
`python -m umc_twin refresh-manifest` for static hosting; the server picks up config changes on its own.
If you change the B/C sign convention, re-run `examples/make_demo.py`, because the demo's 5-axis angles
are computed from the kinematics.

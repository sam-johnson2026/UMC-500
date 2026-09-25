#!/usr/bin/env python3
"""Convert the Haas STEP model into per-part GLB meshes + a machine manifest for the sim.

    python tools/build_assets.py                      # uses machine.model_file from the config
    python tools/build_assets.py --zip path/to/haas_models.zip

Outputs (committed, so the viewer works without the CAD toolchain):
    web/assets/parts/<part id>.glb   meshes in the world (machine) frame, mm, at cad.pose
    web/assets/machine.json          config + per-part bbox / triangle count / mass properties

Needs the `cad` extra (cadquery-ocp). Everything else in the repo only needs the outputs.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import zipfile
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from umc_twin.cadio import triangulate  # noqa: E402
from umc_twin.config import DEFAULT_CONFIG, REPO_ROOT, load_machine  # noqa: E402

from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.IFSelect import IFSelect_RetDone  # noqa: E402
from OCP.STEPCAFControl import STEPCAFControl_Reader  # noqa: E402
from OCP.TCollection import TCollection_ExtendedString  # noqa: E402
from OCP.TDataStd import TDataStd_Name  # noqa: E402
from OCP.TDF import TDF_ChildIterator, TDF_Label  # noqa: E402
from OCP.TDocStd import TDocStd_Document  # noqa: E402
from OCP.XCAFDoc import XCAFDoc_DocumentTool  # noqa: E402

ASSETS = REPO_ROOT / "web" / "assets"


def read_step_components(step_path: Path) -> dict:
    """Return {component name: located TopoDS_Shape} for the top-level assembly."""
    doc = TDocStd_Document(TCollection_ExtendedString("umc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read {step_path}")
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def children(label):
        it = TDF_ChildIterator(label, False)
        while it.More():
            yield it.Value()
            it.Next()

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

    roots = [lab for lab in children(st.BaseLabel()) if st.IsFree_s(lab) and st.IsAssembly_s(lab)]
    if len(roots) != 1:
        raise RuntimeError(f"expected one root assembly, found {len(roots)}")
    out = {}
    for comp in children(roots[0]):
        if not st.IsComponent_s(comp):
            continue
        ref = TDF_Label()
        st.GetReferredShape_s(comp, ref)
        out[name(ref)] = st.GetShape_s(comp)  # already carries the component placement
    return out


def mass_properties(shape, rotation: np.ndarray, pivot: np.ndarray, density: float) -> dict:
    """Uniform-density mass properties in the world frame (kg, m, kg*m^2)."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    vol_mm3 = props.Mass()
    com = props.CentreOfMass()
    com_world = rotation @ (np.array([com.X(), com.Y(), com.Z()]) - pivot)
    m = props.MatrixOfInertia()  # about the centre of mass, mm^5 (density 1)
    inertia = np.array([[m.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])
    inertia_world = rotation @ inertia @ rotation.T
    kg_per_mm3 = density * 1e-9
    return {
        "volume_l": round(vol_mm3 * 1e-6, 3),
        "mass_kg": round(vol_mm3 * kg_per_mm3, 2),
        "com_mm": [round(v, 2) for v in com_world],
        "inertia_kgm2": [[round(v * kg_per_mm3 * 1e-6, 5) for v in row] for row in inertia_world],
    }


def export_glb(path: Path, verts: np.ndarray, faces: np.ndarray, color: str) -> None:
    mesh = trimesh.Trimesh(vertices=verts.astype(np.float32), faces=faces, process=False)
    rgba = [int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1.0]
    mesh.visual = trimesh.visual.TextureVisuals(
        material=trimesh.visual.material.PBRMaterial(baseColorFactor=rgba, metallicFactor=0.3, roughnessFactor=0.6)
    )
    path.write_bytes(trimesh.exchange.gltf.export_glb(trimesh.Scene(mesh), include_normals=True))


def find_step(args, machine) -> Path:
    if args.step:
        return Path(args.step)
    if args.zip:
        with zipfile.ZipFile(args.zip) as z:
            member = next(n for n in z.namelist() if n.lower().endswith(".step") and "__MACOSX" not in n)
            target = REPO_ROOT / machine.raw["machine"]["model_file"]
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(z.read(member))
            print(f"extracted {member} -> {target.relative_to(REPO_ROOT)}")
            return target
    return REPO_ROOT / machine.raw["machine"]["model_file"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--step", help="path to the Haas .STEP file")
    ap.add_argument("--zip", help="path to the Haas solid-model zip (STEP is extracted into cad/)")
    ap.add_argument("--only", help="comma-separated part ids to rebuild")
    args = ap.parse_args()

    machine = load_machine(args.config, calibration=None)
    cad = machine.raw["cad"]
    rotation = np.array(cad["rotation"], dtype=float)
    pivot = np.array(cad["pivot"], dtype=float)
    lin, ang = cad["tessellation"]["linear_deflection"], cad["tessellation"]["angular_deflection"]

    step = find_step(args, machine)
    t0 = time.time()
    print(f"reading {step.name} ...")
    components = read_step_components(step)
    print(f"  {len(components)} components in {time.time() - t0:.0f}s")

    (ASSETS / "parts").mkdir(parents=True, exist_ok=True)
    manifest_path = ASSETS / "machine.json"
    previous = json.loads(manifest_path.read_text())["parts"] if manifest_path.exists() else {}
    only = set(args.only.split(",")) if args.only else None

    parts_out = {}
    for part in machine.parts:
        if only and part.id not in only:
            if part.id in previous:
                parts_out[part.id] = previous[part.id]
            continue
        if part.step not in components:
            raise KeyError(f"part {part.id}: {part.step!r} not found in STEP. Available: {sorted(components)}")
        shape = components[part.step]
        verts, faces = triangulate(shape, lin, ang)
        verts = (verts - pivot) @ rotation.T
        glb = ASSETS / "parts" / f"{part.id}.glb"
        export_glb(glb, verts, faces, part.color)
        info = {
            "file": f"parts/{part.id}.glb",
            "triangles": int(len(faces)),
            "bbox_mm": [np.round(verts.min(0), 2).tolist(), np.round(verts.max(0), 2).tolist()],
            **mass_properties(shape, rotation, pivot, machine.raw["density"]),
        }
        parts_out[part.id] = info
        print(f"  {part.id:18s} {info['triangles']:8d} tris {glb.stat().st_size / 1e6:6.2f} MB  {info['mass_kg']:8.1f} kg")

    manifest = {"config": machine.raw, "parts": parts_out}
    manifest_path.write_text(json.dumps(manifest, indent=1))
    total = sum((ASSETS / p["file"]).stat().st_size for p in parts_out.values())
    print(f"wrote {manifest_path.relative_to(REPO_ROOT)}; meshes total {total / 1e6:.1f} MB")


if __name__ == "__main__":
    main()

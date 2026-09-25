"""Loading geometry files: STL / OBJ / PLY / GLB / 3MF via trimesh, STEP via OpenCascade.

STEP support needs the `cad` extra (`pip install cadquery-ocp`); everything else is always
available. Units are whatever the file uses -- the caller scales (see job.Solid.file_scale).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

STEP_SUFFIXES = {".step", ".stp"}


def load_mesh_file(path: str | Path, linear_deflection: float = 0.2, angular_deflection: float = 0.3) -> trimesh.Trimesh:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    if path.suffix.lower() in STEP_SUFFIXES:
        verts, faces = triangulate(read_step_shape(path), linear_deflection, angular_deflection)
        return trimesh.Trimesh(vertices=verts, faces=faces, process=True)
    loaded = trimesh.load(path, force="scene")
    meshes = loaded.dump() if hasattr(loaded, "dump") else [loaded]
    if not meshes:
        raise ValueError(f"no geometry in {path}")
    return trimesh.util.concatenate(meshes)


def read_step_shape(path: Path):
    """Whole STEP file as one located shape (mm)."""
    try:
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
    except ImportError as e:  # pragma: no cover - depends on the optional extra
        raise ImportError("reading STEP needs `pip install cadquery-ocp`; or export STL") from e
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read {path}")
    reader.TransferRoots()
    return reader.OneShape()


def triangulate(shape, lin_defl: float, ang_defl: float) -> tuple[np.ndarray, np.ndarray]:
    """Tessellate an OpenCascade shape. Vertices are kept per B-rep face so normals stay crisp."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, lin_defl, False, ang_defl, True)
    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n = tri.NbNodes()
            pts = np.empty((n, 3))
            for i in range(n):
                p = tri.Node(i + 1).Transformed(trsf)
                pts[i] = (p.X(), p.Y(), p.Z())
            t = np.array([tri.Triangle(i + 1).Get() for i in range(tri.NbTriangles())]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                t = t[:, ::-1]
            verts.append(pts)
            faces.append(t + offset)
            offset += n
        exp.Next()
    if not verts:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int)
    return np.vstack(verts), np.vstack(faces)

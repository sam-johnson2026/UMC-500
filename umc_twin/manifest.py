"""web/assets/machine.json = the machine config + per-part mesh info.

The mesh info only changes when the CAD is re-tessellated (tools/build_assets.py, needs the
CAD toolchain). The config part changes whenever config/umc500.yaml is edited, so it can be
refreshed on its own -- the server does that on every request, and
`python -m umc_twin refresh-manifest` does it for static hosting.
"""
from __future__ import annotations

import json

from .config import DEFAULT_CONFIG, REPO_ROOT, load_machine

MANIFEST = REPO_ROOT / "web" / "assets" / "machine.json"


def current_manifest(config_path=DEFAULT_CONFIG) -> dict:
    manifest = json.loads(MANIFEST.read_text())
    manifest["config"] = load_machine(config_path).raw
    missing = [p["id"] for p in manifest["config"]["parts"] if p["id"] not in manifest["parts"]]
    if missing:
        manifest["missing_meshes"] = missing  # new parts in the config: rebuild assets
    return manifest


def refresh_manifest(config_path=DEFAULT_CONFIG) -> dict:
    manifest = current_manifest(config_path)
    MANIFEST.write_text(json.dumps(manifest, indent=1))
    return manifest

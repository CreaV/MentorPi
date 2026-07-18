#!/usr/bin/env python3
"""Bake a URDF's zero-pose visuals into one static GLB for preview.

Why this exists: both CAD Viewer and the VS Code URDF Visualizer (three.js
URDF-loader lineage) rendered mentorpi_so101 with fixed-joint subtrees
scattered around the chassis, while offline transform composition proves the
URDF is a correctly assembled single tree (see offline_render_check.png).
A baked GLB bypasses every URDF loader: any mesh viewer renders it as-is.

No joint sliders — this is a look-only artifact, regenerated on demand:
  /tmp/mentorpi-cad-venv/bin/python mechanical/urdf/bake_urdf_glb.py \
      src/mentorpi_description/urdf/mentorpi_so101.urdf \
      src/mentorpi_description/urdf/mentorpi_so101.preview.glb
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "measurements"))
from check_urdf_clearances import transform  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def mesh_disk_path(urdf: Path, filename: str) -> Path:
    if "mentorpi_description/" in filename:
        return REPO / "src/mentorpi_description" / filename.split("mentorpi_description/", 1)[1]
    return (urdf.parent / filename).resolve()


def main() -> None:
    urdf = Path(sys.argv[1])
    out = Path(sys.argv[2])
    root = ET.parse(urdf).getroot()

    palette = {}
    for m in root.findall("material"):
        c = m.find("color")
        if m.get("name") and c is not None:
            palette[m.get("name")] = [float(v) for v in c.get("rgba").split()]

    links = {l.get("name"): l for l in root.findall("link")}
    children, child_names = {}, set()
    for j in root.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        o = j.find("origin")
        m = transform(o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                      o.get("rpy", "0 0 0") if o is not None else "0 0 0")
        children.setdefault(p, []).append((c, m))
        child_names.add(c)
    root_name = next(n for n in links if n not in child_names)
    world = {root_name: np.eye(4)}
    stack = [root_name]
    while stack:
        p = stack.pop()
        for c, m in children.get(p, []):
            world[c] = world[p] @ m
            stack.append(c)

    scene = trimesh.Scene()
    count = 0
    for name, link in links.items():
        for i, v in enumerate(link.findall("visual")):
            o = v.find("origin")
            local = transform(o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                              o.get("rpy", "0 0 0") if o is not None else "0 0 0")
            g = v.find("geometry")
            mesh_el, box, cyl = g.find("mesh"), g.find("box"), g.find("cylinder")
            if mesh_el is not None:
                geom = trimesh.load(mesh_disk_path(urdf, mesh_el.get("filename")), force="mesh")
                scale = [float(s) for s in mesh_el.get("scale", "1 1 1").split()]
                geom.apply_scale(scale)
            elif box is not None:
                geom = trimesh.creation.box([float(s) for s in box.get("size").split()])
            elif cyl is not None:
                geom = trimesh.creation.cylinder(
                    radius=float(cyl.get("radius")), height=float(cyl.get("length")))
            else:
                continue
            mat = v.find("material")
            rgba = [0.7, 0.7, 0.7, 1.0]
            if mat is not None:
                c = mat.find("color")
                if c is not None:
                    rgba = [float(x) for x in c.get("rgba").split()]
                elif mat.get("name") in palette:
                    rgba = palette[mat.get("name")]
            geom.visual = trimesh.visual.ColorVisuals(
                geom, face_colors=(np.array(rgba) * 255).astype(np.uint8))
            scene.add_geometry(geom, node_name=f"{name}_v{i}",
                               transform=world[name] @ local)
            count += 1
    # glTF is Y-up; URDF is Z-up. Rotate so viewers show the robot upright.
    scene.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))
    scene.export(out)
    print(f"baked {count} visuals -> {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

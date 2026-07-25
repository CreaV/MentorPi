#!/usr/bin/env python3
"""Convert the pinned official SO-101 URDF into a prefix-safe xacro macro."""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO_NS = "http://ros.org/wiki/xacro"
PREFIX = "so101_"


def prefixed(value: str) -> str:
    return value if value.startswith(PREFIX) else PREFIX + value


def transform(source: Path) -> ET.Element:
    source_root = ET.parse(source).getroot()
    output = ET.Element("robot", {"name": "so101_module", "xmlns:xacro": XACRO_NS})
    macro = ET.SubElement(
        output,
        "xacro:macro",
        {"name": "so101_module", "params": "parent mount_xyz mount_rpy"},
    )
    ET.SubElement(macro, "joint", {"name": "so101_mount_joint", "type": "fixed"})
    mount = macro[-1]
    ET.SubElement(mount, "parent", {"link": "${parent}"})
    ET.SubElement(mount, "child", {"link": "so101_base_link"})
    ET.SubElement(mount, "origin", {"xyz": "${mount_xyz}", "rpy": "${mount_rpy}"})

    for child in source_root:
        if child.tag not in {"material", "link", "joint", "transmission"}:
            continue
        node = copy.deepcopy(child)
        if "name" in node.attrib:
            node.attrib["name"] = prefixed(node.attrib["name"])
        for element in node.iter():
            if element.tag in {"parent", "child"} and "link" in element.attrib:
                element.attrib["link"] = prefixed(element.attrib["link"])
            elif element.tag == "joint" and "name" in element.attrib:
                element.attrib["name"] = prefixed(element.attrib["name"])
            elif element.tag == "actuator" and "name" in element.attrib:
                element.attrib["name"] = prefixed(element.attrib["name"])
            elif element.tag == "material" and "name" in element.attrib:
                element.attrib["name"] = prefixed(element.attrib["name"])
            elif element.tag == "mesh":
                filename = Path(element.attrib["filename"]).name
                element.attrib["filename"] = f"package://mentorpi_description/meshes/so101/{filename}"
        if node.tag == "link" and node.attrib.get("name") == "so101_gripper_frame_link":
            for element in list(node):
                if element.tag in {"origin", "inertial"}:
                    node.remove(element)
        macro.append(node)
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    root = transform(args.source)
    ET.indent(root, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<!-- Generated from TheRobotStudio/SO-ARM100 commit fda892c. -->\n'
        + ET.tostring(root, encoding="unicode")
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()


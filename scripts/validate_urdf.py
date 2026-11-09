#!/usr/bin/env python3
"""Validate a URDF file for structural and physical correctness.

Checks:

  * ``xml_syntax``: well-formed XML with a ``<robot name="...">`` root.
  * ``links``: unique names; valid origins, geometries, materials, and
    inertial properties (positive mass and principal moments, triangle
    inequalities).
  * ``joints``: unique names; valid type, parent/child links, axis, limits,
    and mimic targets.
  * ``topology``: exactly one root link, one parent joint per non-root link,
    no cycles, and every link reachable from the root.
  * ``meshes`` (with ``--check-meshes``): every referenced mesh file exists.

Exit status is 0 only when every check passes.

Usage:
    python3 scripts/validate_urdf.py robot.urdf
    python3 scripts/validate_urdf.py --check-meshes robot.urdf
    python3 scripts/validate_urdf.py --check-meshes --package-root pkg robot.urdf
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------


@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ""


@dataclass
class Result:
    path: str
    checks: list[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def parse_floats(value: str, count: int) -> list[float]:
    """Parse exactly ``count`` whitespace-separated floats."""
    parts = value.split()
    if len(parts) != count:
        raise ValueError(f"expected {count} numbers, got {len(parts)}")
    return [float(p) for p in parts]


def check_origin(element: ET.Element, where: str) -> list[str]:
    problems: list[str] = []
    origin = element.find("origin")
    if origin is None:
        return problems
    for attr, count in (("xyz", 3), ("rpy", 3)):
        raw = origin.get(attr)
        if raw is None:
            continue
        try:
            values = parse_floats(raw, count)
        except (ValueError, TypeError):
            problems.append(f"{where}: invalid {attr}={raw!r}")
            continue
        if any(not math.isfinite(v) for v in values):
            problems.append(f"{where}: non-finite {attr}={raw!r}")
    return problems


GEOMETRY_TAGS = ("box", "cylinder", "sphere", "mesh")


def check_geometry(geometry: ET.Element, where: str) -> list[str]:
    problems: list[str] = []
    shapes = [child for child in geometry if child.tag in GEOMETRY_TAGS]
    unknown = [child.tag for child in geometry if child.tag not in GEOMETRY_TAGS]
    if unknown:
        problems.append(f"{where}: unknown geometry tags {unknown}")
    if len(shapes) != 1:
        problems.append(f"{where}: expected exactly one geometry shape")
        return problems
    shape = shapes[0]
    try:
        if shape.tag == "box":
            size = parse_floats(shape.get("size", ""), 3)
            if any(v <= 0.0 for v in size):
                problems.append(f"{where}: box size must be positive")
        elif shape.tag == "cylinder":
            radius = float(shape.get("radius", "0"))
            length = float(shape.get("length", "0"))
            if radius <= 0.0 or length <= 0.0:
                problems.append(f"{where}: cylinder dimensions must be positive")
        elif shape.tag == "sphere":
            radius = float(shape.get("radius", "0"))
            if radius <= 0.0:
                problems.append(f"{where}: sphere radius must be positive")
        elif shape.tag == "mesh":
            filename = shape.get("filename", "")
            if not filename:
                problems.append(f"{where}: mesh filename is empty")
            scale = shape.get("scale")
            if scale is not None:
                parse_floats(scale, 3)
    except (ValueError, TypeError) as exc:
        problems.append(f"{where}: invalid {shape.tag} attributes ({exc})")
    return problems


def check_material(material: ET.Element, where: str) -> list[str]:
    problems: list[str] = []
    if not material.get("name"):
        problems.append(f"{where}: material has no name")
    color = material.find("color")
    if color is not None:
        raw = color.get("rgba")
        if raw is None:
            problems.append(f"{where}: <color> has no rgba attribute")
        else:
            try:
                values = parse_floats(raw, 4)
                if any(v < 0.0 or v > 1.0 for v in values):
                    problems.append(f"{where}: rgba components must be in [0, 1]")
            except (ValueError, TypeError):
                problems.append(f"{where}: invalid rgba={raw!r}")
    return problems


def check_inertial(link: ET.Element, where: str) -> list[str]:
    problems: list[str] = []
    inertial = link.find("inertial")
    if inertial is None:
        return problems
    mass_el = inertial.find("mass")
    if mass_el is None:
        problems.append(f"{where}: <inertial> has no <mass>")
    else:
        try:
            mass = float(mass_el.get("value", "0"))
            if mass <= 0.0:
                problems.append(f"{where}: mass must be positive")
        except (ValueError, TypeError):
            problems.append(f"{where}: invalid mass value")
    inertia = inertial.find("inertia")
    if inertia is None:
        problems.append(f"{where}: <inertial> has no <inertia>")
    else:
        try:
            ixx = float(inertia.get("ixx", "0"))
            iyy = float(inertia.get("iyy", "0"))
            izz = float(inertia.get("izz", "0"))
        except (ValueError, TypeError):
            problems.append(f"{where}: invalid inertia diagonal")
            return problems
        if ixx <= 0.0 or iyy <= 0.0 or izz <= 0.0:
            problems.append(f"{where}: principal moments must be positive")
        elif (
            ixx + iyy < izz - 1e-12
            or ixx + izz < iyy - 1e-12
            or iyy + izz < ixx - 1e-12
        ):
            problems.append(f"{where}: inertia triangle inequality violated")
    return problems


def check_links(root: ET.Element) -> list[str]:
    problems: list[str] = []
    links = root.findall("link")
    if not links:
        problems.append("robot has no links")
    names = [link.get("name") for link in links]
    if any(not name for name in names):
        problems.append("every link needs a name")
    for name, count in Counter(names).items():
        if name and count > 1:
            problems.append(f"duplicate link name: {name}")
    for link in links:
        name = link.get("name") or "<unnamed>"
        where = f"link {name}"
        problems.extend(check_origin(link, where))
        problems.extend(check_inertial(link, where))
        for visual in link.findall("visual"):
            problems.extend(check_origin(visual, f"{where} visual"))
            geometry = visual.find("geometry")
            if geometry is None:
                problems.append(f"{where} visual: missing <geometry>")
            else:
                problems.extend(check_geometry(geometry, f"{where} visual"))
            material = visual.find("material")
            if material is not None:
                problems.extend(check_material(material, f"{where} visual"))
        for collision in link.findall("collision"):
            problems.extend(check_origin(collision, f"{where} collision"))
            geometry = collision.find("geometry")
            if geometry is None:
                problems.append(f"{where} collision: missing <geometry>")
            else:
                problems.extend(
                    check_geometry(geometry, f"{where} collision")
                )
    for material in root.findall("material"):
        problems.extend(check_material(material, "robot material"))
    return problems


JOINT_TYPES = {
    "revolute",
    "continuous",
    "prismatic",
    "fixed",
    "floating",
    "planar",
}

AXIS_TYPES = {"revolute", "continuous", "prismatic", "planar"}
LIMIT_TYPES = {"revolute", "prismatic"}


def check_joints(root: ET.Element) -> list[str]:
    problems: list[str] = []
    joints = root.findall("joint")
    link_names = {link.get("name") for link in root.findall("link")}
    joint_names = [joint.get("name") for joint in joints]
    if any(not name for name in joint_names):
        problems.append("every joint needs a name")
    for name, count in Counter(joint_names).items():
        if name and count > 1:
            problems.append(f"duplicate joint name: {name}")
    for joint in joints:
        name = joint.get("name") or "<unnamed>"
        where = f"joint {name}"
        joint_type = joint.get("type", "")
        if joint_type not in JOINT_TYPES:
            problems.append(f"{where}: invalid type {joint_type!r}")
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link") if parent is not None else None
        child_name = child.get("link") if child is not None else None
        if parent_name is None:
            problems.append(f"{where}: missing <parent link>")
        elif parent_name not in link_names:
            problems.append(f"{where}: unknown parent link {parent_name!r}")
        if child_name is None:
            problems.append(f"{where}: missing <child link>")
        elif child_name not in link_names:
            problems.append(f"{where}: unknown child link {child_name!r}")
        if parent_name is not None and parent_name == child_name:
            problems.append(f"{where}: parent and child are the same link")
        problems.extend(check_origin(joint, where))
        axis = joint.find("axis")
        if joint_type in AXIS_TYPES:
            raw = axis.get("xyz") if axis is not None else None
            if raw is None:
                problems.append(f"{where}: missing <axis xyz>")
            else:
                try:
                    values = parse_floats(raw, 3)
                    if math.sqrt(sum(v * v for v in values)) < 1e-12:
                        problems.append(f"{where}: zero-length axis")
                except (ValueError, TypeError):
                    problems.append(f"{where}: invalid axis {raw!r}")
        limit = joint.find("limit")
        if joint_type in LIMIT_TYPES and limit is None:
            problems.append(f"{where}: missing <limit>")
        if limit is not None:
            try:
                effort = float(limit.get("effort", "0"))
                velocity = float(limit.get("velocity", "0"))
                if effort < 0.0:
                    problems.append(f"{where}: negative effort")
                if velocity < 0.0:
                    problems.append(f"{where}: negative velocity")
                if limit.get("lower") is not None and limit.get("upper") is not None:
                    lower = float(limit.get("lower"))
                    upper = float(limit.get("upper"))
                    if lower > upper:
                        problems.append(f"{where}: lower exceeds upper")
            except (ValueError, TypeError):
                problems.append(f"{where}: invalid <limit> values")
        mimic = joint.find("mimic")
        if mimic is not None:
            target = mimic.get("joint")
            if not target:
                problems.append(f"{where}: <mimic> has no joint attribute")
            elif target not in joint_names:
                problems.append(f"{where}: mimic target {target!r} not found")
    # Mimic cycles.
    mimic_map = {}
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint"):
            mimic_map[joint.get("name")] = mimic.get("joint")
    for start in mimic_map:
        seen = set()
        current = start
        while current in mimic_map:
            if current in seen:
                problems.append(f"joint {start}: mimic cycle")
                break
            seen.add(current)
            current = mimic_map[current]
    return problems


def check_topology(root: ET.Element) -> list[str]:
    problems: list[str] = []
    links = [link.get("name") for link in root.findall("link")]
    link_names = {name for name in links if name}
    joints = root.findall("joint")
    parent_of: dict[str, str] = {}
    child_counts: Counter[str] = Counter()
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link") if parent is not None else None
        child_name = child.get("link") if child is not None else None
        if not parent_name or not child_name:
            continue
        if parent_name not in link_names or child_name not in link_names:
            continue
        child_counts[child_name] += 1
        parent_of.setdefault(child_name, parent_name)
    for name, count in child_counts.items():
        if count > 1:
            problems.append(f"link {name}: multiple parent joints")
    roots = [name for name in links if name and name not in parent_of]
    if len(roots) != 1:
        problems.append(
            f"expected exactly one root link, found {len(roots)}: {roots}"
        )
    # Cycle detection by walking parent pointers.
    for name in links:
        if not name:
            continue
        seen: set[str] = set()
        current = name
        for _ in range(len(links) + 1):
            if current in seen:
                problems.append(f"kinematic cycle involving link {name}")
                break
            seen.add(current)
            if current not in parent_of:
                break
            current = parent_of[current]
    if len(roots) == 1:
        # Reachability from the root.
        children: dict[str, list[str]] = {}
        for child, parent in parent_of.items():
            children.setdefault(parent, []).append(child)
        reachable = set()
        stack = [roots[0]]
        while stack:
            link = stack.pop()
            if link in reachable:
                continue
            reachable.add(link)
            stack.extend(children.get(link, []))
        missing = sorted(link_names - reachable)
        if missing:
            problems.append(f"links not reachable from root: {missing}")
    return problems


def resolve_mesh(
    filename: str, urdf_dir: Path, package_root: Path | None
) -> Path:
    if filename.startswith("package://"):
        rest = filename[len("package://") :]
        _package, _, rel = rest.partition("/")
        base = package_root if package_root is not None else urdf_dir.parent
        return base / rel
    path = Path(filename)
    if path.is_absolute():
        return path
    return urdf_dir / path


def check_meshes(
    root: ET.Element, urdf_path: Path, package_root: Path | None
) -> list[str]:
    problems: list[str] = []
    urdf_dir = urdf_path.parent
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename:
            continue
        resolved = resolve_mesh(filename, urdf_dir, package_root)
        if not resolved.exists():
            problems.append(f"missing mesh: {filename} (looked at {resolved})")
    return problems


# ---------------------------------------------------------------------------
# Validation entry points
# ---------------------------------------------------------------------------


def validate_text(
    text: str,
    path: str = "<memory>",
    check_mesh_files: bool = False,
    package_root: Path | None = None,
) -> Result:
    result = Result(path=path)
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        result.checks.append(Check("xml_syntax", False, str(exc)))
        return result
    if root.tag != "robot":
        result.checks.append(
            Check("xml_syntax", False, f"root element is <{root.tag}>, expected <robot>")
        )
        return result
    if not root.get("name"):
        result.checks.append(
            Check("xml_syntax", False, "<robot> is missing a name attribute")
        )
        return result
    result.checks.append(Check("xml_syntax", True))

    link_problems = check_links(root)
    result.checks.append(
        Check("links", not link_problems, "; ".join(link_problems))
    )
    joint_problems = check_joints(root)
    result.checks.append(
        Check("joints", not joint_problems, "; ".join(joint_problems))
    )
    topology_problems = check_topology(root)
    result.checks.append(
        Check("topology", not topology_problems, "; ".join(topology_problems))
    )
    if check_mesh_files:
        mesh_problems = check_meshes(root, Path(path), package_root)
        result.checks.append(
            Check("meshes", not mesh_problems, "; ".join(mesh_problems))
        )
    return result


def validate_file(
    path: str | Path,
    check_mesh_files: bool = False,
    package_root: Path | None = None,
) -> Result:
    urdf_path = Path(path)
    try:
        text = urdf_path.read_text(encoding="utf-8")
    except OSError as exc:
        result = Result(path=str(urdf_path))
        result.checks.append(Check("xml_syntax", False, str(exc)))
        return result
    return validate_text(
        text,
        path=str(urdf_path),
        check_mesh_files=check_mesh_files,
        package_root=package_root,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate a URDF file for structural and physical correctness."
    )
    parser.add_argument("urdf", help="Path to the .urdf file")
    parser.add_argument(
        "--check-meshes",
        action="store_true",
        help="Verify that every referenced mesh file exists",
    )
    parser.add_argument(
        "--package-root",
        help="Directory used to resolve package:// mesh paths",
    )
    args = parser.parse_args(argv)
    package_root = Path(args.package_root) if args.package_root else None
    result = validate_file(
        args.urdf,
        check_mesh_files=args.check_meshes,
        package_root=package_root,
    )
    for check in result.checks:
        status = "PASS" if check.ok else "FAIL"
        line = f"[{status}] {check.name}"
        if not check.ok and check.detail:
            line += f": {check.detail}"
        print(line)
    print("OK" if result.ok else "FAILED")
    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())

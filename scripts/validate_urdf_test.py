#!/usr/bin/env python3
"""Self-tests for scripts/validate_urdf.py (stdlib unittest)."""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_urdf as v  # noqa: E402


def valid_urdf() -> str:
    return """<?xml version="1.0"?>
<robot name="demo">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
    </collision>
  </link>
  <link name="arm">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="j" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def check_map(result):
    return {check.name: check for check in result.checks}


class ValidationTests(unittest.TestCase):
    def test_valid_urdf_passes(self):
        result = v.validate_text(valid_urdf())
        self.assertTrue(result.ok, [c.name for c in result.checks if not c.ok])
        checks = check_map(result)
        self.assertIn("xml_syntax", checks)
        self.assertIn("links", checks)
        self.assertIn("joints", checks)
        self.assertIn("topology", checks)

    def test_malformed_xml(self):
        result = v.validate_text("<robot name='r'><link></robot>")
        self.assertFalse(result.ok)
        self.assertFalse(check_map(result)["xml_syntax"].ok)

    def test_missing_robot_name(self):
        result = v.validate_text("<robot><link name='a'/></robot>")
        self.assertFalse(result.ok)
        self.assertFalse(check_map(result)["xml_syntax"].ok)

    def test_duplicate_link_names(self):
        text = valid_urdf().replace('<link name="arm">', '<link name="base">')
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["links"].ok)

    def test_invalid_inertia(self):
        text = valid_urdf().replace(
            'ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"',
            'ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="5"',
        )
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["links"].ok)

    def test_unknown_joint_parent(self):
        text = valid_urdf().replace(
            '<parent link="base"/>', '<parent link="ghost"/>'
        )
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["joints"].ok)

    def test_zero_axis(self):
        text = valid_urdf().replace('<axis xyz="0 0 1"/>', '<axis xyz="0 0 0"/>')
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["joints"].ok)

    def test_missing_limit(self):
        text = "\n".join(
            line
            for line in valid_urdf().splitlines()
            if "<limit" not in line
        )
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["joints"].ok)

    def test_two_roots(self):
        text = """<?xml version="1.0"?>
<robot name="split">
  <link name="a"/>
  <link name="b"/>
</robot>
"""
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["topology"].ok)

    def test_cycle(self):
        text = valid_urdf().replace('<child link="arm"/>', '<child link="base"/>')
        result = v.validate_text(text)
        self.assertFalse(check_map(result)["topology"].ok)

    def test_mesh_check(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            urdf_dir = tmp_path / "urdf"
            urdf_dir.mkdir()
            (tmp_path / "meshes").mkdir()
            (tmp_path / "meshes" / "a.gltf").write_text("{}", encoding="utf-8")
            text = valid_urdf().replace(
                '<geometry><box size="0.1 0.1 0.1"/></geometry>',
                '<geometry><mesh filename="package://pkg/meshes/a.gltf"/></geometry>',
                1,
            )
            urdf_path = urdf_dir / "demo.urdf"
            urdf_path.write_text(text, encoding="utf-8")
            good = v.validate_file(
                urdf_path, check_mesh_files=True, package_root=tmp_path
            )
            self.assertTrue(good.ok, [c.detail for c in good.checks if not c.ok])
            (tmp_path / "meshes" / "a.gltf").unlink()
            bad = v.validate_file(
                urdf_path, check_mesh_files=True, package_root=tmp_path
            )
            self.assertFalse(check_map(bad)["meshes"].ok)


if __name__ == "__main__":
    unittest.main()

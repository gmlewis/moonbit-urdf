///|
fn mjcf_demo_robot() -> @urdf.Robot {
  let limit = @urdf.JointLimit::new(
    effort=10.0,
    velocity=5.0,
    lower=Some(-1.0),
    upper=Some(1.0),
  )
  let base = @urdf.Link::new("base")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.1, 0.1, mass=1.0))
    .add_collision(
      @urdf.Collision::new(
        @urdf.Geometry::box(0.1, 0.1, 0.1),
        name=Some("base_geom"),
      ),
    )
  let arm = @urdf.Link::new("arm")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.05, 0.05, mass=0.5))
    .add_collision(@urdf.Collision::new(@urdf.Geometry::box(0.1, 0.05, 0.05)))
  let imu = @urdf.Link::new("imu_link").with_inertial(
    @urdf.Inertial::from_box(0.02, 0.02, 0.005, mass=0.01),
  )
  let joint = @urdf.Joint::new(
    "j",
    @urdf.JointType::Revolute,
    parent="base",
    child="arm",
    origin=@urdf.Origin::new(xyz=(0.1, 0.0, 0.0)),
    axis=(0.0, 0.0, 1.0),
  ).with_limit(limit)
  let imu_joint = @urdf.Joint::new(
    "imu_fixed",
    @urdf.JointType::Fixed,
    parent="base",
    child="imu_link",
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.1)),
  )
  @urdf.Robot::new("mjcf-demo")
  .add_link(base)
  .add_link(arm)
  .add_link(imu)
  .add_joint(joint)
  .add_joint(imu_joint)
  .add_loop_constraint(
    @urdf.LoopConstraint::connect(
      name="loop",
      body1="base",
      anchor1=(0.1, 0.0, 0.0),
      body2="arm",
      anchor2=(0.05, 0.0, 0.0),
    ),
  )
}

///|
test "mjcf export carries the expected model structure" {
  let xml = mjcf_demo_robot().to_mjcf_string(mesh_dir="../meshes")
  assert_true(xml.contains("<mujoco model=\"mjcf-demo\">"))
  assert_true(
    xml.contains("<compiler angle=\"radian\" meshdir=\"../meshes\"/>"),
  )
  assert_true(
    xml.contains("<option gravity=\"0 0 -9.81\" timestep=\"0.002\"/>"),
  )
  assert_true(xml.contains("<freejoint/>"))
  assert_true(xml.contains("<inertial pos=\"0 0 0\" mass=\"1\" fullinertia=\""))
  assert_true(
    xml.contains(
      "<joint name=\"j\" type=\"hinge\" axis=\"0 0 1\" range=\"-1 1\" limited=\"true\"/>",
    ),
  )
  assert_true(
    xml.contains(
      "<geom name=\"base_geom_0\" type=\"box\" size=\"0.05 0.05 0.05\"",
    ),
  )
  assert_true(xml.contains("<equality>"))
  assert_true(
    xml.contains(
      "<connect name=\"loop\" body1=\"base\" body2=\"arm\" anchor=\"0.1 0 0\"/>",
    ),
  )
  assert_true(xml.contains("<actuator>"))
  assert_true(
    xml.contains("<motor name=\"j_motor\" joint=\"j\" ctrlrange=\"-10 10\"/>"),
  )
  assert_true(xml.contains("<sensor>"))
  assert_true(
    xml.contains(
      "<accelerometer name=\"imu_link_imu_accel\" site=\"imu_link_imu\"/>",
    ),
  )
  assert_true(
    xml.contains("<gyro name=\"imu_link_imu_gyro\" site=\"imu_link_imu\"/>"),
  )
}

///|
test "urdf preserves loop constraints in comments" {
  let urdf = mjcf_demo_robot().to_urdf_xml()
  assert_true(urdf.contains("<!-- loop_constraint:"))
  assert_true(urdf.contains("body1=base"))
  assert_true(urdf.contains("body2=arm"))
  assert_true(urdf.contains("anchor1=0.1 0 0"))
  assert_true(urdf.contains("anchor2=0.05 0 0"))
}

///|
async test "mjcf is well formed and keeps the robot valid" {
  let robot = mjcf_demo_robot()
  let xml = robot.to_mjcf_string(mesh_dir="../meshes")
  @fs.write_file(
    "/tmp/mjcf-test.xml",
    @encoding/utf8.encode(xml),
    permission=0o644,
    create_mode=@fs.CreateMode::CreateOrTruncate,
  )
  @fs.write_file(
    "/tmp/mjcf-loop.urdf",
    @encoding/utf8.encode(robot.to_urdf_xml()),
    permission=0o644,
    create_mode=@fs.CreateMode::CreateOrTruncate,
  )

  let (xml_code, xml_out) = @process.collect_output_merged("python3", [
    "-c", "import xml.etree.ElementTree as ET; ET.parse('/tmp/mjcf-test.xml')",
  ])
  if xml_code != 0 {
    fail(xml_out.text())
  }
  let (urdf_code, urdf_out) = @process.collect_output_merged("python3", [
    "scripts/validate_urdf.py", "/tmp/mjcf-loop.urdf",
  ])
  if urdf_code != 0 {
    fail(urdf_out.text())
  }

  // Load the model in MuJoCo when it is installed. A failed import means
  // MuJoCo is unavailable (skip); a load failure after a successful import is
  // a real MJCF defect.
  let (import_code, _) = @process.collect_output_merged("python3", [
    "-c", "import mujoco",
  ])
  if import_code == 0 {
    let (load_code, load_out) = @process.collect_output_merged("python3", [
      "-c", "import mujoco; mujoco.MjModel.from_xml_path('/tmp/mjcf-test.xml')",
    ])
    if load_code != 0 {
      fail(load_out.text())
    }
  }
}

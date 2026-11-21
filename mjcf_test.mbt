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
    xml.contains("<site name=\"loop_site1\" pos=\"0.1 0 0\" size=\"0.005\"/>"),
  )
  assert_true(
    xml.contains("<site name=\"loop_site2\" pos=\"0.05 0 0\" size=\"0.005\"/>"),
  )
  assert_true(
    xml.contains(
      "<connect name=\"loop\" site1=\"loop_site1\" site2=\"loop_site2\" solref=\"0.004 1\" solimp=\"0.99 0.999 0.001\"/>",
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

///|
test "mjcf options add ground lift, home keyframe, and contact excludes" {
  let positions : Map[String, Double] = Map([])
  positions.set("j", 0.3)
  let options = @urdf.MjcfOptions::new(
    mesh_dir="../meshes",
    home_qpos=positions,
    default_damping=0.5,
    default_armature=0.01,
    exclude_adjacent_contacts=true,
  )
  let xml = mjcf_demo_robot().to_mjcf_with_options(options)
  assert_true(xml.contains("<body name=\"base\" pos=\"0 0 0.054"))
  assert_true(xml.contains("<key name=\"home\" qpos=\"0 0 0.054"))
  assert_true(xml.contains(" 1 0 0 0 0.3\"/>"))
  assert_true(xml.contains("<default>"))
  assert_true(xml.contains("<joint damping=\"0.5\" armature=\"0.01\"/>"))
  assert_true(xml.contains("<exclude body1=\"base\" body2=\"arm\"/>"))
  assert_true(xml.contains("<exclude body1=\"base\" body2=\"imu_link\"/>"))
  assert_true(xml.contains("<exclude body1=\"arm\" body2=\"imu_link\"/>"))
  assert_true(xml.contains("contype=\"1\""))
  assert_true(xml.contains("contype=\"4\""))
}

///|
test "mjcf options freeze internal joints" {
  let options = @urdf.MjcfOptions::new(freeze_joints=["j"])
  let xml = mjcf_demo_robot().to_mjcf_with_options(options)
  assert_true(!xml.contains("<joint name=\"j\""))
  assert_true(!xml.contains("<motor"))
  assert_true(xml.contains("<equality>"))
}

///|
test "mjcf options emit explicit position and velocity actuators" {
  let options = @urdf.MjcfOptions::new(actuators=[
    @urdf.MjcfActuator::position(
      joint="j",
      kp=70.0,
      kv=3.0,
      ctrl_min=-1.0,
      ctrl_max=1.0,
      forcerange=10.0,
    ),
    @urdf.MjcfActuator::velocity(
      joint="wheel",
      kv=2.0,
      ctrl_min=-30.0,
      ctrl_max=30.0,
      forcerange=5.0,
    ),
  ])
  let xml = mjcf_demo_robot().to_mjcf_with_options(options)
  assert_true(
    xml.contains(
      "<position name=\"j_pos\" joint=\"j\" kp=\"70\" kv=\"3\" ctrlrange=\"-1 1\" forcerange=\"-10 10\"/>",
    ),
  )
  assert_true(
    xml.contains(
      "<velocity name=\"wheel_vel\" joint=\"wheel\" kv=\"2\" ctrlrange=\"-30 30\" forcerange=\"-5 5\"/>",
    ),
  )
  assert_true(!xml.contains("<motor"))
}

///|
test "mjcf emits rolling cylinder geoms for tires" {
  let base = @urdf.Link::new("base")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.1, 0.1, mass=1.0))
    .add_collision(@urdf.Collision::new(@urdf.Geometry::box(0.1, 0.1, 0.1)))
  let tire = @urdf.Link::new("lf_tire")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.1, 0.1, mass=0.2))
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.0889, 0.0381)),
    )
  let robot = @urdf.Robot::new("tire-bot")
    .add_link(base)
    .add_link(tire)
    .add_joint(
      @urdf.Joint::new(
        "axle",
        @urdf.JointType::Continuous,
        parent="base",
        child="lf_tire",
        axis=(0.0, 1.0, 0.0),
      ),
    )
  let xml = robot.to_mjcf_string()
  assert_true(
    xml.contains(
      "<geom name=\"lf_tire_geom_0\" type=\"cylinder\" size=\"0.0889 0.01905\"",
    ),
  )
  assert_true(xml.contains("contype=\"2\""))
  assert_true(xml.contains("friction=\"1.2 0.01 0.001\""))
}

///|
test "mjcf disables contact on internal cycloidal and harmonic stage parts" {
  let base = @urdf.Link::new("casing")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.1, 0.1, mass=1.0))
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.04, 0.01)).with_name(
        "stage1_cycloid_disk_1",
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.03, 0.01)).with_name(
        "stage2_flexspline_cup",
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.025, 0.01)).with_name(
        "stage2_wave_gen",
      ),
    )
  let robot = @urdf.Robot::new("multi-stage-bot").add_link(base)
  let xml = robot.to_mjcf_string()
  assert_true(xml.contains("contype=\"0\" conaffinity=\"0\""))
  assert_true(!xml.contains("contype=\"4\""))
}

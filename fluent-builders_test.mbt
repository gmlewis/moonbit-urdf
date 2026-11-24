///|
test "Link::with_body and with_colored_body add paired visual and collision" {
  let cyl = @urdf.Geometry::Cylinder(0.04, 0.10)
  let base = @urdf.Link::new("base")
    .with_colored_body(cyl, "slate", @urdf.Rgba::new(0.2, 0.25, 0.3))
    .with_inertial(@urdf.Inertial::from_cylinder(0.04, 0.10, mass=0.5))
  assert_eq(base.visuals.length(), 1)
  assert_eq(base.collisions.length(), 1)
  assert_true(base.visuals[0].material is Some(_))
}

///|
test "Joint::with_gear_reduction and mimic_joint configure mimic accurately" {
  let j_out = @urdf.Joint::continuous(
    "carrier_out",
    parent="housing",
    child="carrier",
  ).with_gear_reduction("sun_in", 5.0)
  guard j_out.mimic is Some(m) else { abort("expected mimic") }
  assert_eq(m.joint, "sun_in")
  assert_true((m.multiplier - 0.2).abs() < 1.0e-9)

  let j_rev = @urdf.Joint::continuous(
    "planet_rev",
    parent="carrier",
    child="planet",
  ).mimic_joint("sun_in", multiplier=-0.5, offset=0.1)
  guard j_rev.mimic is Some(m2) else { abort("expected mimic") }
  assert_true((m2.multiplier + 0.5).abs() < 1.0e-9)
  assert_true((m2.offset - 0.1).abs() < 1.0e-9)
}

///|
test "Robot::attach_* builds valid multi-link kinematic chain with transmissions" {
  let cyl = @urdf.Geometry::Cylinder(0.03, 0.12)
  let base = @urdf.Link::new("base")
    .with_body(cyl)
    .with_inertial(@urdf.Inertial::from_cylinder(0.03, 0.12, mass=0.4))
  let upper = @urdf.Link::new("upper_arm")
    .with_body(cyl)
    .with_inertial(@urdf.Inertial::from_cylinder(0.03, 0.12, mass=0.3))
  let tool = @urdf.Link::new("tool")
    .with_body(@urdf.Geometry::Box((0.04, 0.04, 0.04)))
    .with_inertial(@urdf.Inertial::from_box(0.04, 0.04, 0.04, mass=0.1))

  let limit = @urdf.JointLimit::new(
    lower=Some(-1.57),
    upper=Some(1.57),
    effort=10.0,
    velocity=5.0,
  )
  let robot = @urdf.Robot::new("fluent_arm")
    .add_link(base)
    .attach_revolute(
      parent="base",
      joint_name="shoulder",
      upper,
      limit~,
      origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.06)),
      reduction=5.0,
    )
    .attach_fixed(
      parent="upper_arm",
      joint_name="tool_mount",
      tool,
      origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.12)),
    )

  assert_eq(robot.links.length(), 3)
  assert_eq(robot.joints.length(), 2)
  assert_eq(robot.transmissions.length(), 1)
  assert_false(robot.has_errors())
  let xml = robot.to_urdf_xml()
  assert_true(xml.contains("<robot name=\"fluent_arm\">"))
  assert_true(xml.contains("shoulder_trans"))
}

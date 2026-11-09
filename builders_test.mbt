///|
test "visual builder chain" {
  let v = @urdf.Visual::new(@urdf.Geometry::box_mm(10.0, 20.0, 30.0))
    .with_name("body")
    .with_origin(@urdf.Origin::from_mm_deg(x=5.0))
    .with_color("orange", @urdf.Rgba::orange())
  assert_eq(v.name, Some("body"))
  assert_true(!v.origin.is_identity())
  match v.material {
    Some(m) => {
      assert_eq(m.name, "orange")
      assert_true(m.color is Some(_))
    }
    None => fail("expected material")
  }
}

///|
test "link builders are immutable" {
  let base = @urdf.Link::new("base")
  let with_visual = base.add_visual(
    @urdf.Visual::new(@urdf.Geometry::sphere_mm(10.0)),
  )
  let with_collision = with_visual.add_collision(
    @urdf.Collision::new(@urdf.Geometry::sphere_mm(9.0)),
  )
  assert_eq(base.visuals.length(), 0)
  assert_eq(base.collisions.length(), 0)
  assert_eq(with_visual.visuals.length(), 1)
  assert_eq(with_visual.collisions.length(), 0)
  assert_eq(with_collision.collisions.length(), 1)
  let inertial = @urdf.Inertial::new(1.0, @urdf.Inertia::zero())
  let with_inertial = base.with_inertial(inertial)
  assert_true(base.inertial is None)
  assert_true(with_inertial.inertial is Some(_))
}

///|
test "joint convenience constructors" {
  let f = @urdf.Joint::fixed("j0", parent="base", child="arm")
  assert_true(f.joint_type == @urdf.JointType::Fixed)
  assert_eq(f.axis, (1.0, 0.0, 0.0))
  let r = @urdf.Joint::revolute(
    "j1",
    parent="base",
    child="arm",
    limit=@urdf.JointLimit::new(effort=10.0, velocity=2.0),
    axis=(0.0, 1.0, 0.0),
  )
  assert_true(r.joint_type == @urdf.JointType::Revolute)
  assert_true(r.limit is Some(_))
  let c = @urdf.Joint::continuous("j2", parent="a", child="b")
  assert_true(c.joint_type == @urdf.JointType::Continuous)
  assert_true(c.limit is None)
  let p = @urdf.Joint::prismatic(
    "j3",
    parent="a",
    child="b",
    limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
  )
  assert_true(p.joint_type == @urdf.JointType::Prismatic)
  let fl = @urdf.Joint::floating("j4", parent="a", child="b")
  assert_true(fl.joint_type == @urdf.JointType::Floating)
  let pl = @urdf.Joint::planar("j5", parent="a", child="b")
  assert_true(pl.joint_type == @urdf.JointType::Planar)
}

///|
test "joint fluent modifiers" {
  let j = @urdf.Joint::continuous("j", parent="a", child="b")
    .with_origin(@urdf.Origin::from_mm_deg(z=10.0))
    .with_axis((1.0, 0.0, 0.0))
    .with_limit(@urdf.JointLimit::new(effort=3.0, velocity=4.0))
    .with_dynamics(@urdf.JointDynamics::new(damping=0.1, friction=0.2))
    .with_mimic(@urdf.JointMimic::new("other", multiplier=-2.0))
  assert_true(j.limit is Some(_))
  assert_true(j.dynamics is Some(_))
  match j.mimic {
    Some(m) => assert_true((m.multiplier + 2.0).abs() < 1.0e-12)
    None => fail("expected mimic")
  }
}

///|
test "robot builders and lookups are immutable" {
  let robot = @urdf.Robot::new("r", documentation="doc")
    .add_material(@urdf.Material::color_only("red", @urdf.Rgba::red()))
    .add_link(@urdf.Link::new("base"))
    .add_links([@urdf.Link::new("a"), @urdf.Link::new("b")])
    .add_joint(@urdf.Joint::fixed("j", parent="base", child="a"))
    .add_transmission(
      @urdf.Transmission::simple("t", "j", "actuator", reduction=10.0),
    )
  assert_eq(robot.links.length(), 3)
  assert_eq(robot.joints.length(), 1)
  assert_eq(robot.materials.length(), 1)
  assert_eq(robot.transmissions.length(), 1)
  assert_eq(robot.get_link("b").unwrap().name, "b")
  assert_true(robot.get_link("missing") is None)
  assert_true(robot.get_joint("j") is Some(_))
  let copy = robot.with_version("1.0")
  assert_true(robot.version is None)
  assert_eq(copy.version, Some("1.0"))
  let with_extra = robot.add_extra(@urdf.RawXmlNode::{
    tag: "gazebo",
    attrs: [("reference", "base")],
    text: "",
    children: [],
  })
  assert_eq(robot.extra.length(), 0)
  assert_eq(with_extra.extra.length(), 1)
}

///|
fn mate_near(a : Double, b : Double) -> Bool {
  (a - b).abs() < 1.0e-9
}

///|
fn mate_near_vec(
  a : (Double, Double, Double),
  b : (Double, Double, Double),
) -> Bool {
  mate_near(a.0, b.0) && mate_near(a.1, b.1) && mate_near(a.2, b.2)
}

///|
fn empty_robot() -> @urdf.Robot {
  @urdf.Robot::new("robot")
}

///|
test "mate connector normalizes its axis" {
  let m = @urdf.MateConnector::from_mm_deg("m", x=100.0, axis=(0.0, 0.0, 5.0))
  assert_true(mate_near(m.origin.xyz.0, 0.1))
  assert_true(mate_near_vec(m.axis, (0.0, 0.0, 1.0)))
}

///|
test "snap mates applies offset along the parent axis" {
  let (origin, axis) = @urdf.snap_mates(
    @urdf.MateConnector::new("p"),
    @urdf.MateConnector::new("c"),
    offset=0.1,
  )
  assert_true(mate_near_vec(origin.xyz, (0.0, 0.0, 0.1)))
  assert_true(mate_near_vec(axis, (0.0, 0.0, 1.0)))
}

///|
test "snap mates inverts the child mate frame" {
  let (origin, axis) = @urdf.snap_mates(
    @urdf.MateConnector::new("p"),
    @urdf.MateConnector::from_mm_deg("c", z=100.0),
  )
  assert_true(mate_near_vec(origin.xyz, (0.0, 0.0, -0.1)))
  assert_true(mate_near_vec(axis, (0.0, 0.0, 1.0)))
}

///|
test "snap mates supports twist and axis flip" {
  let (origin, axis) = @urdf.snap_mates(
    @urdf.MateConnector::new("p"),
    @urdf.MateConnector::new("c"),
    offset=0.2,
    angle_rad=@math.PI / 2.0,
    flip_axis=true,
  )
  assert_true(mate_near_vec(origin.xyz, (0.0, 0.0, 0.2)))
  assert_true(mate_near_vec(axis, (0.0, 0.0, -1.0)))
  let rotated = @urdf.rotation_matrix_rpy(
    origin.rpy.0,
    origin.rpy.1,
    origin.rpy.2,
  )
  assert_true(mate_near_vec(rotated.apply((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0)))
}

///|
test "joint from mates carries limit and dynamics" {
  let j = @urdf.Joint::from_mates(
    "hip",
    @urdf.JointType::Revolute,
    parent_link="body",
    parent_mate=@urdf.MateConnector::from_mm_deg("hip_port", x=50.0),
    child_link="leg",
    child_mate=@urdf.MateConnector::new("leg_port"),
    limit=Some(@urdf.JointLimit::new(effort=10.0, velocity=1.0)),
    dynamics=Some(@urdf.JointDynamics::new(damping=0.2)),
  )
  assert_eq(j.parent, "body")
  assert_eq(j.child, "leg")
  assert_true(mate_near_vec(j.origin.xyz, (0.05, 0.0, 0.0)))
  assert_true(j.limit is Some(_))
  assert_true(j.dynamics is Some(_))
}

///|
test "rebase to mate shifts all link frames" {
  let link = @urdf.Link::new("leg")
    .add_visual(
      @urdf.Visual::new(
        @urdf.Geometry::box_mm(10.0, 10.0, 10.0),
        origin=@urdf.Origin::from_mm_deg(x=200.0),
      ),
    )
    .add_collision(
      @urdf.Collision::new(
        @urdf.Geometry::box_mm(10.0, 10.0, 10.0),
        origin=@urdf.Origin::from_mm_deg(x=300.0),
      ),
    )
    .with_inertial(
      @urdf.Inertial::from_box(
        0.01,
        0.01,
        0.01,
        mass=1.0,
        origin=@urdf.Origin::from_mm_deg(x=400.0),
      ),
    )
  let rebased = link.rebase_to_mate(
    @urdf.MateConnector::from_mm_deg("mount", x=100.0),
  )
  assert_true(mate_near(rebased.visuals[0].origin.xyz.0, 0.1))
  assert_true(mate_near(rebased.collisions[0].origin.xyz.0, 0.2))
  match rebased.inertial {
    Some(i) => assert_true(mate_near(i.origin.xyz.0, 0.3))
    None => fail("expected inertial")
  }
  // The original link is untouched.
  assert_true(mate_near(link.visuals[0].origin.xyz.0, 0.2))
}

///|
test "forward kinematics of a planar arm" {
  let robot = empty_robot()
    .add_links([
      @urdf.Link::new("base"),
      @urdf.Link::new("link1"),
      @urdf.Link::new("link2"),
      @urdf.Link::new("tool"),
    ])
    .add_joints([
      @urdf.Joint::revolute(
        "j1",
        parent="base",
        child="link1",
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ),
      @urdf.Joint::revolute(
        "j2",
        parent="link1",
        child="link2",
        origin=@urdf.Origin::new(xyz=(1.0, 0.0, 0.0)),
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ),
      @urdf.Joint::fixed(
        "j3",
        parent="link2",
        child="tool",
        origin=@urdf.Origin::new(xyz=(1.0, 0.0, 0.0)),
      ),
    ])
  let q = Map([])
  q.set("j1", @math.PI / 2.0)
  q.set("j2", 0.0)
  let poses = robot.forward_kinematics(joint_positions=q)
  assert_true(mate_near_vec(poses.get("base").unwrap().trans, (0.0, 0.0, 0.0)))
  assert_true(mate_near_vec(poses.get("link2").unwrap().trans, (0.0, 1.0, 0.0)))
  assert_true(mate_near_vec(poses.get("tool").unwrap().trans, (0.0, 2.0, 0.0)))
  let q2 = Map([])
  q2.set("j1", @math.PI / 2.0)
  q2.set("j2", -@math.PI / 2.0)
  let poses2 = robot.forward_kinematics(joint_positions=q2)
  assert_true(mate_near_vec(poses2.get("tool").unwrap().trans, (1.0, 1.0, 0.0)))
}

///|
test "forward kinematics resolves mimic joints" {
  let robot = empty_robot()
    .add_links([
      @urdf.Link::new("base"),
      @urdf.Link::new("link1"),
      @urdf.Link::new("link2"),
    ])
    .add_joints([
      @urdf.Joint::revolute(
        "j1",
        parent="base",
        child="link1",
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ),
      @urdf.Joint::revolute(
        "j2",
        parent="link1",
        child="link2",
        origin=@urdf.Origin::new(xyz=(1.0, 0.0, 0.0)),
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ).with_mimic(@urdf.JointMimic::new("j1", multiplier=-2.0)),
    ])
  let q = Map([])
  q.set("j1", @math.PI / 2.0)
  let poses = robot.forward_kinematics(joint_positions=q)
  let link2 = poses.get("link2").unwrap()
  assert_true(mate_near_vec(link2.trans, (0.0, 1.0, 0.0)))
  assert_true(mate_near_vec(link2.apply_dir((1.0, 0.0, 0.0)), (0.0, -1.0, 0.0)))
}

///|
test "forward kinematics applies mimic offset" {
  let robot = empty_robot()
    .add_links([
      @urdf.Link::new("base"),
      @urdf.Link::new("link1"),
      @urdf.Link::new("link2"),
    ])
    .add_joints([
      @urdf.Joint::revolute(
        "j1",
        parent="base",
        child="link1",
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ),
      @urdf.Joint::revolute(
        "j2",
        parent="link1",
        child="link2",
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ).with_mimic(@urdf.JointMimic::new("j1", multiplier=-2.0, offset=0.5)),
    ])
  let q = Map([])
  q.set("j1", 1.0)
  let poses = robot.forward_kinematics(joint_positions=q)
  let link2 = poses.get("link2").unwrap()
  let angle = -0.5
  assert_true(
    mate_near_vec(
      link2.apply_dir((1.0, 0.0, 0.0)),
      (@math.cos(angle), @math.sin(angle), 0.0),
    ),
  )
}

///|
test "link world origins returns origins" {
  let robot = empty_robot()
    .add_link(@urdf.Link::new("base"))
    .add_link(@urdf.Link::new("arm"))
    .add_joint(
      @urdf.Joint::fixed(
        "j",
        parent="base",
        child="arm",
        origin=@urdf.Origin::from_mm_deg(z=100.0),
      ),
    )
  let origins = robot.link_world_origins()
  assert_true(mate_near(origins.get("arm").unwrap().xyz.2, 0.1))
}

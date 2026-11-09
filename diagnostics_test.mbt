///|
fn has_diag(
  diagnostics : Array[@urdf.Diagnostic],
  target : @urdf.Diagnostic,
) -> Bool {
  diagnostics.any(fn(d) { d == target })
}

///|
test "empty robot and root link" {
  let robot = @urdf.Robot::new("r")
  let ds = robot.diagnostics()
  assert_true(has_diag(ds, @urdf.Diagnostic::EmptyRobot("r")))
  assert_true(robot.root_link() is None)
  let chain = @urdf.Robot::new("chain")
    .add_links([
      @urdf.Link::new("a"),
      @urdf.Link::new("b"),
      @urdf.Link::new("c"),
    ])
    .add_joints([
      @urdf.Joint::fixed("j1", parent="a", child="b"),
      @urdf.Joint::fixed("j2", parent="b", child="c"),
    ])
  assert_eq(chain.root_link(), Some("a"))
}

///|
test "duplicate names and missing references" {
  let robot = @urdf.Robot::new("r")
    .add_links([@urdf.Link::new("a"), @urdf.Link::new("a")])
    .add_joints([
      @urdf.Joint::fixed("j", parent="ghost", child="a"),
      @urdf.Joint::fixed("j", parent="a", child="a"),
    ])
  let ds = robot.diagnostics()
  assert_true(has_diag(ds, @urdf.Diagnostic::DuplicateLinkName("a")))
  assert_true(has_diag(ds, @urdf.Diagnostic::DuplicateJointName("j")))
  assert_true(has_diag(ds, @urdf.Diagnostic::MissingParentLink("j", "ghost")))
  assert_true(has_diag(ds, @urdf.Diagnostic::SelfLoopJoint("j", "a")))
}

///|
test "missing child and multiple parents" {
  let robot = @urdf.Robot::new("r")
    .add_links([
      @urdf.Link::new("a"),
      @urdf.Link::new("b"),
      @urdf.Link::new("c"),
    ])
    .add_joints([
      @urdf.Joint::fixed("j1", parent="a", child="c"),
      @urdf.Joint::fixed("j2", parent="b", child="c"),
      @urdf.Joint::fixed("j3", parent="a", child="ghost"),
    ])
  let ds = robot.diagnostics()
  assert_true(has_diag(ds, @urdf.Diagnostic::MultipleParentJoints("c")))
  assert_true(has_diag(ds, @urdf.Diagnostic::MissingChildLink("j3", "ghost")))
  assert_true(ds.any(fn(d) { d.severity() == @urdf.Severity::Error }))
}

///|
test "kinematic cycles and disconnected graphs" {
  let cycle = @urdf.Robot::new("cycle")
    .add_links([@urdf.Link::new("a"), @urdf.Link::new("b")])
    .add_joints([
      @urdf.Joint::fixed("j1", parent="a", child="b"),
      @urdf.Joint::fixed("j2", parent="b", child="a"),
    ])
  let cds = cycle.diagnostics()
  assert_true(
    cds.any(fn(d) {
      match d {
        @urdf.Diagnostic::KinematicCycle(_) => true
        _ => false
      }
    }),
  )
  let split = @urdf.Robot::new("split").add_links([
    @urdf.Link::new("a"),
    @urdf.Link::new("b"),
  ])
  let sds = split.diagnostics()
  assert_true(has_diag(sds, @urdf.Diagnostic::DisconnectedGraph("b")))
}

///|
test "joint axis and limit checks" {
  let robot = @urdf.Robot::new("r")
    .add_links([@urdf.Link::new("a"), @urdf.Link::new("b")])
    .add_joints([
      @urdf.Joint::revolute(
        "zero_axis",
        parent="a",
        child="b",
        axis=(0.0, 0.0, 0.0),
        limit=@urdf.JointLimit::new(effort=1.0, velocity=1.0),
      ),
      @urdf.Joint::new(
        "no_limit",
        @urdf.JointType::Revolute,
        parent="a",
        child="b",
      ),
      @urdf.Joint::prismatic(
        "bad_limit",
        parent="a",
        child="b",
        limit=@urdf.JointLimit::new(
          lower=Some(1.0),
          upper=Some(0.0),
          effort=-1.0,
          velocity=-1.0,
        ),
      ),
    ])
  let ds = robot.diagnostics()
  assert_true(has_diag(ds, @urdf.Diagnostic::ZeroJointAxis("zero_axis")))
  assert_true(has_diag(ds, @urdf.Diagnostic::MissingJointLimit("no_limit")))
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::InvalidJointLimit(name, _) => name == "bad_limit"
        _ => false
      }
    }),
  )
}

///|
test "invalid mimic joints" {
  let robot = @urdf.Robot::new("r")
    .add_links([@urdf.Link::new("a"), @urdf.Link::new("b")])
    .add_joints([
      @urdf.Joint::continuous("j1", parent="a", child="b").with_mimic(
        @urdf.JointMimic::new("missing"),
      ),
      @urdf.Joint::continuous("j2", parent="a", child="b").with_mimic(
        @urdf.JointMimic::new("j3"),
      ),
      @urdf.Joint::continuous("j3", parent="a", child="b").with_mimic(
        @urdf.JointMimic::new("j2"),
      ),
    ])
  let ds = robot.diagnostics()
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::InvalidMimicJoint(name, _) => name == "j1"
        _ => false
      }
    }),
  )
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::InvalidMimicJoint(_, detail) =>
          detail.contains("cycle")
        _ => false
      }
    }),
  )
}

///|
test "geometry mass and inertia warnings" {
  let robot = @urdf.Robot::new("r")
    .add_link(
      @urdf.Link::new("bad_geom").add_visual(
        @urdf.Visual::new(@urdf.Geometry::box(0.0, 1.0, 1.0)),
      ),
    )
    .add_link(
      @urdf.Link::new("bad_mass").with_inertial(
        @urdf.Inertial::new(0.0, @urdf.Inertia::diagonal(1.0, 1.0, 1.0)),
      ),
    )
    .add_link(
      @urdf.Link::new("bad_inertia").with_inertial(
        @urdf.Inertial::new(1.0, @urdf.Inertia::diagonal(1.0, 1.0, 3.0)),
      ),
    )
  let ds = robot.diagnostics()
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::InvalidGeometry(link, _) => link == "bad_geom"
        _ => false
      }
    }),
  )
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::NonPositiveMass(link, _) => link == "bad_mass"
        _ => false
      }
    }),
  )
  assert_true(
    ds.any(fn(d) {
      match d {
        @urdf.Diagnostic::NonPhysicalInertia(link, _) => link == "bad_inertia"
        _ => false
      }
    }),
  )
  assert_true(ds.any(fn(d) { d.severity() == @urdf.Severity::Warning }))
}

///|
test "severity helpers and checked output" {
  let error = @urdf.Severity::Error
  let warning = @urdf.Severity::Warning
  assert_true(error.is_error())
  assert_false(warning.is_error())
  assert_eq(@urdf.Severity::all().length(), 3)
  let valid = @urdf.Robot::new("r").add_link(@urdf.Link::new("a"))
  assert_false(valid.has_errors())
  assert_true(valid.to_urdf_file_checked().contains("<link name=\"a\"/>"))
  let invalid = @urdf.Robot::new("r")
  assert_true(invalid.has_errors())
}

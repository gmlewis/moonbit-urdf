///|
/// 04-six-dof-arm
///
/// A parameterized six-axis revolute arm. Links use `Inertial::combine` to
/// merge motor and tube masses, joints carry safety controllers, and every
/// axis has a simple transmission.
///
/// Usage:
///   ./run-example.sh 04 -o arm.urdf
///   ./run-example.sh 04 --upperArm 320 --forearm 260 --wrist 90
fn safety(limit_value : Double) -> @urdf.SafetyController {
  {
    soft_lower_limit: -limit_value * 0.95,
    soft_upper_limit: limit_value * 0.95,
    k_position: 5.0,
    k_velocity: 1.0,
  }
}

///|
fn cylinder_link(
  name : String,
  radius : Double,
  length : Double,
  z : Double,
  color : @urdf.Rgba,
) -> @urdf.Link {
  let origin = @urdf.Origin::new(xyz=(0.0, 0.0, z))
  @urdf.Link::new(name)
  .with_inertial(
    @urdf.Inertial::from_cylinder_density(
      radius,
      length,
      density=@urdf.Density::aluminum_6061(),
      origin~,
    ),
  )
  .add_visual(
    @urdf.Visual::new(@urdf.Geometry::cylinder(radius, length))
    .with_origin(origin)
    .with_color(name, color),
  )
  .add_collision(
    @urdf.Collision::new(@urdf.Geometry::cylinder(radius, length)).with_origin(
      origin,
    ),
  )
}

///|
fn arm_joint(
  name : String,
  parent : String,
  child : String,
  z : Double,
  axis : (Double, Double, Double),
  limit_value : Double,
  effort : Double,
  velocity : Double,
) -> @urdf.Joint {
  @urdf.Joint::revolute(
    name,
    parent~,
    child~,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, z)),
    axis~,
    limit=@urdf.JointLimit::new(
      lower=Some(-limit_value),
      upper=Some(limit_value),
      effort~,
      velocity~,
    ),
  ).with_safety_controller(safety(limit_value))
}

///|
async fn main {
  let spec = @common.with_output_args({
    "baseHeight": @cli.opt_double(
      120.0,
      help="Base column height in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "upperArm": @cli.opt_double(
      300.0,
      help="Upper arm length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "forearm": @cli.opt_double(
      250.0,
      help="Forearm length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "wrist": @cli.opt_double(
      80.0,
      help="Wrist link length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="04-six-dof-arm",
      description="Generate a six-DOF revolute arm URDF.",
    )
    is Some(args) else {
    return
  }
  let base_h = args.double("baseHeight") * 0.001
  let upper = args.double("upperArm") * 0.001
  let forearm = args.double("forearm") * 0.001
  let wrist = args.double("wrist") * 0.001
  let base_link = cylinder_link(
    "base_link",
    0.08,
    base_h,
    base_h / 2.0,
    @urdf.Rgba::grey(),
  )
  let shoulder_link = cylinder_link(
    "shoulder_link",
    0.05,
    0.1,
    0.05,
    @urdf.Rgba::silver(),
  )
  let upper_inertial = @urdf.Inertial::combine([
    @urdf.Inertial::from_cylinder(0.04, 0.06, mass=0.5),
    @urdf.Inertial::from_tube(
      0.012,
      0.02,
      upper,
      mass=0.9,
      origin=@urdf.Origin::new(xyz=(0.0, 0.0, upper / 2.0)),
    ),
  ])
  let upper_arm = @urdf.Link::new("upper_arm")
    .with_inertial(upper_inertial)
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(0.025, upper))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, upper / 2.0)))
      .with_color("upper_arm", @urdf.Rgba::orange()),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.025, upper)).with_origin(
        @urdf.Origin::new(xyz=(0.0, 0.0, upper / 2.0)),
      ),
    )
  let forearm_inertial = @urdf.Inertial::combine([
    @urdf.Inertial::from_cylinder(0.035, 0.05, mass=0.4),
    @urdf.Inertial::from_tube(
      0.01,
      0.018,
      forearm,
      mass=0.6,
      origin=@urdf.Origin::new(xyz=(0.0, 0.0, forearm / 2.0)),
    ),
  ])
  let forearm_link = @urdf.Link::new("forearm")
    .with_inertial(forearm_inertial)
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(0.022, forearm))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, forearm / 2.0)))
      .with_color("forearm", @urdf.Rgba::blue()),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.022, forearm)).with_origin(
        @urdf.Origin::new(xyz=(0.0, 0.0, forearm / 2.0)),
      ),
    )
  let wrist_1 = cylinder_link(
    "wrist_1",
    0.03,
    wrist,
    wrist / 2.0,
    @urdf.Rgba::purple(),
  )
  let wrist_2 = cylinder_link(
    "wrist_2",
    0.026,
    wrist,
    wrist / 2.0,
    @urdf.Rgba::cyan(),
  )
  let ee_link = @urdf.Link::new("ee_link")
    .with_inertial(
      @urdf.Inertial::from_box_density(
        0.06,
        0.06,
        0.02,
        density=@urdf.Density::aluminum_6061(),
        origin=@urdf.Origin::new(xyz=(0.0, 0.0, wrist)),
      ),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(0.06, 0.06, 0.02))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, wrist)))
      .with_color("ee", @urdf.Rgba::red()),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::box(0.06, 0.06, 0.02)).with_origin(
        @urdf.Origin::new(xyz=(0.0, 0.0, wrist)),
      ),
    )
  let joints = [
    arm_joint(
      "joint_1",
      "base_link",
      "shoulder_link",
      base_h,
      (0.0, 0.0, 1.0),
      3.1,
      150.0,
      2.0,
    ),
    arm_joint(
      "joint_2",
      "shoulder_link",
      "upper_arm",
      0.1,
      (0.0, 1.0, 0.0),
      2.0,
      150.0,
      2.0,
    ),
    arm_joint(
      "joint_3",
      "upper_arm",
      "forearm",
      upper,
      (0.0, 1.0, 0.0),
      2.5,
      100.0,
      2.5,
    ),
    arm_joint(
      "joint_4",
      "forearm",
      "wrist_1",
      forearm,
      (0.0, 0.0, 1.0),
      3.1,
      40.0,
      3.0,
    ),
    arm_joint(
      "joint_5",
      "wrist_1",
      "wrist_2",
      wrist,
      (0.0, 1.0, 0.0),
      2.0,
      30.0,
      3.0,
    ),
    arm_joint(
      "joint_6",
      "wrist_2",
      "ee_link",
      wrist,
      (0.0, 0.0, 1.0),
      3.1,
      20.0,
      3.0,
    ),
  ]
  let robot = @urdf.Robot::new(
      "six_dof_arm",
      documentation=args.documentation(),
    )
    .add_links([
      base_link, shoulder_link, upper_arm, forearm_link, wrist_1, wrist_2, ee_link,
    ])
    .add_joints(joints)
  let mut robot = robot
  for joint in joints {
    robot = robot.add_transmission(
      @urdf.Transmission::simple(
        "transmission_" + joint.name,
        joint.name,
        "motor_" + joint.name,
        reduction=50.0,
      ),
    )
  }
  @common.write_output(robot, @common.output_paths(args))
}

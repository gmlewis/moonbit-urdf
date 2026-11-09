///|
/// 06-assistive-scooter
///
/// An assistive lightweight wheelchair/scooter parameterized by the rider's
/// seat width and leg length. The frame, seat, and battery form one composite
/// rigid body; the front column steers and all three wheels are continuous
/// joints.
///
/// Usage:
///   ./run-example.sh 06 -o scooter.urdf
///   ./run-example.sh 06 --seatWidth 420 --legLength 450 --wheelDiameter 300
async fn main {
  let spec = @common.with_output_args({
    "seatWidth": @cli.opt_double(
      420.0,
      help="Seat width in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "legLength": @cli.opt_double(
      450.0,
      help="Steering column length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "wheelbase": @cli.opt_double(
      900.0,
      help="Distance between front and rear axles in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "trackWidth": @cli.opt_double(
      600.0,
      help="Rear track width in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "wheelDiameter": @cli.opt_double(
      300.0,
      help="Wheel diameter in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "targetWeightKg": @cli.opt_double(
      18.0,
      help="Target vehicle mass in kilograms",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="06-assistive-scooter",
      description="Generate an assistive wheelchair/scooter URDF.",
    )
    is Some(args) else {
    return
  }
  let seat_width = args.double("seatWidth") * 0.001
  let leg_length = args.double("legLength") * 0.001
  let wheelbase = args.double("wheelbase") * 0.001
  let track = args.double("trackWidth") * 0.001
  let wheel_radius = args.double("wheelDiameter") * 0.001 / 2.0
  let target = args.double("targetWeightKg")
  let wheel_width = 0.045
  let alu = @urdf.Density::aluminum_6061()
  let frame = @urdf.Inertial::from_tube(
    0.02,
    0.028,
    wheelbase,
    mass=target * 0.45,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.12)),
  )
  let seat = @urdf.Inertial::from_box(
    seat_width,
    seat_width * 0.7,
    0.05,
    mass=target * 0.15,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.45)),
  )
  let battery = @urdf.Inertial::from_box(
    0.25,
    0.18,
    0.1,
    mass=target * 0.25,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, 0.2)),
  )
  let base_inertial = @urdf.Inertial::combine([frame, seat, battery])
  let base_link = @urdf.Link::new("base_link")
    .with_inertial(base_inertial)
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(wheelbase, 0.06, 0.06))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, 0.12)))
      .with_color("frame", @urdf.Rgba::silver()),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(seat_width, seat_width * 0.7, 0.05))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, 0.45)))
      .with_color("seat", @urdf.Rgba::black()),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(0.25, 0.18, 0.1))
      .with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, 0.2)))
      .with_color("battery", @urdf.Rgba::green()),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::box(wheelbase, 0.06, 0.06)).with_origin(
        @urdf.Origin::new(xyz=(0.0, 0.0, 0.12)),
      ),
    )
    .add_collision(
      @urdf.Collision::new(
        @urdf.Geometry::box(seat_width, seat_width * 0.7, 0.05),
      ).with_origin(@urdf.Origin::new(xyz=(0.0, 0.0, 0.45))),
    )
  let column_origin = @urdf.Origin::new(xyz=(0.0, 0.0, leg_length / 2.0))
  let steering_column = @urdf.Link::new("steering_column")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(
        0.03,
        leg_length,
        density=alu,
        origin=column_origin,
      ),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(0.03, leg_length))
      .with_origin(column_origin)
      .with_color("steering_column", @urdf.Rgba::grey()),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(0.03, leg_length)).with_origin(
        column_origin,
      ),
    )
  let wheel_geometry = @urdf.Geometry::cylinder(wheel_radius, wheel_width)
  let front_wheel = @urdf.Link::new("front_wheel")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(
        wheel_radius,
        wheel_width,
        density=@urdf.Density::pla(),
      ),
    )
    .add_visual(
      @urdf.Visual::new(wheel_geometry).with_color(
        "front_wheel",
        @urdf.Rgba::black(),
      ),
    )
    .add_collision(@urdf.Collision::new(wheel_geometry))
  let rear_left_wheel = @urdf.Link::new("rear_left_wheel")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(
        wheel_radius,
        wheel_width,
        density=@urdf.Density::pla(),
      ),
    )
    .add_visual(
      @urdf.Visual::new(wheel_geometry).with_color(
        "rear_left_wheel",
        @urdf.Rgba::black(),
      ),
    )
    .add_collision(@urdf.Collision::new(wheel_geometry))
  let rear_right_wheel = @urdf.Link::new("rear_right_wheel")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(
        wheel_radius,
        wheel_width,
        density=@urdf.Density::pla(),
      ),
    )
    .add_visual(
      @urdf.Visual::new(wheel_geometry).with_color(
        "rear_right_wheel",
        @urdf.Rgba::black(),
      ),
    )
    .add_collision(@urdf.Collision::new(wheel_geometry))
  let steering_joint = @urdf.Joint::revolute(
    "steering_joint",
    parent="base_link",
    child="steering_column",
    origin=@urdf.Origin::new(xyz=(wheelbase / 2.0, 0.0, 0.0)),
    axis=(0.0, 0.0, 1.0),
    limit=@urdf.JointLimit::new(
      lower=Some(-0.6),
      upper=Some(0.6),
      effort=20.0,
      velocity=2.0,
    ),
  ).with_dynamics(@urdf.JointDynamics::new(damping=0.2, friction=0.05))
  let front_wheel_joint = @urdf.Joint::continuous(
    "front_wheel_joint",
    parent="steering_column",
    child="front_wheel",
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, -leg_length)),
    axis=(0.0, 1.0, 0.0),
  )
  let rear_left_joint = @urdf.Joint::continuous(
    "rear_left_wheel_joint",
    parent="base_link",
    child="rear_left_wheel",
    origin=@urdf.Origin::new(xyz=(-wheelbase / 2.0, track / 2.0, -wheel_radius)),
    axis=(0.0, 1.0, 0.0),
  )
  let rear_right_joint = @urdf.Joint::continuous(
    "rear_right_wheel_joint",
    parent="base_link",
    child="rear_right_wheel",
    origin=@urdf.Origin::new(
      xyz=(-wheelbase / 2.0, -track / 2.0, -wheel_radius),
    ),
    axis=(0.0, 1.0, 0.0),
  )
  let robot = @urdf.Robot::new(
      "assistive_scooter",
      documentation=args.documentation(),
    )
    .add_links([
      base_link, steering_column, front_wheel, rear_left_wheel, rear_right_wheel,
    ])
    .add_joints([
      steering_joint, front_wheel_joint, rear_left_joint, rear_right_joint,
    ])
  let mut robot = robot
  for
    joint in [
      steering_joint, front_wheel_joint, rear_left_joint, rear_right_joint,
    ] {
    robot = robot.add_transmission(
      @urdf.Transmission::simple(
        "transmission_" + joint.name,
        joint.name,
        "motor_" + joint.name,
        reduction=15.0,
      ),
    )
  }
  @common.write_output(robot, @common.output_paths(args))
}

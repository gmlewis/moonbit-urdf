///|
/// 02-two-link-pendulum
///
/// Two revolute links assembled through `MateConnector` snapping, with joint
/// limits and damping/friction dynamics.
///
/// Usage:
///   ./run-example.sh 02 -o pendulum.urdf
///   ./run-example.sh 02 --length1 200 --length2 150 --radius 15
async fn main {
  let spec = @common.with_output_args({
    "length1": @cli.opt_double(
      250.0,
      help="First link length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "length2": @cli.opt_double(
      200.0,
      help="Second link length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "radius": @cli.opt_double(
      15.0,
      help="Link radius in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="02-two-link-pendulum",
      description="Generate a two-link revolute pendulum URDF.",
    )
    is Some(args) else {
    return
  }
  let length1_mm = args.double("length1")
  let length2_mm = args.double("length2")
  let radius_mm = args.double("radius")
  let length1 = length1_mm * 0.001
  let length2 = length2_mm * 0.001
  let radius = radius_mm * 0.001
  let rho = @urdf.Density::pla()
  let base_link = @urdf.Link::new("base_link")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(radius * 2.5, 0.06, density=rho),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(radius * 2.5, 0.06)).with_color(
        "base",
        @urdf.Rgba::grey(),
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(radius * 2.5, 0.06)),
    )
  let link1 = @urdf.Link::new("link1")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(radius, length1, density=rho),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(radius, length1)).with_color(
        "link1",
        @urdf.Rgba::orange(),
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(radius, length1)),
    )
  let link2 = @urdf.Link::new("link2")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(radius, length2, density=rho),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(radius, length2)).with_color(
        "link2",
        @urdf.Rgba::blue(),
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(radius, length2)),
    )
  let base_mate = @urdf.MateConnector::from_mm_deg(
    "shoulder",
    axis=(0.0, 1.0, 0.0),
  )
  let link1_top = @urdf.MateConnector::from_mm_deg(
    "top",
    z=length1_mm / 2.0,
    axis=(0.0, 1.0, 0.0),
  )
  let link1_bottom = @urdf.MateConnector::from_mm_deg(
    "bottom",
    z=-length1_mm / 2.0,
    axis=(0.0, 1.0, 0.0),
  )
  let link2_top = @urdf.MateConnector::from_mm_deg(
    "top",
    z=length2_mm / 2.0,
    axis=(0.0, 1.0, 0.0),
  )
  let shoulder_joint = @urdf.Joint::from_mates(
    "shoulder_joint",
    @urdf.JointType::Revolute,
    parent_link="base_link",
    parent_mate=base_mate,
    child_link="link1",
    child_mate=link1_top,
    limit=Some(
      @urdf.JointLimit::new(
        lower=Some(-1.5),
        upper=Some(1.5),
        effort=10.0,
        velocity=3.0,
      ),
    ),
    dynamics=Some(@urdf.JointDynamics::new(damping=0.1, friction=0.02)),
  )
  let elbow_joint = @urdf.Joint::from_mates(
    "elbow_joint",
    @urdf.JointType::Revolute,
    parent_link="link1",
    parent_mate=link1_bottom,
    child_link="link2",
    child_mate=link2_top,
    limit=Some(
      @urdf.JointLimit::new(
        lower=Some(-2.0),
        upper=Some(2.0),
        effort=8.0,
        velocity=4.0,
      ),
    ),
    dynamics=Some(@urdf.JointDynamics::new(damping=0.05, friction=0.01)),
  )
  let robot = @urdf.Robot::new(
      "two_link_pendulum",
      documentation=args.documentation(),
    )
    .add_links([base_link, link1, link2])
    .add_joints([shoulder_joint, elbow_joint])
  @common.write_output(robot, @common.output_paths(args))
}

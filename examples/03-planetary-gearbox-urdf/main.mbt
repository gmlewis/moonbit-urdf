///|
/// 03-planetary-gearbox-urdf
///
/// A parameterized planetary gearbox with sun, ring, orbiting planets, and a
/// carrier output. Planet and carrier rotations are coupled to the sun joint
/// with `JointMimic`.
///
/// Usage:
///   ./run-example.sh 03 -o gearbox.urdf
///   ./run-example.sh 03 --module 2 --sunTeeth 12 --planetTeeth 18 --numPlanets 4
async fn main {
  let spec = @common.with_output_args({
    "module": @cli.opt_double(
      1.5,
      help="Gear module in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "sunTeeth": @cli.opt_int(12, help="Number of sun gear teeth", min=6),
    "planetTeeth": @cli.opt_int(18, help="Number of planet gear teeth", min=6),
    "numPlanets": @cli.opt_int(3, help="Number of planet gears", min=1),
    "gearLength": @cli.opt_double(
      10.0,
      help="Gear face width in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="03-planetary-gearbox-urdf",
      description="Generate a parameterized planetary gearbox URDF.",
    )
    is Some(args) else {
    return
  }
  let module_m = args.double("module") * 0.001
  let sun_teeth = args.int("sunTeeth").to_double()
  let planet_teeth = args.int("planetTeeth").to_double()
  let num_planets = args.int("numPlanets")
  let gear_length = args.double("gearLength") * 0.001
  let sun_r = module_m * sun_teeth / 2.0
  let planet_r = module_m * planet_teeth / 2.0
  let ring_teeth = sun_teeth + 2.0 * planet_teeth
  let ring_r = module_m * ring_teeth / 2.0
  let orbit_r = sun_r + planet_r
  let carrier_ratio = 1.0 / (1.0 + ring_teeth / sun_teeth)
  let steel = @urdf.Density::steel_mild()
  let alu = @urdf.Density::aluminum_6061()
  let ring_housing = @urdf.Link::new("ring_housing")
    .with_inertial(
      @urdf.Inertial::from_tube_density(
        ring_r,
        ring_r * 1.25,
        gear_length,
        density=alu,
      ),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(ring_r * 1.25, gear_length)).with_color(
        "ring",
        @urdf.Rgba::silver(),
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(ring_r * 1.25, gear_length)),
    )
  let sun_gear = @urdf.Link::new("sun_gear")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(
        sun_r * 0.9,
        gear_length * 0.7,
        density=steel,
      ),
    )
    .add_visual(
      @urdf.Visual::new(
        @urdf.Geometry::cylinder(sun_r * 0.9, gear_length * 0.7),
      ).with_color("sun", @urdf.Rgba::yellow()),
    )
    .add_collision(
      @urdf.Collision::new(
        @urdf.Geometry::cylinder(sun_r * 0.9, gear_length * 0.7),
      ),
    )
  let carrier = @urdf.Link::new("carrier")
    .with_inertial(
      @urdf.Inertial::from_cylinder_density(ring_r * 0.35, 0.012, density=alu),
    )
    .add_visual(
      @urdf.Visual::new(@urdf.Geometry::cylinder(ring_r * 0.35, 0.012)).with_color(
        "carrier",
        @urdf.Rgba::orange(),
      ),
    )
    .add_collision(
      @urdf.Collision::new(@urdf.Geometry::cylinder(ring_r * 0.35, 0.012)),
    )
  let ring_input = @urdf.MateConnector::new("ring_input")
  let input_shaft = @urdf.MateConnector::new("input_shaft")
  let carrier_mount = @urdf.MateConnector::new("carrier_mount")
  let output_flange = @urdf.MateConnector::new("output_flange")
  let sun_joint = @urdf.Joint::from_mates(
    "sun_joint",
    @urdf.JointType::Revolute,
    parent_link="ring_housing",
    parent_mate=ring_input,
    child_link="sun_gear",
    child_mate=input_shaft,
    limit=Some(
      @urdf.JointLimit::new(
        lower=Some(-100.0),
        upper=Some(100.0),
        effort=20.0,
        velocity=50.0,
      ),
    ),
    dynamics=Some(@urdf.JointDynamics::new(damping=0.01, friction=0.005)),
  )
  let carrier_joint = @urdf.Joint::from_mates(
    "carrier_joint",
    @urdf.JointType::Revolute,
    parent_link="ring_housing",
    parent_mate=carrier_mount,
    child_link="carrier",
    child_mate=output_flange,
    limit=Some(
      @urdf.JointLimit::new(
        lower=Some(-100.0),
        upper=Some(100.0),
        effort=40.0,
        velocity=20.0,
      ),
    ),
  ).with_mimic(@urdf.JointMimic::new("sun_joint", multiplier=carrier_ratio))
  let robot = @urdf.Robot::new(
      "planetary_gearbox",
      documentation=args.documentation(),
    )
    .add_links([ring_housing, sun_gear, carrier])
    .add_joint(sun_joint)
    .add_joint(carrier_joint)
  let mut robot = robot
  let planet_multiplier = -sun_teeth / planet_teeth
  for i in 0..<num_planets {
    let name = "planet_\{i}".to_string()
    let angle = 2.0 * @math.PI * i.to_double() / num_planets.to_double()
    let planet = @urdf.Link::new(name)
      .with_inertial(
        @urdf.Inertial::from_cylinder_density(
          planet_r * 0.9,
          gear_length * 0.7,
          density=steel,
        ),
      )
      .add_visual(
        @urdf.Visual::new(
          @urdf.Geometry::cylinder(planet_r * 0.9, gear_length * 0.7),
        ).with_color("planet", @urdf.Rgba::blue()),
      )
      .add_collision(
        @urdf.Collision::new(
          @urdf.Geometry::cylinder(planet_r * 0.9, gear_length * 0.7),
        ),
      )
    let joint = @urdf.Joint::revolute(
      "planet_\{i}_joint",
      parent="carrier",
      child=name,
      origin=@urdf.Origin::new(
        xyz=(@math.cos(angle) * orbit_r, @math.sin(angle) * orbit_r, 0.0),
      ),
      axis=(0.0, 0.0, 1.0),
      limit=@urdf.JointLimit::new(
        lower=Some(-200.0),
        upper=Some(200.0),
        effort=15.0,
        velocity=60.0,
      ),
    ).with_mimic(
      @urdf.JointMimic::new("sun_joint", multiplier=planet_multiplier),
    )
    robot = robot.add_link(planet).add_joint(joint)
  }
  @common.write_output(robot, @common.output_paths(args))
}

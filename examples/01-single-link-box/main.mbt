///|
/// 01-single-link-box
///
/// A parameterized single-link box demonstrating millimeter inputs, material
/// density, analytical inertia, and visual/collision geometry.
///
/// Usage:
///   ./run-example.sh 01 -o box.urdf
///   ./run-example.sh 01 --sx 200 --sy 100 --sz 50 --density 2700
async fn main {
  let spec = @common.with_output_args({
    "sx": @cli.opt_double(
      100.0,
      help="Box X size in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "sy": @cli.opt_double(
      60.0,
      help="Box Y size in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "sz": @cli.opt_double(
      40.0,
      help="Box Z size in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "density": @cli.opt_double(
      1240.0,
      help="Material density in kg/m^3",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="01-single-link-box",
      description="Generate a parameterized single-link box URDF.",
    )
    is Some(args) else {
    return
  }
  let sx = args.double("sx") * 0.001
  let sy = args.double("sy") * 0.001
  let sz = args.double("sz") * 0.001
  let density = @urdf.Density::kg_per_m3(args.double("density"))
  let geometry = @urdf.Geometry::box(sx, sy, sz)
  let material = @urdf.Material::color_only("body", @urdf.Rgba::orange())
  let robot = @urdf.Robot::new(
    "single_link_box",
    documentation=args.documentation(),
  ).add_link(
    @urdf.Link::new("box")
    .with_inertial(@urdf.Inertial::from_box_density(sx, sy, sz, density~))
    .add_visual(
      @urdf.Visual::new(geometry)
      .with_name("box_visual")
      .with_material(material),
    )
    .add_collision(@urdf.Collision::new(geometry).with_name("box_collision")),
  )
  @common.write_output(robot, @common.output_paths(args))
}

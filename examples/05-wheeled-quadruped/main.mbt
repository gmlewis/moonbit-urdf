///|
/// 05-wheeled-quadruped
///
/// The flagship Galileo-style wheeled quadruped: a composite chassis with four
/// named hip mate connectors, four legs (abduction, flexion, knee, wheel), and
/// a full ROS package bundle with placeholder glTF mesh assets.
///
/// Usage:
///   ./run-example.sh 05 -o galileo.urdf
///   ./run-example.sh 05 --bundle /tmp/galileo_pkg
///   python3 scripts/validate_urdf.py --check-meshes /tmp/galileo_pkg/urdf/galileo_quadruped.urdf
fn hip_link(name : String) -> @urdf.Link {
  let geometry = @urdf.Geometry::cylinder(0.035, 0.08)
  @urdf.Link::new(name)
  .with_inertial(
    @urdf.Inertial::from_cylinder_density(
      0.035,
      0.08,
      density=@urdf.Density::aluminum_6061(),
    ),
  )
  .add_visual(
    @urdf.Visual::new(
      @urdf.Geometry::mesh("package://galileo_quadruped/meshes/hip.gltf"),
    ).with_color("hip", @urdf.Rgba::silver()),
  )
  .add_collision(@urdf.Collision::new(geometry))
}

///|
fn segment_link(
  name : String,
  radius : Double,
  length : Double,
  mesh_name : String,
  color : @urdf.Rgba,
) -> @urdf.Link {
  let origin = @urdf.Origin::new(xyz=(0.0, 0.0, -length / 2.0))
  let geometry = @urdf.Geometry::cylinder(radius, length)
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
    @urdf.Visual::new(
      @urdf.Geometry::mesh("package://galileo_quadruped/meshes/" + mesh_name),
    )
    .with_origin(origin)
    .with_color(name, color),
  )
  .add_collision(@urdf.Collision::new(geometry).with_origin(origin))
}

///|
fn wheel_link(name : String, radius : Double, width : Double) -> @urdf.Link {
  let geometry = @urdf.Geometry::cylinder(radius, width)
  @urdf.Link::new(name)
  .with_inertial(
    @urdf.Inertial::from_cylinder_density(
      radius,
      width,
      density=@urdf.Density::pla(),
    ),
  )
  .add_visual(
    @urdf.Visual::new(
      @urdf.Geometry::mesh("package://galileo_quadruped/meshes/wheel.gltf"),
    ).with_color("wheel", @urdf.Rgba::black()),
  )
  .add_collision(@urdf.Collision::new(geometry))
}

///|
fn leg_joint(
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
  )
}

///|
async fn main {
  let spec = @common.with_output_args({
    "bodyLength": @cli.opt_double(
      400.0,
      help="Chassis length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "bodyWidth": @cli.opt_double(
      240.0,
      help="Chassis width in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "bodyHeight": @cli.opt_double(
      90.0,
      help="Chassis height in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "hipOffset": @cli.opt_double(
      20.0,
      help="Hip offset beyond the chassis in millimeters",
      min=0.0,
    ),
    "thighLength": @cli.opt_double(
      180.0,
      help="Thigh length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "calfLength": @cli.opt_double(
      180.0,
      help="Calf length in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "wheelRadius": @cli.opt_double(
      65.0,
      help="Wheel radius in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
    "wheelWidth": @cli.opt_double(
      35.0,
      help="Wheel width in millimeters",
      min=0.0,
      min_exclusive=true,
    ),
  })
  guard @cli.parse(
      spec,
      prog="05-wheeled-quadruped",
      description="Generate a Galileo-style wheeled quadruped URDF bundle.",
    )
    is Some(args) else {
    return
  }
  let body_length_mm = args.double("bodyLength")
  let body_width_mm = args.double("bodyWidth")
  let body_height_mm = args.double("bodyHeight")
  let hip_offset_mm = args.double("hipOffset")
  let thigh_mm = args.double("thighLength")
  let calf_mm = args.double("calfLength")
  let wheel_radius_mm = args.double("wheelRadius")
  let wheel_width_mm = args.double("wheelWidth")
  let body_length = body_length_mm * 0.001
  let body_width = body_width_mm * 0.001
  let body_height = body_height_mm * 0.001
  let thigh = thigh_mm * 0.001
  let calf = calf_mm * 0.001
  let wheel_radius = wheel_radius_mm * 0.001
  let wheel_width = wheel_width_mm * 0.001
  let alu = @urdf.Density::aluminum_6061()
  let body_part = @urdf.Inertial::from_box_density(
    body_length,
    body_width,
    body_height,
    density=alu,
  )
  let pod = @urdf.Inertial::from_cylinder_density(0.04, 0.08, density=alu)
  let pod_x = body_length / 2.0
  let pod_y = body_width / 2.0
  let base_inertial = @urdf.Inertial::combine([
    body_part,
    { ..pod, origin: @urdf.Origin::new(xyz=(pod_x, pod_y, 0.0)), },
    { ..pod, origin: @urdf.Origin::new(xyz=(pod_x, -pod_y, 0.0)), },
    { ..pod, origin: @urdf.Origin::new(xyz=(-pod_x, pod_y, 0.0)), },
    { ..pod, origin: @urdf.Origin::new(xyz=(-pod_x, -pod_y, 0.0)), },
  ])
  let base_link = @urdf.Link::new("base_link")
    .with_inertial(base_inertial)
    .add_visual(
      @urdf.Visual::new(
        @urdf.Geometry::mesh("package://galileo_quadruped/meshes/body.gltf"),
      ).with_color("chassis", @urdf.Rgba::orange()),
    )
    .add_collision(
      @urdf.Collision::new(
        @urdf.Geometry::box(body_length, body_width, body_height),
      ),
    )
  let robot = @urdf.Robot::new(
    "galileo_quadruped",
    documentation=args.documentation(),
  ).add_link(base_link)
  let mut robot = robot
  let hips : Array[(String, Double, Double)] = [
    ("fl", body_length_mm / 2.0, body_width_mm / 2.0 + hip_offset_mm),
    ("fr", body_length_mm / 2.0, -(body_width_mm / 2.0 + hip_offset_mm)),
    ("rl", -body_length_mm / 2.0, body_width_mm / 2.0 + hip_offset_mm),
    ("rr", -body_length_mm / 2.0, -(body_width_mm / 2.0 + hip_offset_mm)),
  ]
  let transmissions : Array[@urdf.Transmission] = []
  for (prefix, hx, hy) in hips {
    let hip_mate = @urdf.MateConnector::from_mm_deg(
      prefix + "_hip",
      x=hx,
      y=hy,
      axis=(1.0, 0.0, 0.0),
    )
    let hip_mount = @urdf.MateConnector::new(prefix + "_hip_mount")
    let hip_name = prefix + "_hip_link"
    let thigh_name = prefix + "_thigh_link"
    let calf_name = prefix + "_calf_link"
    let wheel_name = prefix + "_wheel_link"
    let abduction = @urdf.Joint::from_mates(
      prefix + "_hip_abduction",
      @urdf.JointType::Revolute,
      parent_link="base_link",
      parent_mate=hip_mate,
      child_link=hip_name,
      child_mate=hip_mount,
      limit=Some(
        @urdf.JointLimit::new(
          lower=Some(-0.8),
          upper=Some(0.8),
          effort=30.0,
          velocity=10.0,
        ),
      ),
    )
    let flexion = leg_joint(
      prefix + "_hip_flexion",
      hip_name,
      thigh_name,
      -0.04,
      (0.0, 1.0, 0.0),
      1.6,
      60.0,
      12.0,
    )
    let knee = leg_joint(
      prefix + "_knee_flexion",
      thigh_name,
      calf_name,
      -thigh,
      (0.0, 1.0, 0.0),
      2.4,
      60.0,
      12.0,
    )
    let drive = @urdf.Joint::continuous(
      prefix + "_wheel_drive",
      parent=calf_name,
      child=wheel_name,
      origin=@urdf.Origin::new(xyz=(0.0, 0.0, -calf)),
      axis=(0.0, 1.0, 0.0),
    )
    robot = robot
      .add_link(hip_link(hip_name))
      .add_link(
        segment_link(thigh_name, 0.025, thigh, "thigh.gltf", @urdf.Rgba::blue()),
      )
      .add_link(
        segment_link(calf_name, 0.02, calf, "calf.gltf", @urdf.Rgba::silver()),
      )
      .add_link(wheel_link(wheel_name, wheel_radius, wheel_width))
      .add_joint(abduction)
      .add_joint(flexion)
      .add_joint(knee)
      .add_joint(drive)
    for joint in [abduction, flexion, knee, drive] {
      transmissions.push(
        @urdf.Transmission::simple(
          "transmission_" + joint.name,
          joint.name,
          "motor_" + joint.name,
          reduction=10.0,
        ),
      )
    }
  }
  for transmission in transmissions {
    robot = robot.add_transmission(transmission)
  }
  let gltf = "{\"asset\":{\"version\":\"2.0\"}}\n"
  let bundle = @urdf.UrdfBundle::new("galileo_quadruped", robot)
    .add_mesh_text("meshes/body.gltf", gltf)
    .add_mesh_text("meshes/hip.gltf", gltf)
    .add_mesh_text("meshes/thigh.gltf", gltf)
    .add_mesh_text("meshes/calf.gltf", gltf)
    .add_mesh_text("meshes/wheel.gltf", gltf)
  @common.write_output(robot, @common.output_paths(args), bundle~)
}

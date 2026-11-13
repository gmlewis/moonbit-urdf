///|
/// preview_geometry maps a URDF primitive visual to a CAD shape plus the local
/// centering offset needed because URDF primitives are centred on their origin
/// while the CAD builders put boxes and cylinders on a base plane.
///
/// Mesh visuals return `None`; the preview shows them as small frame markers.
fn preview_geometry(geometry : @urdf.Geometry) -> (@cad.Shape, @urdf.Vec3)? {
  match geometry {
    @urdf.Geometry::Box((sx, sy, sz)) =>
      Some(
        (
          @cad.Cuboid::new(sx * 1000.0, sy * 1000.0, sz * 1000.0),
          (-sx / 2.0, -sy / 2.0, -sz / 2.0),
        ),
      )
    @urdf.Geometry::Cylinder(radius, length) =>
      Some(
        (
          @cad.Cylinder::new(radius * 2000.0, length * 1000.0),
          (0.0, 0.0, -length / 2.0),
        ),
      )
    @urdf.Geometry::Sphere(radius) =>
      Some(
        (
          @cad.RevolvedProfile::sphere(radius * 2000.0).as_shape(),
          (0.0, 0.0, 0.0),
        ),
      )
    @urdf.Geometry::Mesh(_, _) => None
  }
}

///|
/// visual_color returns the visual material colour, or a neutral grey.
fn visual_color(visual : @urdf.Visual) -> @cad.Rgb {
  match visual.material {
    Some(material) =>
      match material.color {
        Some(color) => @cad.Rgb::new(color.r, color.g, color.b)
        None => @cad.Rgb::grey()
      }
    None => @cad.Rgb::grey()
  }
}

///|
/// preview_node places a shape at a world URDF origin and names it.
fn preview_node(
  shape : @cad.Shape,
  world : @urdf.Origin,
  name : String,
  color : @cad.Rgb,
) -> @cad.SceneNode {
  let (x, y, z) = world.xyz
  let (rx, ry, rz) = world.rpy
  let to_deg = 180.0 / @math.PI
  shape
  .with_name(name)
  .with_color(color)
  .rotate_x(rx * to_deg)
  .rotate_y(ry * to_deg)
  .rotate_z(rz * to_deg)
  .translate(x=x * 1000.0, y=y * 1000.0, z=z * 1000.0)
}

///|
/// write_step_preview writes a STEP preview of a robot's visual geometry.
///
/// Link world poses come from zero-position forward kinematics. Box, cylinder,
/// and sphere visuals are emitted at their exact envelope sizes. Mesh visuals
/// are represented by a 20 mm frame marker at their visual origin, because the
/// preview does not load mesh files.
pub async fn write_step_preview(robot : @urdf.Robot, path : String) -> Unit {
  let poses = robot.forward_kinematics()
  let mut design = @cad.Design::new(
    name=robot.name,
    description="URDF visual preview",
  )
  let mut mesh_visuals = 0
  let mut node_count = 0
  for link in robot.links {
    let base = match poses.get(link.name) {
      Some(transform) => transform
      None => @urdf.Transform::identity()
    }
    for i, visual in link.visuals {
      let visual_world = base.compose(
        @urdf.Transform::from_origin(visual.origin),
      )
      let color = visual_color(visual)
      match preview_geometry(visual.geometry) {
        Some((shape, center)) => {
          let centered = @urdf.Transform::from_origin(
            @urdf.Origin::new(xyz=center),
          )
          let world = visual_world.compose(centered).to_origin()
          design = design.add(
            preview_node(shape, world, "\{link.name}/visual-\{i}", color),
          )
          node_count = node_count + 1
        }
        None => {
          mesh_visuals = mesh_visuals + 1
          design = design.add(
            preview_node(
              @cad.RevolvedProfile::sphere(20.0).as_shape(),
              visual_world.to_origin(),
              "\{link.name}/mesh-marker-\{i}",
              @cad.Rgb::orange(),
            ),
          )
          node_count = node_count + 1
        }
      }
    }
  }
  if node_count == 0 {
    @cli.eprintln(
      "note: this robot has no visual geometry; writing an empty STEP preview",
    )
  }
  if mesh_visuals > 0 {
    @cli.eprintln(
      "note: \{mesh_visuals} mesh visual(s) are shown as 20 mm frame markers; primitive visuals use exact envelope sizes",
    )
  }
  design.write_step_file(Some(path))
}

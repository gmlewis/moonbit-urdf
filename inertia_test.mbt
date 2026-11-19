///|
fn near_i(a : Double, b : Double) -> Bool {
  (a - b).abs() < 1.0e-9
}

///|
fn near_inertia(a : @urdf.Inertia, b : @urdf.Inertia) -> Bool {
  near_i(a.ixx, b.ixx) &&
  near_i(a.ixy, b.ixy) &&
  near_i(a.ixz, b.ixz) &&
  near_i(a.iyy, b.iyy) &&
  near_i(a.iyz, b.iyz) &&
  near_i(a.izz, b.izz)
}

///|
/// Creates a closed triangle mesh for an axis-aligned box.
fn box_triangles(
  min : (Double, Double, Double),
  max : (Double, Double, Double),
) -> Array[
  ((Double, Double, Double), (Double, Double, Double), (Double, Double, Double)),
] {
  let (x0, y0, z0) = min
  let (x1, y1, z1) = max
  let p000 = (x0, y0, z0)
  let p001 = (x0, y0, z1)
  let p010 = (x0, y1, z0)
  let p011 = (x0, y1, z1)
  let p100 = (x1, y0, z0)
  let p101 = (x1, y0, z1)
  let p110 = (x1, y1, z0)
  let p111 = (x1, y1, z1)
  [
    (p000, p010, p011),
    (p000, p011, p001),
    (p100, p111, p110),
    (p100, p101, p111),
    (p000, p001, p101),
    (p000, p101, p100),
    (p010, p110, p111),
    (p010, p111, p011),
    (p000, p100, p110),
    (p000, p110, p010),
    (p001, p011, p111),
    (p001, p111, p101),
  ]
}

///|
test "analytical box inertia" {
  let i = @urdf.Inertial::from_box(2.0, 4.0, 6.0, mass=12.0)
  assert_true(near_i(i.mass, 12.0))
  assert_true(near_i(i.inertia.ixx, 12.0 * (16.0 + 36.0) / 12.0))
  assert_true(near_i(i.inertia.iyy, 12.0 * (4.0 + 36.0) / 12.0))
  assert_true(near_i(i.inertia.izz, 12.0 * (4.0 + 16.0) / 12.0))
  assert_true(near_i(i.inertia.ixy, 0.0))
}

///|
test "analytical cylinder tube and sphere inertia" {
  let c = @urdf.Inertial::from_cylinder(0.5, 2.0, mass=3.0)
  assert_true(near_i(c.inertia.ixx, 3.0 * (3.0 * 0.25 + 4.0) / 12.0))
  assert_true(near_i(c.inertia.izz, 3.0 * 0.25 / 2.0))
  let t = @urdf.Inertial::from_tube(0.0, 0.5, 2.0, mass=3.0)
  assert_true(near_inertia(t.inertia, c.inertia))
  let s = @urdf.Inertial::from_sphere(0.5, mass=10.0)
  assert_true(near_i(s.inertia.ixx, 2.0 * 10.0 * 0.25 / 5.0))
  assert_true(near_i(s.inertia.ixx, s.inertia.izz))
}

///|
test "density variants compute mass from volume" {
  let rho = @urdf.Density::kg_per_m3(1000.0)
  let b = @urdf.Inertial::from_box_density(2.0, 4.0, 6.0, density=rho)
  assert_true(near_i(b.mass, 48.0 * 1000.0))
  let c = @urdf.Inertial::from_cylinder_density(0.5, 2.0, density=rho)
  assert_true(near_i(c.mass, @math.PI * 0.25 * 2.0 * 1000.0))
  let t = @urdf.Inertial::from_tube_density(0.25, 0.5, 2.0, density=rho)
  assert_true(near_i(t.mass, @math.PI * (0.25 - 0.0625) * 2.0 * 1000.0))
  let s = @urdf.Inertial::from_sphere_density(0.5, density=rho)
  assert_true(near_i(s.mass, 4.0 / 3.0 * @math.PI * 0.125 * 1000.0))
}

///|
test "triangle box matches analytical box exactly" {
  let tri = box_triangles((1.0, 2.0, -1.0), (3.0, 6.0, 5.0))
  let mesh = @urdf.Inertial::from_triangles(tri, density=@urdf.Density::pla())
  let analytic = @urdf.Inertial::from_box_density(
    2.0,
    4.0,
    6.0,
    density=@urdf.Density::pla(),
    origin=@urdf.Origin::new(xyz=(2.0, 4.0, 2.0)),
  )
  assert_true(near_i(mesh.mass, analytic.mass))
  assert_true(near_i(mesh.origin.xyz.0, 2.0))
  assert_true(near_i(mesh.origin.xyz.1, 4.0))
  assert_true(near_i(mesh.origin.xyz.2, 2.0))
  assert_true(near_inertia(mesh.inertia, analytic.inertia))
}

///|
test "triangle mesh handles millimeter input" {
  let tri = box_triangles((1000.0, 2000.0, -1000.0), (3000.0, 6000.0, 5000.0))
  let mesh = @urdf.Inertial::from_triangles_mm(
    tri,
    density=@urdf.Density::pla(),
  )
  let analytic = @urdf.Inertial::from_box_density(
    2.0,
    4.0,
    6.0,
    density=@urdf.Density::pla(),
    origin=@urdf.Origin::new(xyz=(2.0, 4.0, 2.0)),
  )
  assert_true(near_i(mesh.mass, analytic.mass))
  assert_true(near_i(mesh.origin.xyz.0, 2.0))
  assert_true(near_inertia(mesh.inertia, analytic.inertia))
}

///|
test "combine half boxes matches single box" {
  let upper = @urdf.Inertial::from_box(
    2.0,
    2.0,
    2.0,
    mass=1.0,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, 1.0)),
  )
  let lower = @urdf.Inertial::from_box(
    2.0,
    2.0,
    2.0,
    mass=1.0,
    origin=@urdf.Origin::new(xyz=(0.0, 0.0, -1.0)),
  )
  let combined = @urdf.Inertial::combine([upper, lower])
  let single = @urdf.Inertial::from_box(2.0, 2.0, 4.0, mass=2.0)
  assert_true(near_i(combined.mass, single.mass))
  assert_true(near_i(combined.origin.xyz.0, 0.0))
  assert_true(near_i(combined.origin.xyz.1, 0.0))
  assert_true(near_i(combined.origin.xyz.2, 0.0))
  assert_true(near_inertia(combined.inertia, single.inertia))
}

///|
test "parallel axis theorem shifts inertia" {
  let base = @urdf.Inertial::from_box(1.0, 1.0, 1.0, mass=2.0)
  let shifted = @urdf.Inertia::shift_by_offset(
    base.inertia,
    2.0,
    (3.0, 0.0, 0.0),
  )
  assert_true(near_i(shifted.iyy, base.inertia.iyy + 2.0 * 9.0))
  assert_true(near_i(shifted.izz, base.inertia.izz + 2.0 * 9.0))
  assert_true(near_i(shifted.iyz, base.inertia.iyz))
}

///|
test "transform_by maps the center of mass" {
  let part = @urdf.Inertial::from_box(
    1.0,
    1.0,
    1.0,
    mass=1.0,
    origin=@urdf.Origin::new(xyz=(1.0, 0.0, 0.0)),
  )
  let moved = part.transform_by(
    @urdf.Transform::from_origin(
      @urdf.Origin::new(xyz=(0.0, 2.0, 0.0), rpy=(0.0, 0.0, @math.PI / 2.0)),
    ),
  )
  assert_true(near_i(moved.origin.xyz.0, 0.0))
  assert_true(near_i(moved.origin.xyz.1, 3.0))
  assert_true(near_i(moved.mass, part.mass))
}

///|
test "rotate inertia leaves a sphere unchanged" {
  let s = @urdf.Inertial::from_sphere(0.5, mass=10.0)
  let rotated = s.inertia.rotate(@urdf.rotation_matrix_rpy(0.3, 0.4, 0.5))
  assert_true(near_inertia(rotated, s.inertia))
}

///|
test "geometry density dispatch" {
  let rho = @urdf.Density::pla()
  assert_true(
    @urdf.Inertial::from_geometry_density(
      @urdf.Geometry::box_mm(10.0, 10.0, 10.0),
      density=rho,
    )
    is Some(_),
  )
  assert_true(
    @urdf.Inertial::from_geometry_density(
      @urdf.Geometry::mesh_mm("a.stl"),
      density=rho,
    )
    is None,
  )
}

///|
test "degenerate and empty inputs" {
  let empty = @urdf.Inertial::combine([])
  assert_true(near_i(empty.mass, 0.0))
  let tri = @urdf.Inertial::from_triangles([], density=@urdf.Density::pla())
  assert_true(near_i(tri.mass, 0.0))
}

///|
test "robotics material density presets are positive and ordered" {
  let presets : Array[(String, @urdf.Density)] = [
    ("pla", @urdf.Density::pla()),
    ("petg", @urdf.Density::petg()),
    ("abs", @urdf.Density::abs()),
    ("tpu", @urdf.Density::tpu()),
    ("cf_nylon", @urdf.Density::cf_nylon()),
    ("rubber", @urdf.Density::rubber()),
    ("lipo_battery", @urdf.Density::lipo_battery()),
    ("fr4_pcb", @urdf.Density::fr4_pcb()),
    ("delrin", @urdf.Density::delrin()),
    ("aluminum", @urdf.Density::aluminum_6061()),
    ("steel", @urdf.Density::steel()),
    ("brass", @urdf.Density::brass()),
  ]
  for entry in presets {
    assert_true(entry.1.to_kg_per_m3() > 0.0)
  }
  assert_true(near_i(@urdf.Density::steel().to_kg_per_m3(), 7850.0))
  assert_true(
    near_i(
      @urdf.Density::steel().to_kg_per_m3(),
      @urdf.Density::steel_mild().to_kg_per_m3(),
    ),
  )
  assert_true(near_i(@urdf.Density::cf_nylon().to_kg_per_m3(), 1250.0))
  assert_true(near_i(@urdf.Density::rubber().to_kg_per_m3(), 1150.0))
  assert_true(near_i(@urdf.Density::lipo_battery().to_kg_per_m3(), 1850.0))
  assert_true(near_i(@urdf.Density::fr4_pcb().to_kg_per_m3(), 1850.0))
  assert_true(near_i(@urdf.Density::brass().to_kg_per_m3(), 8500.0))
  assert_true(near_i(@urdf.Density::delrin().to_kg_per_m3(), 1410.0))
  assert_true(
    @urdf.Density::pla().to_kg_per_m3() <
    @urdf.Density::cf_nylon().to_kg_per_m3(),
  )
  assert_true(
    @urdf.Density::cf_nylon().to_kg_per_m3() <
    @urdf.Density::aluminum_6061().to_kg_per_m3(),
  )
  assert_true(
    @urdf.Density::aluminum_6061().to_kg_per_m3() <
    @urdf.Density::steel().to_kg_per_m3(),
  )
  assert_true(
    @urdf.Density::steel().to_kg_per_m3() <
    @urdf.Density::brass().to_kg_per_m3(),
  )
}

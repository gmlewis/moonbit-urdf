///|
fn close(a : Double, b : Double) -> Bool {
  (a - b).abs() < 1.0e-12
}

///|
test "rgba presets and defaults" {
  let c = @urdf.Rgba::new(0.1, 0.2, 0.3)
  assert_true(close(c.a, 1.0))
  assert_true(close(@urdf.Rgba::orange().r, 1.0))
  assert_true(close(@urdf.Rgba::orange().g, 0.6))
  assert_true(close(@urdf.Rgba::blue().b, 1.0))
  assert_true(close(@urdf.Rgba::green().g, 0.8))
  assert_true(close(@urdf.Rgba::silver().r, 0.75))
  assert_true(close(@urdf.Rgba::yellow().b, 0.0))
}

///|
test "origin identity and mm deg conversion" {
  assert_true(@urdf.Origin::identity().is_identity())
  let o = @urdf.Origin::from_mm_deg(x=250.0, rz_deg=90.0)
  assert_true(close(o.xyz.0, 0.25))
  assert_true(close(o.rpy.2, @math.PI / 2.0))
  assert_false(o.is_identity())
  assert_false(@urdf.Origin::new(xyz=(1.0e-6, 0.0, 0.0)).is_identity())
}

///|
test "geometry constructors convert millimeters" {
  match @urdf.Geometry::box_mm(2.0, 4.0, 6.0) {
    @urdf.Geometry::Box((sx, sy, sz)) => {
      assert_true(close(sx, 0.002))
      assert_true(close(sy, 0.004))
      assert_true(close(sz, 0.006))
    }
    _ => fail("expected box")
  }
  match @urdf.Geometry::cylinder_mm(10.0, 20.0) {
    @urdf.Geometry::Cylinder(r, l) => {
      assert_true(close(r, 0.01))
      assert_true(close(l, 0.02))
    }
    _ => fail("expected cylinder")
  }
  match @urdf.Geometry::sphere_mm(5.0) {
    @urdf.Geometry::Sphere(r) => assert_true(close(r, 0.005))
    _ => fail("expected sphere")
  }
  match @urdf.Geometry::mesh_mm("a.stl") {
    @urdf.Geometry::Mesh(f, Some((sx, _, _))) => {
      assert_eq(f, "a.stl")
      assert_true(close(sx, 0.001))
    }
    _ => fail("expected scaled mesh")
  }
}

///|
test "joint type roundtrip" {
  let all = [
    @urdf.JointType::Revolute,
    @urdf.JointType::Continuous,
    @urdf.JointType::Prismatic,
    @urdf.JointType::Fixed,
    @urdf.JointType::Floating,
    @urdf.JointType::Planar,
  ]
  for jt in all {
    let s = jt.to_urdf_str()
    assert_true(@urdf.JointType::from_urdf_str(s) == Some(jt))
  }
  assert_true(@urdf.JointType::from_urdf_str("bogus") is None)
}

///|
test "inertia constructors" {
  let z = @urdf.Inertia::zero()
  assert_true(close(z.ixx, 0.0))
  let d = @urdf.Inertia::diagonal(1.0, 2.0, 3.0)
  assert_true(close(d.iyy, 2.0))
  assert_true(close(d.ixy, 0.0))
  let m = @urdf.Inertia::new(
    ixx=4.0,
    iyy=5.0,
    izz=6.0,
    ixy=0.1,
    ixz=0.2,
    iyz=0.3,
  )
  assert_true(close(m.ixz, 0.2))
  assert_true(close(m.iyz, 0.3))
}

///|
test "material and joint helpers" {
  let mat = @urdf.Material::color_only("red", @urdf.Rgba::red())
  assert_eq(mat.name, "red")
  assert_true(mat.color is Some(_))
  assert_true(mat.texture is None)
  let lim = @urdf.JointLimit::new(effort=1.0, velocity=2.0)
  assert_true(lim.lower is None)
  let mimic = @urdf.JointMimic::new("other")
  assert_true(close(mimic.multiplier, 1.0))
  assert_true(close(@urdf.JointDynamics::new(damping=0.5).friction, 0.0))
}

///|
fn near(a : Double, b : Double) -> Bool {
  (a - b).abs() < 1.0e-9
}

///|
fn near_vec(a : (Double, Double, Double), b : (Double, Double, Double)) -> Bool {
  near(a.0, b.0) && near(a.1, b.1) && near(a.2, b.2)
}

///|
fn near_matrix(a : @urdf.Matrix3x3, b : @urdf.Matrix3x3) -> Bool {
  near(a.m00, b.m00) &&
  near(a.m01, b.m01) &&
  near(a.m02, b.m02) &&
  near(a.m10, b.m10) &&
  near(a.m11, b.m11) &&
  near(a.m12, b.m12) &&
  near(a.m20, b.m20) &&
  near(a.m21, b.m21) &&
  near(a.m22, b.m22)
}

///|
test "vector helpers" {
  let a = (1.0, 2.0, 3.0)
  let b = (4.0, 5.0, 6.0)
  assert_true(near_vec(@urdf.v3_add(a, b), (5.0, 7.0, 9.0)))
  assert_true(near_vec(@urdf.v3_sub(b, a), (3.0, 3.0, 3.0)))
  assert_true(near_vec(@urdf.v3_scale(a, 2.0), (2.0, 4.0, 6.0)))
  assert_true(near(@urdf.v3_dot(a, b), 32.0))
  assert_true(
    near_vec(@urdf.v3_cross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0)),
  )
  assert_true(near(@urdf.v3_len((3.0, 4.0, 0.0)), 5.0))
  assert_true(near_vec(@urdf.v3_norm((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0)))
  assert_true(near_vec(@urdf.v3_norm((2.0, 0.0, 0.0)), (1.0, 0.0, 0.0)))
}

///|
test "rpy to matrix to rpy roundtrips" {
  let angles = [
    (0.0, 0.0, 0.0),
    (0.3, -0.4, 0.5),
    (@math.PI / 2.0, 0.0, 0.0),
    (0.0, 0.0, @math.PI / 2.0),
    (0.1, 1.2, -0.7),
    (@math.PI / 2.0, @math.PI / 2.0, @math.PI / 2.0),
  ]
  for (r, p, y) in angles {
    let m = @urdf.rotation_matrix_rpy(r, p, y)
    let (r2, p2, y2) = @urdf.matrix_to_rpy(m)
    let m2 = @urdf.rotation_matrix_rpy(r2, p2, y2)
    assert_true(near_matrix(m, m2))
  }
}

///|
test "gimbal lock pitch extraction" {
  let m = @urdf.rotation_matrix_rpy(0.0, @math.PI / 2.0, 0.6)
  let (r, p, y) = @urdf.matrix_to_rpy(m)
  assert_true(near(r, 0.0))
  assert_true(near(p, @math.PI / 2.0))
  assert_true(near(y, 0.6))
}

///|
test "axis angle rotates x into y" {
  let m = @urdf.rotation_matrix_axis_angle((0.0, 0.0, 1.0), @math.PI / 2.0)
  assert_true(near_vec(m.apply((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0)))
  let m2 = @urdf.rotation_matrix_axis_angle((0.0, 0.0, 0.0), 1.0)
  assert_true(near_matrix(m2, @urdf.Matrix3x3::identity()))
}

///|
test "transform compose and inverse" {
  let t = @urdf.Transform::from_origin(
    @urdf.Origin::new(xyz=(1.0, 2.0, 3.0), rpy=(0.2, -0.3, 0.4)),
  )
  let ident = t.compose(t.inverse())
  assert_true(near_matrix(ident.rot, @urdf.Matrix3x3::identity()))
  assert_true(near_vec(ident.trans, (0.0, 0.0, 0.0)))
  let p = (0.5, -1.0, 2.5)
  let m = @urdf.rotation_matrix_rpy(0.2, -0.3, 0.4)
  let expected = @urdf.v3_add(m.apply(p), t.trans)
  assert_true(near_vec(t.apply_point(p), expected))
}

///|
test "transform to origin roundtrip" {
  let o = @urdf.Origin::new(xyz=(0.1, 0.2, 0.3), rpy=(0.4, 0.5, 0.6))
  let o2 = @urdf.Transform::from_origin(o).to_origin()
  assert_true(near_vec(o2.xyz, o.xyz))
  let m1 = @urdf.rotation_matrix_rpy(o.rpy.0, o.rpy.1, o.rpy.2)
  let m2 = @urdf.rotation_matrix_rpy(o2.rpy.0, o2.rpy.1, o2.rpy.2)
  assert_true(near_matrix(m1, m2))
}

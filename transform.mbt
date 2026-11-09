///|
/// Public 3D vector helpers used throughout the URDF math.

///|
pub fn v3_add(a : Vec3, b : Vec3) -> Vec3 {
  (a.0 + b.0, a.1 + b.1, a.2 + b.2)
}

///|
pub fn v3_sub(a : Vec3, b : Vec3) -> Vec3 {
  (a.0 - b.0, a.1 - b.1, a.2 - b.2)
}

///|
pub fn v3_scale(a : Vec3, s : Double) -> Vec3 {
  (a.0 * s, a.1 * s, a.2 * s)
}

///|
pub fn v3_dot(a : Vec3, b : Vec3) -> Double {
  a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}

///|
pub fn v3_cross(a : Vec3, b : Vec3) -> Vec3 {
  (a.1 * b.2 - a.2 * b.1, a.2 * b.0 - a.0 * b.2, a.0 * b.1 - a.1 * b.0)
}

///|
pub fn v3_len(a : Vec3) -> Double {
  v3_dot(a, a).sqrt()
}

///|
pub fn v3_norm(a : Vec3) -> Vec3 {
  let len = v3_len(a)
  if len < 1.0e-15 {
    (0.0, 0.0, 0.0)
  } else {
    v3_scale(a, 1.0 / len)
  }
}

///|
/// A 3x3 matrix stored in row-major order.
#derive(Debug, Eq)
pub(all) struct Matrix3x3 {
  m00 : Double
  m01 : Double
  m02 : Double
  m10 : Double
  m11 : Double
  m12 : Double
  m20 : Double
  m21 : Double
  m22 : Double
}

///|
pub fn Matrix3x3::identity() -> Matrix3x3 {
  {
    m00: 1.0,
    m01: 0.0,
    m02: 0.0,
    m10: 0.0,
    m11: 1.0,
    m12: 0.0,
    m20: 0.0,
    m21: 0.0,
    m22: 1.0,
  }
}

///|
pub fn Matrix3x3::transpose(self : Matrix3x3) -> Matrix3x3 {
  {
    m00: self.m00,
    m01: self.m10,
    m02: self.m20,
    m10: self.m01,
    m11: self.m11,
    m12: self.m21,
    m20: self.m02,
    m21: self.m12,
    m22: self.m22,
  }
}

///|
pub fn Matrix3x3::mul(self : Matrix3x3, other : Matrix3x3) -> Matrix3x3 {
  {
    m00: self.m00 * other.m00 + self.m01 * other.m10 + self.m02 * other.m20,
    m01: self.m00 * other.m01 + self.m01 * other.m11 + self.m02 * other.m21,
    m02: self.m00 * other.m02 + self.m01 * other.m12 + self.m02 * other.m22,
    m10: self.m10 * other.m00 + self.m11 * other.m10 + self.m12 * other.m20,
    m11: self.m10 * other.m01 + self.m11 * other.m11 + self.m12 * other.m21,
    m12: self.m10 * other.m02 + self.m11 * other.m12 + self.m12 * other.m22,
    m20: self.m20 * other.m00 + self.m21 * other.m10 + self.m22 * other.m20,
    m21: self.m20 * other.m01 + self.m21 * other.m11 + self.m22 * other.m21,
    m22: self.m20 * other.m02 + self.m21 * other.m12 + self.m22 * other.m22,
  }
}

///|
pub fn Matrix3x3::apply(self : Matrix3x3, v : Vec3) -> Vec3 {
  (
    self.m00 * v.0 + self.m01 * v.1 + self.m02 * v.2,
    self.m10 * v.0 + self.m11 * v.1 + self.m12 * v.2,
    self.m20 * v.0 + self.m21 * v.1 + self.m22 * v.2,
  )
}

///|
/// Builds the URDF fixed-axis roll/pitch/yaw rotation matrix
/// `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
pub fn rotation_matrix_rpy(
  roll : Double,
  pitch : Double,
  yaw : Double,
) -> Matrix3x3 {
  let cr = @math.cos(roll)
  let sr = @math.sin(roll)
  let cp = @math.cos(pitch)
  let sp = @math.sin(pitch)
  let cy = @math.cos(yaw)
  let sy = @math.sin(yaw)
  {
    m00: cy * cp,
    m01: cy * sp * sr - sy * cr,
    m02: cy * sp * cr + sy * sr,
    m10: sy * cp,
    m11: sy * sp * sr + cy * cr,
    m12: sy * sp * cr - cy * sr,
    m20: -sp,
    m21: cp * sr,
    m22: cp * cr,
  }
}

///|
/// Extracts `(roll, pitch, yaw)` radians from a rotation matrix.
pub fn matrix_to_rpy(m : Matrix3x3) -> Vec3 {
  let z = -m.m20
  let clamped = if z < -1.0 { -1.0 } else if z > 1.0 { 1.0 } else { z }
  let pitch = @math.asin(clamped)
  if @math.cos(pitch).abs() > 1.0e-6 {
    (@math.atan2(m.m21, m.m22), pitch, @math.atan2(m.m10, m.m00))
  } else {
    (0.0, pitch, @math.atan2(-m.m01, m.m11))
  }
}

///|
/// Builds a rotation matrix from a unit axis and angle using Rodrigues'
/// formula.
pub fn rotation_matrix_axis_angle(axis : Vec3, angle_rad : Double) -> Matrix3x3 {
  let u = v3_norm(axis)
  let (ux, uy, uz) = u
  if ux == 0.0 && uy == 0.0 && uz == 0.0 {
    return Matrix3x3::identity()
  }
  let c = @math.cos(angle_rad)
  let s = @math.sin(angle_rad)
  let t = 1.0 - c
  {
    m00: t * ux * ux + c,
    m01: t * ux * uy - s * uz,
    m02: t * ux * uz + s * uy,
    m10: t * ux * uy + s * uz,
    m11: t * uy * uy + c,
    m12: t * uy * uz - s * ux,
    m20: t * ux * uz - s * uy,
    m21: t * uy * uz + s * ux,
    m22: t * uz * uz + c,
  }
}

///|
/// A rigid transform: rotation followed by translation.
#derive(Debug, Eq)
pub(all) struct Transform {
  rot : Matrix3x3
  trans : Vec3
}

///|
pub fn Transform::identity() -> Transform {
  { rot: Matrix3x3::identity(), trans: (0.0, 0.0, 0.0), }
}

///|
pub fn Transform::from_origin(origin : Origin) -> Transform {
  {
    rot: rotation_matrix_rpy(origin.rpy.0, origin.rpy.1, origin.rpy.2),
    trans: origin.xyz,
  }
}

///|
pub fn Transform::to_origin(self : Transform) -> Origin {
  { xyz: self.trans, rpy: matrix_to_rpy(self.rot), }
}

///|
/// Composes `self` followed by `other`.
pub fn Transform::compose(self : Transform, other : Transform) -> Transform {
  {
    rot: self.rot.mul(other.rot),
    trans: v3_add(self.rot.apply(other.trans), self.trans),
  }
}

///|
pub fn Transform::inverse(self : Transform) -> Transform {
  let rt = self.rot.transpose()
  let neg_t = (-self.trans.0, -self.trans.1, -self.trans.2)
  { rot: rt, trans: rt.apply(neg_t), }
}

///|
pub fn Transform::apply_point(self : Transform, p : Vec3) -> Vec3 {
  v3_add(self.rot.apply(p), self.trans)
}

///|
pub fn Transform::apply_dir(self : Transform, v : Vec3) -> Vec3 {
  self.rot.apply(v)
}

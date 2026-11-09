///|
/// Inertial and physical mass-property math for URDF links.

///|
pub fn Inertia::to_matrix(self : Inertia) -> Matrix3x3 {
  {
    m00: self.ixx,
    m01: self.ixy,
    m02: self.ixz,
    m10: self.ixy,
    m11: self.iyy,
    m12: self.iyz,
    m20: self.ixz,
    m21: self.iyz,
    m22: self.izz,
  }
}

///|
pub fn Inertia::from_matrix(m : Matrix3x3) -> Inertia {
  {
    ixx: m.m00,
    ixy: 0.5 * (m.m01 + m.m10),
    ixz: 0.5 * (m.m02 + m.m20),
    iyy: m.m11,
    iyz: 0.5 * (m.m12 + m.m21),
    izz: m.m22,
  }
}

///|
pub fn Inertia::add(self : Inertia, other : Inertia) -> Inertia {
  {
    ixx: self.ixx + other.ixx,
    ixy: self.ixy + other.ixy,
    ixz: self.ixz + other.ixz,
    iyy: self.iyy + other.iyy,
    iyz: self.iyz + other.iyz,
    izz: self.izz + other.izz,
  }
}

///|
pub fn Inertia::scale(self : Inertia, factor : Double) -> Inertia {
  {
    ixx: self.ixx * factor,
    ixy: self.ixy * factor,
    ixz: self.ixz * factor,
    iyy: self.iyy * factor,
    iyz: self.iyz * factor,
    izz: self.izz * factor,
  }
}

///|
/// Rotates the inertia tensor into a new frame: `R * I * R^T`.
pub fn Inertia::rotate(self : Inertia, rot : Matrix3x3) -> Inertia {
  Inertia::from_matrix(rot.mul(self.to_matrix()).mul(rot.transpose()))
}

///|
/// Applies the parallel-axis theorem, shifting the tensor from the
/// center of mass by displacement `d`.
pub fn Inertia::shift_by_offset(
  self : Inertia,
  mass : Double,
  d : Vec3,
) -> Inertia {
  let (dx, dy, dz) = d
  {
    ixx: self.ixx + mass * (dy * dy + dz * dz),
    ixy: self.ixy - mass * dx * dy,
    ixz: self.ixz - mass * dx * dz,
    iyy: self.iyy + mass * (dx * dx + dz * dz),
    iyz: self.iyz - mass * dy * dz,
    izz: self.izz + mass * (dx * dx + dy * dy),
  }
}

///|
pub fn Inertial::from_box(
  sx : Double,
  sy : Double,
  sz : Double,
  mass~ : Double,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let ixx = mass * (sy * sy + sz * sz) / 12.0
  let iyy = mass * (sx * sx + sz * sz) / 12.0
  let izz = mass * (sx * sx + sy * sy) / 12.0
  { origin, mass, inertia: Inertia::diagonal(ixx, iyy, izz), }
}

///|
pub fn Inertial::from_box_density(
  sx : Double,
  sy : Double,
  sz : Double,
  density~ : Density,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let volume = sx * sy * sz
  Inertial::from_box(sx, sy, sz, mass=volume * density.to_kg_per_m3(), origin~)
}

///|
pub fn Inertial::from_cylinder(
  radius : Double,
  length : Double,
  mass~ : Double,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let ixx = mass * (3.0 * radius * radius + length * length) / 12.0
  let izz = mass * radius * radius / 2.0
  { origin, mass, inertia: Inertia::diagonal(ixx, ixx, izz), }
}

///|
pub fn Inertial::from_cylinder_density(
  radius : Double,
  length : Double,
  density~ : Density,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let volume = @math.PI * radius * radius * length
  Inertial::from_cylinder(
    radius,
    length,
    mass=volume * density.to_kg_per_m3(),
    origin~,
  )
}

///|
pub fn Inertial::from_tube(
  inner_radius : Double,
  outer_radius : Double,
  length : Double,
  mass~ : Double,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let r2 = outer_radius * outer_radius + inner_radius * inner_radius
  let ixx = mass * (3.0 * r2 + length * length) / 12.0
  let izz = mass * r2 / 2.0
  { origin, mass, inertia: Inertia::diagonal(ixx, ixx, izz), }
}

///|
pub fn Inertial::from_tube_density(
  inner_radius : Double,
  outer_radius : Double,
  length : Double,
  density~ : Density,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let area = @math.PI *
    (outer_radius * outer_radius - inner_radius * inner_radius)
  let mass = area * length * density.to_kg_per_m3()
  Inertial::from_tube(inner_radius, outer_radius, length, mass~, origin~)
}

///|
pub fn Inertial::from_sphere(
  radius : Double,
  mass~ : Double,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let i = 2.0 * mass * radius * radius / 5.0
  { origin, mass, inertia: Inertia::diagonal(i, i, i), }
}

///|
pub fn Inertial::from_sphere_density(
  radius : Double,
  density~ : Density,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  let volume = 4.0 * @math.PI * radius * radius * radius / 3.0
  Inertial::from_sphere(radius, mass=volume * density.to_kg_per_m3(), origin~)
}

///|
pub fn Inertial::from_geometry_density(
  geom : Geometry,
  density~ : Density,
  origin~ : Origin = Origin::identity(),
) -> Inertial? {
  match geom {
    Box((sx, sy, sz)) =>
      Some(Inertial::from_box_density(sx, sy, sz, density~, origin~))
    Cylinder(radius, length) =>
      Some(Inertial::from_cylinder_density(radius, length, density~, origin~))
    Sphere(radius) =>
      Some(Inertial::from_sphere_density(radius, density~, origin~))
    Mesh(_, _) => None
  }
}

///|
/// One covariance component of a tetrahedron with vertices at the origin,
/// `a`, `b`, and `c`.
fn tri_cov(
  vi : Double,
  aj : Double,
  ak : Double,
  bj : Double,
  bk : Double,
  cj : Double,
  ck : Double,
) -> Double {
  vi /
  20.0 *
  (
    2.0 * aj * ak +
    2.0 * bj * bk +
    2.0 * cj * ck +
    aj * bk +
    bj * ak +
    aj * ck +
    cj * ak +
    bj * ck +
    cj * bk
  )
}

///|
/// Computes exact mass properties from a closed triangle mesh using signed
/// tetrahedron integration.
pub fn Inertial::from_triangles(
  triangles : Array[(Vec3, Vec3, Vec3)],
  density~ : Density,
  scale~ : Double = 1.0,
) -> Inertial {
  let mut volume = 0.0
  let mut m1x = 0.0
  let mut m1y = 0.0
  let mut m1z = 0.0
  let mut cxx = 0.0
  let mut cxy = 0.0
  let mut cxz = 0.0
  let mut cyy = 0.0
  let mut cyz = 0.0
  let mut czz = 0.0
  for (a0, b0, c0) in triangles {
    let a = v3_scale(a0, scale)
    let b = v3_scale(b0, scale)
    let c = v3_scale(c0, scale)
    let vi = v3_dot(v3_cross(a, b), c) / 6.0
    let gx = (a.0 + b.0 + c.0) / 4.0
    let gy = (a.1 + b.1 + c.1) / 4.0
    let gz = (a.2 + b.2 + c.2) / 4.0
    volume = volume + vi
    m1x = m1x + vi * gx
    m1y = m1y + vi * gy
    m1z = m1z + vi * gz
    cxx = cxx + tri_cov(vi, a.0, a.0, b.0, b.0, c.0, c.0)
    cxy = cxy + tri_cov(vi, a.0, a.1, b.0, b.1, c.0, c.1)
    cxz = cxz + tri_cov(vi, a.0, a.2, b.0, b.2, c.0, c.2)
    cyy = cyy + tri_cov(vi, a.1, a.1, b.1, b.1, c.1, c.1)
    cyz = cyz + tri_cov(vi, a.1, a.2, b.1, b.2, c.1, c.2)
    czz = czz + tri_cov(vi, a.2, a.2, b.2, b.2, c.2, c.2)
  }
  if volume.abs() < 1.0e-18 {
    return Inertial::new(0.0, Inertia::zero())
  }
  let (vol, sgn) = if volume < 0.0 { (-volume, -1.0) } else { (volume, 1.0) }
  m1x = m1x * sgn
  m1y = m1y * sgn
  m1z = m1z * sgn
  cxx = cxx * sgn
  cxy = cxy * sgn
  cxz = cxz * sgn
  cyy = cyy * sgn
  cyz = cyz * sgn
  czz = czz * sgn
  let cx = m1x / vol
  let cy = m1y / vol
  let cz = m1z / vol
  let rho = density.to_kg_per_m3()
  // Shift the covariance matrix to the center of mass.
  let dxx = cxx - vol * cx * cx
  let dxy = cxy - vol * cx * cy
  let dxz = cxz - vol * cx * cz
  let dyy = cyy - vol * cy * cy
  let dyz = cyz - vol * cy * cz
  let dzz = czz - vol * cz * cz
  let tr = dxx + dyy + dzz
  let inertia = Inertia::new(
    ixx=rho * (tr - dxx),
    iyy=rho * (tr - dyy),
    izz=rho * (tr - dzz),
    ixy=-rho * dxy,
    ixz=-rho * dxz,
    iyz=-rho * dyz,
  )
  Inertial::new(rho * vol, inertia, origin=Origin::new(xyz=(cx, cy, cz)))
}

///|
/// Convenience wrapper for triangle meshes authored in millimeters.
pub fn Inertial::from_triangles_mm(
  triangles : Array[(Vec3, Vec3, Vec3)],
  density~ : Density,
) -> Inertial {
  Inertial::from_triangles(triangles, density~, scale=0.001)
}

///|
/// Maps an inertial frame through a parent transform. The local inertia tensor
/// is unchanged because the URDF `<inertial><origin>` defines both the center
/// of mass and the tensor's orientation.
pub fn Inertial::transform_by(
  self : Inertial,
  parent_transform : Transform,
) -> Inertial {
  {
    ..self,
    origin: parent_transform
    .compose(Transform::from_origin(self.origin))
    .to_origin(),
  }
}

///|
/// Combines rigid sub-parts, expressed in the same link frame, into one
/// equivalent inertial whose origin is the combined center of mass.
pub fn Inertial::combine(parts : Array[Inertial]) -> Inertial {
  let mut total_mass = 0.0
  for part in parts {
    total_mass = total_mass + part.mass
  }
  if total_mass <= 1.0e-15 {
    return Inertial::new(0.0, Inertia::zero())
  }
  let mut cx = 0.0
  let mut cy = 0.0
  let mut cz = 0.0
  for part in parts {
    cx = cx + part.mass * part.origin.xyz.0
    cy = cy + part.mass * part.origin.xyz.1
    cz = cz + part.mass * part.origin.xyz.2
  }
  cx = cx / total_mass
  cy = cy / total_mass
  cz = cz / total_mass
  let mut total = Inertia::zero()
  for part in parts {
    let rot = rotation_matrix_rpy(
      part.origin.rpy.0,
      part.origin.rpy.1,
      part.origin.rpy.2,
    )
    let rotated = part.inertia.rotate(rot)
    let d = (
      part.origin.xyz.0 - cx,
      part.origin.xyz.1 - cy,
      part.origin.xyz.2 - cz,
    )
    total = total.add(rotated.shift_by_offset(part.mass, d))
  }
  Inertial::new(total_mass, total, origin=Origin::new(xyz=(cx, cy, cz)))
}

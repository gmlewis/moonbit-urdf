///|
/// A 3-component vector: `(x, y, z)`.
pub type Vec3 = (Double, Double, Double)

///|
/// An RGBA color, with each component in `[0, 1]`.
#derive(Debug, Eq)
pub(all) struct Rgba {
  r : Double
  g : Double
  b : Double
  a : Double
}

///|
pub fn Rgba::new(r : Double, g : Double, b : Double, a~ : Double = 1.0) -> Rgba {
  { r, g, b, a, }
}

///|
pub fn Rgba::orange() -> Rgba {
  { r: 1.0, g: 0.6, b: 0.0, a: 1.0, }
}

///|
pub fn Rgba::blue() -> Rgba {
  { r: 0.2, g: 0.6, b: 1.0, a: 1.0, }
}

///|
pub fn Rgba::grey() -> Rgba {
  { r: 0.2, g: 0.2, b: 0.2, a: 1.0, }
}

///|
pub fn Rgba::silver() -> Rgba {
  { r: 0.75, g: 0.75, b: 0.75, a: 1.0, }
}

///|
pub fn Rgba::purple() -> Rgba {
  { r: 0.5, g: 0.0, b: 0.5, a: 1.0, }
}

///|
pub fn Rgba::magenta() -> Rgba {
  { r: 1.0, g: 0.0, b: 1.0, a: 1.0, }
}

///|
pub fn Rgba::white() -> Rgba {
  { r: 1.0, g: 1.0, b: 1.0, a: 1.0, }
}

///|
pub fn Rgba::black() -> Rgba {
  { r: 0.0, g: 0.0, b: 0.0, a: 1.0, }
}

///|
pub fn Rgba::green() -> Rgba {
  { r: 0.0, g: 0.8, b: 0.0, a: 1.0, }
}

///|
pub fn Rgba::red() -> Rgba {
  { r: 0.8, g: 0.0, b: 0.0, a: 1.0, }
}

///|
pub fn Rgba::cyan() -> Rgba {
  { r: 0.0, g: 0.8, b: 0.8, a: 1.0, }
}

///|
pub fn Rgba::yellow() -> Rgba {
  { r: 1.0, g: 1.0, b: 0.0, a: 1.0, }
}

///|
/// A URDF `<origin>`: translation in meters and fixed-axis roll/pitch/yaw in
/// radians.
#derive(Debug, Eq)
pub(all) struct Origin {
  xyz : Vec3
  rpy : Vec3
}

///|
pub fn Origin::identity() -> Origin {
  { xyz: (0.0, 0.0, 0.0), rpy: (0.0, 0.0, 0.0), }
}

///|
pub fn Origin::new(
  xyz~ : Vec3 = (0.0, 0.0, 0.0),
  rpy~ : Vec3 = (0.0, 0.0, 0.0),
) -> Origin {
  { xyz, rpy, }
}

///|
/// Builds an origin from millimeter translations and degree rotations, which
/// is the natural input unit for CAD-authored components.
pub fn Origin::from_mm_deg(
  x~ : Double = 0.0,
  y~ : Double = 0.0,
  z~ : Double = 0.0,
  rx_deg~ : Double = 0.0,
  ry_deg~ : Double = 0.0,
  rz_deg~ : Double = 0.0,
) -> Origin {
  let d = @math.PI / 180.0
  {
    xyz: (x * 0.001, y * 0.001, z * 0.001),
    rpy: (rx_deg * d, ry_deg * d, rz_deg * d),
  }
}

///|
pub fn Origin::is_identity(self : Origin) -> Bool {
  let (x, y, z) = self.xyz
  let (r, p, w) = self.rpy
  x.abs() < 1.0e-12 &&
  y.abs() < 1.0e-12 &&
  z.abs() < 1.0e-12 &&
  r.abs() < 1.0e-12 &&
  p.abs() < 1.0e-12 &&
  w.abs() < 1.0e-12
}

///|
/// The shape of a link's visual or collision geometry.
#derive(Debug, Eq)
pub(all) enum Geometry {
  /// Box with dimensions `(sx, sy, sz)` in meters.
  Box(Vec3)
  /// Cylinder with `(radius, length)` in meters, axis along local Z.
  Cylinder(Double, Double)
  /// Sphere with `radius` in meters.
  Sphere(Double)
  /// Mesh file with an optional `(sx, sy, sz)` scale.
  Mesh(String, Vec3?)
}

///|
pub fn Geometry::box(sx : Double, sy : Double, sz : Double) -> Geometry {
  Box((sx, sy, sz))
}

///|
pub fn Geometry::box_mm(
  sx_mm : Double,
  sy_mm : Double,
  sz_mm : Double,
) -> Geometry {
  Box((sx_mm * 0.001, sy_mm * 0.001, sz_mm * 0.001))
}

///|
pub fn Geometry::cylinder(radius : Double, length : Double) -> Geometry {
  Cylinder(radius, length)
}

///|
pub fn Geometry::cylinder_mm(
  radius_mm : Double,
  length_mm : Double,
) -> Geometry {
  Cylinder(radius_mm * 0.001, length_mm * 0.001)
}

///|
pub fn Geometry::sphere(radius : Double) -> Geometry {
  Sphere(radius)
}

///|
pub fn Geometry::sphere_mm(radius_mm : Double) -> Geometry {
  Sphere(radius_mm * 0.001)
}

///|
pub fn Geometry::mesh(filename : String, scale~ : Vec3? = None) -> Geometry {
  Mesh(filename, scale)
}

///|
pub fn Geometry::mesh_mm(filename : String) -> Geometry {
  Mesh(filename, Some((0.001, 0.001, 0.001)))
}

///|
/// A named material with an optional color and texture.
#derive(Debug, Eq)
pub(all) struct Material {
  name : String
  color : Rgba?
  texture : String?
}

///|
pub fn Material::new(
  name : String,
  color~ : Rgba? = None,
  texture~ : String? = None,
) -> Material {
  { name, color, texture, }
}

///|
pub fn Material::color_only(name : String, color : Rgba) -> Material {
  { name, color: Some(color), texture: None, }
}

///|
/// A verbatim XML element, used to preserve extension tags (`<gazebo>`,
/// `<ros2_control>`, `<sensor>`, ...) across parse/serialize roundtrips.
#derive(Debug, Eq)
pub(all) struct RawXmlNode {
  tag : String
  attrs : Array[(String, String)]
  text : String
  children : Array[RawXmlNode]
}

///|
/// A visual element on a link.
#derive(Debug, Eq)
pub(all) struct Visual {
  name : String?
  origin : Origin
  geometry : Geometry
  material : Material?
}

///|
/// A collision element on a link.
#derive(Debug, Eq)
pub(all) struct Collision {
  name : String?
  origin : Origin
  geometry : Geometry
}

///|
/// The symmetric 3x3 inertia tensor of a rigid body.
#derive(Debug, Eq)
pub(all) struct Inertia {
  ixx : Double
  ixy : Double
  ixz : Double
  iyy : Double
  iyz : Double
  izz : Double
}

///|
pub fn Inertia::zero() -> Inertia {
  { ixx: 0.0, ixy: 0.0, ixz: 0.0, iyy: 0.0, iyz: 0.0, izz: 0.0, }
}

///|
pub fn Inertia::diagonal(ixx : Double, iyy : Double, izz : Double) -> Inertia {
  { ixx, ixy: 0.0, ixz: 0.0, iyy, iyz: 0.0, izz, }
}

///|
pub fn Inertia::new(
  ixx~ : Double,
  iyy~ : Double,
  izz~ : Double,
  ixy~ : Double = 0.0,
  ixz~ : Double = 0.0,
  iyz~ : Double = 0.0,
) -> Inertia {
  { ixx, ixy, ixz, iyy, iyz, izz, }
}

///|
/// The inertial properties of a link: center-of-mass frame, mass, and tensor.
#derive(Debug, Eq)
pub(all) struct Inertial {
  origin : Origin
  mass : Double
  inertia : Inertia
}

///|
/// A rigid link in the robot tree.
#derive(Debug, Eq)
pub(all) struct Link {
  name : String
  inertial : Inertial?
  visuals : Array[Visual]
  collisions : Array[Collision]
  extra : Array[RawXmlNode]
}

///|
/// The kind of motion a joint permits.
#derive(Debug, Eq)
pub(all) enum JointType {
  Revolute
  Continuous
  Prismatic
  Fixed
  Floating
  Planar
}

///|
pub fn JointType::to_urdf_str(self : JointType) -> String {
  match self {
    Revolute => "revolute"
    Continuous => "continuous"
    Prismatic => "prismatic"
    Fixed => "fixed"
    Floating => "floating"
    Planar => "planar"
  }
}

///|
pub fn JointType::from_urdf_str(s : String) -> JointType? {
  match s {
    "revolute" => Some(Revolute)
    "continuous" => Some(Continuous)
    "prismatic" => Some(Prismatic)
    "fixed" => Some(Fixed)
    "floating" => Some(Floating)
    "planar" => Some(Planar)
    _ => None
  }
}

///|
/// Joint travel limits.
#derive(Debug, Eq)
pub(all) struct JointLimit {
  lower : Double?
  upper : Double?
  effort : Double
  velocity : Double
}

///|
pub fn JointLimit::new(
  effort~ : Double,
  velocity~ : Double,
  lower~ : Double? = None,
  upper~ : Double? = None,
) -> JointLimit {
  { lower, upper, effort, velocity, }
}

///|
/// Joint damping and friction.
#derive(Debug, Eq)
pub(all) struct JointDynamics {
  damping : Double
  friction : Double
}

///|
pub fn JointDynamics::new(
  damping~ : Double = 0.0,
  friction~ : Double = 0.0,
) -> JointDynamics {
  { damping, friction, }
}

///|
/// Joint calibration reference positions.
#derive(Debug, Eq)
pub(all) struct JointCalibration {
  rising : Double?
  falling : Double?
}

///|
/// Joint safety controller limits.
#derive(Debug, Eq)
pub(all) struct SafetyController {
  soft_lower_limit : Double
  soft_upper_limit : Double
  k_position : Double
  k_velocity : Double
}

///|
/// A joint whose motion is derived from another joint.
#derive(Debug, Eq)
pub(all) struct JointMimic {
  joint : String
  multiplier : Double
  offset : Double
}

///|
pub fn JointMimic::new(
  joint : String,
  multiplier~ : Double = 1.0,
  offset~ : Double = 0.0,
) -> JointMimic {
  { joint, multiplier, offset, }
}

///|
/// A kinematic joint connecting a parent link to a child link.
#derive(Debug, Eq)
pub(all) struct Joint {
  name : String
  joint_type : JointType
  origin : Origin
  parent : String
  child : String
  axis : Vec3
  limit : JointLimit?
  dynamics : JointDynamics?
  calibration : JointCalibration?
  safety_controller : SafetyController?
  mimic : JointMimic?
  extra : Array[RawXmlNode]
}

///|
/// A transmission joint entry.
#derive(Debug, Eq)
pub(all) struct TransmissionJoint {
  name : String
  hardware_interfaces : Array[String]
}

///|
/// A transmission actuator entry.
#derive(Debug, Eq)
pub(all) struct Actuator {
  name : String
  hardware_interfaces : Array[String]
  mechanical_reduction : Double?
}

///|
/// A `ros_control` transmission description.
#derive(Debug, Eq)
pub(all) struct Transmission {
  name : String
  trans_type : String
  joints : Array[TransmissionJoint]
  actuators : Array[Actuator]
}

///|
/// A complete URDF robot model.
#derive(Debug, Eq)
pub(all) struct Robot {
  name : String
  version : String?
  documentation : String
  materials : Array[Material]
  links : Array[Link]
  joints : Array[Joint]
  transmissions : Array[Transmission]
  extra : Array[RawXmlNode]
}

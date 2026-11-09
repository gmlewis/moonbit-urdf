///|
/// Fluent builders for the URDF data model.
///
/// Every builder takes `self` by value and returns a new value; arrays are
/// copied before an element is appended so an original value is never mutated.

///|
pub fn Visual::new(
  geometry : Geometry,
  name~ : String? = None,
  origin~ : Origin = Origin::identity(),
  material~ : Material? = None,
) -> Visual {
  { name, origin, geometry, material, }
}

///|
pub fn Visual::with_name(self : Visual, name : String) -> Visual {
  { ..self, name: Some(name), }
}

///|
pub fn Visual::with_origin(self : Visual, origin : Origin) -> Visual {
  { ..self, origin, }
}

///|
pub fn Visual::with_material(self : Visual, material : Material) -> Visual {
  { ..self, material: Some(material), }
}

///|
pub fn Visual::with_color(self : Visual, name : String, color : Rgba) -> Visual {
  { ..self, material: Some(Material::color_only(name, color)), }
}

///|
pub fn Collision::new(
  geometry : Geometry,
  name~ : String? = None,
  origin~ : Origin = Origin::identity(),
) -> Collision {
  { name, origin, geometry, }
}

///|
pub fn Collision::with_name(self : Collision, name : String) -> Collision {
  { ..self, name: Some(name), }
}

///|
pub fn Collision::with_origin(self : Collision, origin : Origin) -> Collision {
  { ..self, origin, }
}

///|
pub fn Inertial::new(
  mass : Double,
  inertia : Inertia,
  origin~ : Origin = Origin::identity(),
) -> Inertial {
  { origin, mass, inertia, }
}

///|
pub fn Inertial::with_origin(self : Inertial, origin : Origin) -> Inertial {
  { ..self, origin, }
}

///|
pub fn Link::new(name : String) -> Link {
  { name, inertial: None, visuals: [], collisions: [], extra: [], }
}

///|
pub fn Link::with_inertial(self : Link, inertial : Inertial) -> Link {
  { ..self, inertial: Some(inertial), }
}

///|
pub fn Link::add_visual(self : Link, visual : Visual) -> Link {
  let visuals = self.visuals.copy()
  visuals.push(visual)
  { ..self, visuals, }
}

///|
pub fn Link::add_collision(self : Link, collision : Collision) -> Link {
  let collisions = self.collisions.copy()
  collisions.push(collision)
  { ..self, collisions, }
}

///|
pub fn Link::add_extra(self : Link, node : RawXmlNode) -> Link {
  let extra = self.extra.copy()
  extra.push(node)
  { ..self, extra, }
}

///|
pub fn Joint::new(
  name : String,
  joint_type : JointType,
  parent~ : String,
  child~ : String,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (1.0, 0.0, 0.0),
) -> Joint {
  {
    name,
    joint_type,
    origin,
    parent,
    child,
    axis,
    limit: None,
    dynamics: None,
    calibration: None,
    safety_controller: None,
    mimic: None,
    extra: [],
  }
}

///|
pub fn Joint::fixed(
  name : String,
  parent~ : String,
  child~ : String,
  origin~ : Origin = Origin::identity(),
) -> Joint {
  Joint::new(name, Fixed, parent~, child~, origin~)
}

///|
pub fn Joint::revolute(
  name : String,
  parent~ : String,
  child~ : String,
  limit~ : JointLimit,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (0.0, 0.0, 1.0),
) -> Joint {
  Joint::new(name, Revolute, parent~, child~, origin~, axis~).with_limit(limit)
}

///|
pub fn Joint::continuous(
  name : String,
  parent~ : String,
  child~ : String,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (0.0, 0.0, 1.0),
  limit~ : JointLimit? = None,
) -> Joint {
  let joint = Joint::new(name, Continuous, parent~, child~, origin~, axis~)
  match limit {
    Some(lim) => joint.with_limit(lim)
    None => joint
  }
}

///|
pub fn Joint::prismatic(
  name : String,
  parent~ : String,
  child~ : String,
  limit~ : JointLimit,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (0.0, 0.0, 1.0),
) -> Joint {
  Joint::new(name, Prismatic, parent~, child~, origin~, axis~).with_limit(limit)
}

///|
pub fn Joint::floating(
  name : String,
  parent~ : String,
  child~ : String,
  origin~ : Origin = Origin::identity(),
) -> Joint {
  Joint::new(name, Floating, parent~, child~, origin~)
}

///|
pub fn Joint::planar(
  name : String,
  parent~ : String,
  child~ : String,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (0.0, 0.0, 1.0),
) -> Joint {
  Joint::new(name, Planar, parent~, child~, origin~, axis~)
}

///|
pub fn Joint::with_origin(self : Joint, origin : Origin) -> Joint {
  { ..self, origin, }
}

///|
pub fn Joint::with_axis(self : Joint, axis : Vec3) -> Joint {
  { ..self, axis, }
}

///|
pub fn Joint::with_limit(self : Joint, limit : JointLimit) -> Joint {
  { ..self, limit: Some(limit), }
}

///|
pub fn Joint::with_dynamics(self : Joint, dynamics : JointDynamics) -> Joint {
  { ..self, dynamics: Some(dynamics), }
}

///|
pub fn Joint::with_calibration(
  self : Joint,
  calibration : JointCalibration,
) -> Joint {
  { ..self, calibration: Some(calibration), }
}

///|
pub fn Joint::with_safety_controller(
  self : Joint,
  controller : SafetyController,
) -> Joint {
  { ..self, safety_controller: Some(controller), }
}

///|
pub fn Joint::with_mimic(self : Joint, mimic : JointMimic) -> Joint {
  { ..self, mimic: Some(mimic), }
}

///|
pub fn Joint::add_extra(self : Joint, node : RawXmlNode) -> Joint {
  let extra = self.extra.copy()
  extra.push(node)
  { ..self, extra, }
}

///|
pub fn Transmission::simple(
  name : String,
  joint_name : String,
  actuator_name : String,
  reduction~ : Double = 1.0,
  hardware_interface~ : String = "hardware_interface/EffortJointInterface",
) -> Transmission {
  {
    name,
    trans_type: "transmission_interface/SimpleTransmission",
    joints: [{ name: joint_name, hardware_interfaces: [hardware_interface], }],
    actuators: [
      {
        name: actuator_name,
        hardware_interfaces: [hardware_interface],
        mechanical_reduction: Some(reduction),
      },
    ],
  }
}

///|
pub fn Robot::new(
  name : String,
  version~ : String? = None,
  documentation~ : String = "",
) -> Robot {
  {
    name,
    version,
    documentation,
    materials: [],
    links: [],
    joints: [],
    transmissions: [],
    extra: [],
  }
}

///|
pub fn Robot::with_version(self : Robot, version : String) -> Robot {
  { ..self, version: Some(version), }
}

///|
pub fn Robot::with_documentation(self : Robot, documentation : String) -> Robot {
  { ..self, documentation, }
}

///|
pub fn Robot::add_material(self : Robot, material : Material) -> Robot {
  let materials = self.materials.copy()
  materials.push(material)
  { ..self, materials, }
}

///|
pub fn Robot::add_link(self : Robot, link : Link) -> Robot {
  let links = self.links.copy()
  links.push(link)
  { ..self, links, }
}

///|
pub fn Robot::add_links(self : Robot, links : Array[Link]) -> Robot {
  let out = self.links.copy()
  for link in links {
    out.push(link)
  }
  { ..self, links: out, }
}

///|
pub fn Robot::add_joint(self : Robot, joint : Joint) -> Robot {
  let joints = self.joints.copy()
  joints.push(joint)
  { ..self, joints, }
}

///|
pub fn Robot::add_joints(self : Robot, joints : Array[Joint]) -> Robot {
  let out = self.joints.copy()
  for joint in joints {
    out.push(joint)
  }
  { ..self, joints: out, }
}

///|
pub fn Robot::add_transmission(
  self : Robot,
  transmission : Transmission,
) -> Robot {
  let transmissions = self.transmissions.copy()
  transmissions.push(transmission)
  { ..self, transmissions, }
}

///|
pub fn Robot::add_extra(self : Robot, node : RawXmlNode) -> Robot {
  let extra = self.extra.copy()
  extra.push(node)
  { ..self, extra, }
}

///|
pub fn Robot::get_link(self : Robot, name : String) -> Link? {
  for link in self.links {
    if link.name == name {
      return Some(link)
    }
  }
  None
}

///|
pub fn Robot::get_joint(self : Robot, name : String) -> Joint? {
  for joint in self.joints {
    if joint.name == name {
      return Some(joint)
    }
  }
  None
}

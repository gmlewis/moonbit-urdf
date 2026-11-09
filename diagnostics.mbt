///|
/// Topological and physical validation diagnostics for URDF robots.

///|
#derive(Debug, Eq)
pub(all) enum Severity {
  Info
  Warning
  Error
}

///|
pub fn Severity::is_error(self : Severity) -> Bool {
  match self {
    Error => true
    _ => false
  }
}

///|
pub fn Severity::all() -> Array[Severity] {
  [Info, Warning, Error]
}

///|
#derive(Debug, Eq)
pub(all) enum Diagnostic {
  EmptyRobot(String)
  DuplicateLinkName(String)
  DuplicateJointName(String)
  MissingParentLink(String, String)
  MissingChildLink(String, String)
  SelfLoopJoint(String, String)
  MultipleParentJoints(String)
  KinematicCycle(String)
  DisconnectedGraph(String)
  ZeroJointAxis(String)
  MissingJointLimit(String)
  InvalidJointLimit(String, String)
  InvalidMimicJoint(String, String)
  InvalidGeometry(String, String)
  NonPositiveMass(String, Double)
  NonPhysicalInertia(String, String)
}

///|
pub fn Diagnostic::severity(self : Diagnostic) -> Severity {
  match self {
    NonPositiveMass(_, _) | NonPhysicalInertia(_, _) => Warning
    _ => Error
  }
}

///|
pub fn Diagnostic::message(self : Diagnostic) -> String {
  match self {
    EmptyRobot(name) => "robot \"\{name}\" has no links"
    DuplicateLinkName(name) => "duplicate link name: \{name}"
    DuplicateJointName(name) => "duplicate joint name: \{name}"
    MissingParentLink(joint, link) =>
      "joint \"\{joint}\" references missing parent link \"\{link}\""
    MissingChildLink(joint, link) =>
      "joint \"\{joint}\" references missing child link \"\{link}\""
    SelfLoopJoint(joint, link) =>
      "joint \"\{joint}\" connects link \"\{link}\" to itself"
    MultipleParentJoints(link) =>
      "link \"\{link}\" has more than one parent joint"
    KinematicCycle(link) => "kinematic cycle involving link \"\{link}\""
    DisconnectedGraph(link) =>
      "robot has multiple root links (for example \"\{link}\")"
    ZeroJointAxis(joint) => "joint \"\{joint}\" has a zero-length axis"
    MissingJointLimit(joint) => "joint \"\{joint}\" requires a <limit> element"
    InvalidJointLimit(joint, detail) =>
      "joint \"\{joint}\" has an invalid limit: \{detail}"
    InvalidMimicJoint(joint, detail) =>
      "joint \"\{joint}\" has an invalid mimic: \{detail}"
    InvalidGeometry(link, detail) =>
      "link \"\{link}\" has invalid geometry: \{detail}"
    NonPositiveMass(link, mass) =>
      "link \"\{link}\" has non-positive mass: \{mass}"
    NonPhysicalInertia(link, detail) =>
      "link \"\{link}\" has non-physical inertia: \{detail}"
  }
}

///|
fn link_exists(robot : Robot, name : String) -> Bool {
  for link in robot.links {
    if link.name == name {
      return true
    }
  }
  false
}

///|
fn joint_exists(robot : Robot, name : String) -> Bool {
  for joint in robot.joints {
    if joint.name == name {
      return true
    }
  }
  false
}

///|
fn geometry_problem(geometry : Geometry) -> String? {
  match geometry {
    Box((sx, sy, sz)) =>
      if sx <= 0.0 || sy <= 0.0 || sz <= 0.0 {
        Some("box dimensions must be positive")
      } else {
        None
      }
    Cylinder(radius, length) =>
      if radius <= 0.0 || length <= 0.0 {
        Some("cylinder radius and length must be positive")
      } else {
        None
      }
    Sphere(radius) =>
      if radius <= 0.0 {
        Some("sphere radius must be positive")
      } else {
        None
      }
    Mesh(filename, _) =>
      if filename == "" {
        Some("mesh filename must not be empty")
      } else {
        None
      }
  }
}

///|
fn inertia_problem(inertia : Inertia) -> String? {
  if inertia.ixx <= 0.0 || inertia.iyy <= 0.0 || inertia.izz <= 0.0 {
    return Some("principal moments must be positive")
  }
  if inertia.ixx + inertia.iyy < inertia.izz - 1.0e-12 {
    return Some("principal triangle inequality violated")
  }
  if inertia.ixx + inertia.izz < inertia.iyy - 1.0e-12 {
    return Some("principal triangle inequality violated")
  }
  if inertia.iyy + inertia.izz < inertia.ixx - 1.0e-12 {
    return Some("principal triangle inequality violated")
  }
  None
}

///|
/// Returns the unique root link, or `None` when there are zero or several.
pub fn Robot::root_link(self : Robot) -> String? {
  let mut root : String? = None
  let mut count = 0
  for link in self.links {
    let mut is_child = false
    for joint in self.joints {
      if joint.child == link.name {
        is_child = true
      }
    }
    if !is_child {
      count = count + 1
      if root is None {
        root = Some(link.name)
      }
    }
  }
  if count == 1 {
    root
  } else {
    None
  }
}

///|
pub fn Robot::diagnostics(self : Robot) -> Array[Diagnostic] {
  let out : Array[Diagnostic] = []
  let n_links = self.links.length()
  if n_links == 0 {
    out.push(EmptyRobot(self.name))
  }
  // Duplicate link names.
  let link_counts : Map[String, Int32] = Map([])
  for link in self.links {
    link_counts.set(link.name, link_counts.get(link.name).unwrap_or(0) + 1)
  }
  let reported_links : Map[String, Bool] = Map([])
  for link in self.links {
    if link_counts.get(link.name).unwrap_or(0) > 1 &&
      !reported_links.get(link.name).unwrap_or(false) {
      reported_links.set(link.name, true)
      out.push(DuplicateLinkName(link.name))
    }
  }
  // Duplicate joint names.
  let joint_counts : Map[String, Int32] = Map([])
  for joint in self.joints {
    joint_counts.set(joint.name, joint_counts.get(joint.name).unwrap_or(0) + 1)
  }
  let reported_joints : Map[String, Bool] = Map([])
  for joint in self.joints {
    if joint_counts.get(joint.name).unwrap_or(0) > 1 &&
      !reported_joints.get(joint.name).unwrap_or(false) {
      reported_joints.set(joint.name, true)
      out.push(DuplicateJointName(joint.name))
    }
  }
  // Parent/child references, self loops, and joint-level checks.
  let child_counts : Map[String, Int32] = Map([])
  for joint in self.joints {
    if !link_exists(self, joint.parent) {
      out.push(MissingParentLink(joint.name, joint.parent))
    }
    if !link_exists(self, joint.child) {
      out.push(MissingChildLink(joint.name, joint.child))
    }
    if joint.parent == joint.child {
      out.push(SelfLoopJoint(joint.name, joint.parent))
    }
    child_counts.set(
      joint.child,
      child_counts.get(joint.child).unwrap_or(0) + 1,
    )
    match joint.joint_type {
      Revolute | Continuous | Prismatic | Planar =>
        if v3_len(joint.axis) < 1.0e-12 {
          out.push(ZeroJointAxis(joint.name))
        }
      _ => ()
    }
    match joint.joint_type {
      Revolute | Prismatic =>
        match joint.limit {
          None => out.push(MissingJointLimit(joint.name))
          Some(limit) => {
            if limit.effort < 0.0 {
              out.push(
                InvalidJointLimit(joint.name, "effort must be non-negative"),
              )
            }
            if limit.velocity < 0.0 {
              out.push(
                InvalidJointLimit(joint.name, "velocity must be non-negative"),
              )
            }
            match (limit.lower, limit.upper) {
              (Some(lower), Some(upper)) =>
                if lower > upper {
                  out.push(InvalidJointLimit(joint.name, "lower exceeds upper"))
                }
              _ => ()
            }
          }
        }
      _ =>
        match joint.limit {
          Some(limit) =>
            match (limit.lower, limit.upper) {
              (Some(lower), Some(upper)) =>
                if lower > upper {
                  out.push(InvalidJointLimit(joint.name, "lower exceeds upper"))
                }
              _ => ()
            }
          None => ()
        }
    }
    // Mimic target and mimic cycles.
    match joint.mimic {
      Some(mimic) =>
        if !joint_exists(self, mimic.joint) {
          out.push(
            InvalidMimicJoint(joint.name, "unknown target joint \{mimic.joint}"),
          )
        } else {
          let mut current = mimic.joint
          let mut seen = false
          for _ in 0..<self.joints.length() {
            if current == joint.name {
              seen = true
              break
            }
            let next : String? = match self.get_joint(current) {
              Some(other) =>
                match other.mimic {
                  Some(m) => Some(m.joint)
                  None => None
                }
              None => None
            }
            match next {
              Some(name) => current = name
              None => break
            }
          }
          if seen {
            out.push(InvalidMimicJoint(joint.name, "mimic cycle detected"))
          }
        }
      None => ()
    }
  }
  // Links with multiple parents.
  for link in self.links {
    if child_counts.get(link.name).unwrap_or(0) > 1 {
      out.push(MultipleParentJoints(link.name))
    }
  }
  // Cycle detection: walking parent pointers must terminate.
  let parent_map : Map[String, String] = Map([])
  for joint in self.joints {
    if !parent_map.contains(joint.child) {
      parent_map.set(joint.child, joint.parent)
    }
  }
  let cycle_reported : Map[String, Bool] = Map([])
  for link in self.links {
    let visited : Map[String, Bool] = Map([])
    let mut current = link.name
    let mut steps = 0
    let mut cycle = false
    let limit = n_links + 1
    while steps < limit {
      if visited.get(current).unwrap_or(false) {
        cycle = true
        break
      }
      visited.set(current, true)
      match parent_map.get(current) {
        Some(parent) => current = parent
        None => break
      }
      steps = steps + 1
    }
    if cycle && !cycle_reported.get(link.name).unwrap_or(false) {
      cycle_reported.set(link.name, true)
      out.push(KinematicCycle(link.name))
    }
  }
  // Root count: a tree has exactly one root.
  let roots : Array[String] = []
  for link in self.links {
    let mut is_child = false
    for joint in self.joints {
      if joint.child == link.name {
        is_child = true
      }
    }
    if !is_child {
      roots.push(link.name)
    }
  }
  if roots.length() > 1 {
    for i in 1..<roots.length() {
      out.push(DisconnectedGraph(roots[i]))
    }
  }
  // Geometry, mass, and inertia checks.
  for link in self.links {
    for visual in link.visuals {
      match geometry_problem(visual.geometry) {
        Some(detail) => out.push(InvalidGeometry(link.name, detail))
        None => ()
      }
    }
    for collision in link.collisions {
      match geometry_problem(collision.geometry) {
        Some(detail) => out.push(InvalidGeometry(link.name, detail))
        None => ()
      }
    }
    match link.inertial {
      Some(inertial) => {
        if inertial.mass <= 0.0 {
          out.push(NonPositiveMass(link.name, inertial.mass))
        }
        match inertia_problem(inertial.inertia) {
          Some(detail) => out.push(NonPhysicalInertia(link.name, detail))
          None => ()
        }
      }
      None => ()
    }
  }
  out
}

///|
pub fn Robot::has_errors(self : Robot) -> Bool {
  self.diagnostics().any(fn(d) { d.severity().is_error() })
}

///|
/// Returns the URDF XML, aborting with the first error diagnostic if the
/// robot is not valid.
pub fn Robot::to_urdf_file_checked(self : Robot) -> String {
  for diagnostic in self.diagnostics() {
    if diagnostic.severity().is_error() {
      abort(diagnostic.message())
    }
  }
  self.to_urdf_xml()
}

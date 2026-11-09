///|
/// A named attachment frame on a component. Mate connectors are the CAD-side
/// ports that let parameterized parts snap together without hand-computed
/// coordinates.
#derive(Debug, Eq)
pub(all) struct MateConnector {
  name : String
  origin : Origin
  axis : Vec3
}

///|
pub fn MateConnector::new(
  name : String,
  origin~ : Origin = Origin::identity(),
  axis~ : Vec3 = (0.0, 0.0, 1.0),
) -> MateConnector {
  { name, origin, axis, }
}

///|
/// Builds a mate from millimeter/degree inputs, normalizing its axis.
pub fn MateConnector::from_mm_deg(
  name : String,
  x~ : Double = 0.0,
  y~ : Double = 0.0,
  z~ : Double = 0.0,
  rx_deg~ : Double = 0.0,
  ry_deg~ : Double = 0.0,
  rz_deg~ : Double = 0.0,
  axis~ : Vec3 = (0.0, 0.0, 1.0),
) -> MateConnector {
  {
    name,
    origin: Origin::from_mm_deg(x~, y~, z~, rx_deg~, ry_deg~, rz_deg~),
    axis: v3_norm(axis),
  }
}

///|
/// Snaps a child mate onto a parent mate. Returns the joint origin (the
/// child-link pose at `q = 0`) and the joint axis expressed in the child
/// frame.
pub fn snap_mates(
  parent_mate : MateConnector,
  child_mate : MateConnector,
  offset~ : Double = 0.0,
  angle_rad~ : Double = 0.0,
  flip_axis~ : Bool = false,
) -> (Origin, Vec3) {
  let t_parent = Transform::from_origin(parent_mate.origin)
  let t_child = Transform::from_origin(child_mate.origin)
  let u = v3_norm(parent_mate.axis)
  let t_offset = {
    rot: rotation_matrix_axis_angle(u, angle_rad),
    trans: v3_scale(u, offset),
  }
  let origin = t_parent.compose(t_offset).compose(t_child.inverse()).to_origin()
  let dir = if flip_axis { (-u.0, -u.1, -u.2) } else { u }
  (origin, t_child.rot.apply(dir))
}

///|
/// Builds a joint by snapping a child component's mate onto a parent's mate.
pub fn Joint::from_mates(
  name : String,
  joint_type : JointType,
  parent_link~ : String,
  parent_mate~ : MateConnector,
  child_link~ : String,
  child_mate~ : MateConnector,
  offset~ : Double = 0.0,
  angle_rad~ : Double = 0.0,
  flip_axis~ : Bool = false,
  limit~ : JointLimit? = None,
  dynamics~ : JointDynamics? = None,
) -> Joint {
  let (origin, axis) = snap_mates(
    parent_mate,
    child_mate,
    offset~,
    angle_rad~,
    flip_axis~,
  )
  let joint = Joint::new(
    name,
    joint_type,
    parent=parent_link,
    child=child_link,
    origin~,
    axis~,
  )
  let joint = match limit {
    Some(lim) => joint.with_limit(lim)
    None => joint
  }
  match dynamics {
    Some(d) => joint.with_dynamics(d)
    None => joint
  }
}

///|
fn rebase_origin(inv : Transform, origin : Origin) -> Origin {
  inv.compose(Transform::from_origin(origin)).to_origin()
}

///|
/// Moves the link's coordinate frame onto `mate`, rewriting every inertial,
/// visual, and collision origin accordingly. This is what makes a component
/// modeled around its own center usable as a URDF child link, whose pivot is
/// always at the link origin.
pub fn Link::rebase_to_mate(self : Link, mate : MateConnector) -> Link {
  let inv = Transform::from_origin(mate.origin).inverse()
  let visuals : Array[Visual] = []
  for visual in self.visuals {
    visuals.push({ ..visual, origin: rebase_origin(inv, visual.origin), })
  }
  let collisions : Array[Collision] = []
  for collision in self.collisions {
    collisions.push({
      ..collision,
      origin: rebase_origin(inv, collision.origin),
    })
  }
  let inertial = match self.inertial {
    Some(inertial) =>
      Some({ ..inertial, origin: rebase_origin(inv, inertial.origin), })
    None => None
  }
  { ..self, inertial, visuals, collisions, }
}

///|
/// Computes world poses for every link reachable from the robot's root
/// link(s). Mimic joints are resolved before propagation.
pub fn Robot::forward_kinematics(
  self : Robot,
  joint_positions~ : Map[String, Double] = Map([]),
) -> Map[String, Transform] {
  let poses : Map[String, Transform] = Map([])
  for link in self.links {
    let mut is_child = false
    for joint in self.joints {
      if joint.child == link.name {
        is_child = true
      }
    }
    if !is_child {
      poses.set(link.name, Transform::identity())
    }
  }
  let q_map : Map[String, Double] = Map([])
  for joint in self.joints {
    if joint.mimic is None {
      q_map.set(joint.name, joint_positions.get(joint.name).unwrap_or(0.0))
    }
  }
  let n = self.joints.length()
  // Resolve mimic chains (possibly chained) in repeated passes.
  for _ in 0..<n {
    for joint in self.joints {
      if joint.mimic is Some(mimic) {
        if q_map.get(mimic.joint) is Some(q_ref) {
          q_map.set(joint.name, mimic.multiplier * q_ref + mimic.offset)
        }
      }
    }
  }
  // Propagate parent -> child until no more links can be resolved, so the
  // order of joints in the robot does not matter.
  for _ in 0..<n {
    for joint in self.joints {
      if poses.get(joint.parent) is Some(t_parent) &&
        !poses.contains(joint.child) {
        let t_origin = Transform::from_origin(joint.origin)
        let q = q_map.get(joint.name).unwrap_or(0.0)
        let u = v3_norm(joint.axis)
        let motion = match joint.joint_type {
          Revolute | Continuous =>
            { rot: rotation_matrix_axis_angle(u, q), trans: (0.0, 0.0, 0.0), }
          Prismatic => { rot: Matrix3x3::identity(), trans: v3_scale(u, q), }
          _ => Transform::identity()
        }
        poses.set(joint.child, t_parent.compose(t_origin).compose(motion))
      }
    }
  }
  poses
}

///|
/// Convenience wrapper returning each link's world origin (xyz + rpy).
pub fn Robot::link_world_origins(
  self : Robot,
  joint_positions~ : Map[String, Double] = Map([]),
) -> Map[String, Origin] {
  let poses = self.forward_kinematics(joint_positions~)
  let out : Map[String, Origin] = Map([])
  for name, transform in poses {
    out.set(name, transform.to_origin())
  }
  out
}

///|
/// MuJoCo MJCF serialization.

///|
/// One actuator declaration for the MJCF `<actuator>` section.
///
/// The tuple fields are `(joint, name, ...)`; use the snake-case constructors
/// (`MjcfActuator::motor`, `::position`, `::velocity`) for readable call sites.
#derive(Debug, Eq)
pub(all) enum MjcfActuator {
  /// Torque/force motor: joint, name, ctrl_min, ctrl_max, gear.
  Motor(String, String, Double, Double, Double)
  /// Position servo: joint, name, kp, kv, ctrl_min, ctrl_max, forcerange.
  Position(String, String, Double, Double, Double, Double, Double?)
  /// Velocity servo: joint, name, kv, ctrl_min, ctrl_max, forcerange.
  Velocity(String, String, Double, Double, Double, Double?)
}

///|
pub fn MjcfActuator::motor(
  joint~ : String,
  name~ : String = "",
  ctrl_min~ : Double,
  ctrl_max~ : Double,
  gear~ : Double = 1.0,
) -> MjcfActuator {
  Motor(joint, name, ctrl_min, ctrl_max, gear)
}

///|
pub fn MjcfActuator::position(
  joint~ : String,
  name~ : String = "",
  kp~ : Double,
  kv~ : Double,
  ctrl_min~ : Double,
  ctrl_max~ : Double,
  forcerange? : Double,
) -> MjcfActuator {
  Position(joint, name, kp, kv, ctrl_min, ctrl_max, forcerange)
}

///|
pub fn MjcfActuator::velocity(
  joint~ : String,
  name~ : String = "",
  kv~ : Double,
  ctrl_min~ : Double,
  ctrl_max~ : Double,
  forcerange? : Double,
) -> MjcfActuator {
  Velocity(joint, name, kv, ctrl_min, ctrl_max, forcerange)
}

///|
/// Simulation-oriented MJCF options.
///
/// The defaults reproduce the original exporter's structure; the CAD compiler
/// fills these in for a standing-pose, contact-clean RL model.
pub(all) struct MjcfOptions {
  mesh_dir : String
  include_floor : Bool
  /// World Z lift for the root free body. Computed from the home pose's
  /// lowest collision geometry when `None` and `home_qpos` is non-empty.
  root_z_m : Double?
  /// Home/standing joint positions (joint name -> radians or metres).
  home_qpos : Map[String, Double]
  /// Explicit actuators. Empty keeps the legacy "motor for every limited joint"
  /// behavior.
  actuators : Array[MjcfActuator]
  /// Joints to omit from the model (welded bodies), e.g. cosmetic rotors.
  freeze_joints : Array[String]
  /// Default joint damping (N*m*s/rad) and reflected armature (kg*m^2).
  default_damping : Double
  default_armature : Double
  /// Emit `<contact><exclude>` pairs for adjacent and loop-constrained bodies.
  exclude_adjacent_contacts : Bool
}

///|
pub fn MjcfOptions::new(
  mesh_dir~ : String = "meshes",
  include_floor~ : Bool = true,
  root_z_m~ : Double? = None,
  home_qpos~ : Map[String, Double] = Map([]),
  actuators~ : Array[MjcfActuator] = [],
  freeze_joints~ : Array[String] = [],
  default_damping~ : Double = 0.0,
  default_armature~ : Double = 0.0,
  exclude_adjacent_contacts~ : Bool = false,
) -> MjcfOptions {
  {
    mesh_dir,
    include_floor,
    root_z_m,
    home_qpos,
    actuators,
    freeze_joints,
    default_damping,
    default_armature,
    exclude_adjacent_contacts,
  }
}

///|
priv struct MjcfAssets {
  meshes : Map[String, String]
  materials : Map[String, String]
}

///|
fn mjcf_quat(rpy : Vec3) -> String {
  let (roll, pitch, yaw) = rpy
  let cr = @math.cos(roll * 0.5)
  let sr = @math.sin(roll * 0.5)
  let cp = @math.cos(pitch * 0.5)
  let sp = @math.sin(pitch * 0.5)
  let cy = @math.cos(yaw * 0.5)
  let sy = @math.sin(yaw * 0.5)
  let w = cr * cp * cy + sr * sp * sy
  let x = sr * cp * cy - cr * sp * sy
  let y = cr * sp * cy + sr * cp * sy
  let z = cr * cp * sy - sr * sp * cy
  "\{fmt_num(w)} \{fmt_num(x)} \{fmt_num(y)} \{fmt_num(z)}"
}

///|
fn mjcf_basename(path : String) -> String {
  match path.rev_find("/") {
    Some(index) =>
      String::unsafe_substring(path, start=index + 1, end=path.length())
    None => path
  }
}

///|
fn mjcf_mesh_name(path : String) -> String {
  let base = mjcf_basename(path)
  base
  .replace_all(old="\\", new="_")
  .replace_all(old=".", new="_")
  .replace_all(old="-", new="_")
  .replace_all(old=" ", new="_")
}

///|
fn mjcf_name(name : String) -> String {
  name
  .replace_all(old="/", new="_")
  .replace_all(old="\\", new="_")
  .replace_all(old=".", new="_")
  .replace_all(old="-", new="_")
  .replace_all(old=" ", new="_")
}

///|
fn mjcf_sorted_keys(map : Map[String, String]) -> Array[String] {
  let keys : Array[String] = []
  for key, _ in map {
    keys.push(key)
  }
  keys.sort_by(fn(a : String, b : String) -> Int32 {
    if a < b {
      -1
    } else if a > b {
      1
    } else {
      0
    }
  })
  keys
}

///|
fn mjcf_collect_assets(robot : Robot) -> MjcfAssets {
  let assets : MjcfAssets = { meshes: Map([]), materials: Map([]), }
  for material in robot.materials {
    match material.color {
      Some(color) => assets.materials.set(material.name, fmt_rgba(color))
      None => ()
    }
  }
  for link in robot.links {
    // Always collect visual materials so collision geoms inherit each link's
    // CAD colors in MuJoCo.
    for visual in link.visuals {
      match visual.material {
        Some(mat) =>
          match mat.color {
            Some(color) => assets.materials.set(mat.name, fmt_rgba(color))
            None => ()
          }
        None => ()
      }
    }
    // Only the geoms that are actually emitted contribute mesh assets:
    // collisions take precedence over visuals, matching `mjcf_write_body`.
    if link.collisions.length() > 0 {
      for collision in link.collisions {
        match collision.geometry {
          Mesh(path, _) =>
            assets.meshes.set(mjcf_mesh_name(path), mjcf_basename(path))
          _ => ()
        }
      }
    } else {
      for visual in link.visuals {
        match visual.geometry {
          Mesh(path, _) =>
            assets.meshes.set(mjcf_mesh_name(path), mjcf_basename(path))
          _ => ()
        }
        match visual.material {
          Some(material) =>
            match material.color {
              Some(color) =>
                if !assets.materials.contains(material.name) {
                  assets.materials.set(material.name, fmt_rgba(color))
                }
              None => ()
            }
          None => ()
        }
      }
    }
  }
  assets
}

///|
fn mjcf_write_inertial(
  sb : StringBuilder,
  inertial : Inertial,
  level : Int32,
) -> Unit {
  let (x, y, z) = inertial.origin.xyz
  let (roll, pitch, yaw) = inertial.origin.rpy
  pad(sb, level)
  sb.write_string(
    "<inertial pos=\"" +
    fmt_vec3((x, y, z)) +
    "\"" +
    (if roll.abs() + pitch.abs() + yaw.abs() > 1.0e-12 {
      " quat=\"" + mjcf_quat((roll, pitch, yaw)) + "\""
    } else {
      ""
    }) +
    " mass=\"" +
    fmt_num(inertial.mass) +
    "\" fullinertia=\"" +
    fmt_num(inertial.inertia.ixx) +
    " " +
    fmt_num(inertial.inertia.iyy) +
    " " +
    fmt_num(inertial.inertia.izz) +
    " " +
    fmt_num(inertial.inertia.ixy) +
    " " +
    fmt_num(inertial.inertia.ixz) +
    " " +
    fmt_num(inertial.inertia.iyz) +
    "\"/>\n",
  )
}

///|
fn mjcf_is_frozen(options : MjcfOptions, joint : Joint) -> Bool {
  options.freeze_joints.contains(joint.name)
}

///|
fn mjcf_write_joint(
  sb : StringBuilder,
  joint : Joint,
  options : MjcfOptions,
  qpos_names : Array[String],
  level : Int32,
) -> Unit {
  if mjcf_is_frozen(options, joint) {
    return
  }
  match joint.joint_type {
    Fixed => ()
    Floating => {
      pad(sb, level)
      sb.write_string("<freejoint/>\n")
    }
    Revolute | Continuous => {
      qpos_names.push(joint.name)
      pad(sb, level)
      sb.write_string(
        "<joint name=\"" +
        xml_escape(joint.name) +
        "\" type=\"hinge\" axis=\"" +
        fmt_vec3(joint.axis) +
        "\"",
      )
      match joint.limit {
        Some(limit) =>
          match (limit.lower, limit.upper) {
            (Some(lower), Some(upper)) =>
              sb.write_string(
                " range=\"" +
                fmt_num(lower) +
                " " +
                fmt_num(upper) +
                "\" limited=\"true\"",
              )
            _ => sb.write_string(" limited=\"false\"")
          }
        None => sb.write_string(" limited=\"false\"")
      }
      sb.write_string("/>\n")
    }
    Prismatic => {
      qpos_names.push(joint.name)
      pad(sb, level)
      sb.write_string(
        "<joint name=\"" +
        xml_escape(joint.name) +
        "\" type=\"slide\" axis=\"" +
        fmt_vec3(joint.axis) +
        "\"",
      )
      match joint.limit {
        Some(limit) =>
          match (limit.lower, limit.upper) {
            (Some(lower), Some(upper)) =>
              sb.write_string(
                " range=\"" +
                fmt_num(lower) +
                " " +
                fmt_num(upper) +
                "\" limited=\"true\"",
              )
            _ => sb.write_string(" limited=\"false\"")
          }
        None => sb.write_string(" limited=\"false\"")
      }
      sb.write_string("/>\n")
    }
    Planar => {
      qpos_names.push(joint.name)
      qpos_names.push(joint.name + "_y")
      pad(sb, level)
      sb.write_string(
        "<joint name=\"" +
        xml_escape(joint.name) +
        "\" type=\"slide\" axis=\"1 0 0\"/>\n",
      )
      pad(sb, level)
      sb.write_string(
        "<joint name=\"" +
        xml_escape(joint.name) +
        "_y\" type=\"slide\" axis=\"0 1 0\"/>\n",
      )
    }
  }
}

///|
/// Contact class for one geom: tires/wheels/feet collide with the floor only;
/// structural parts collide with the floor if the robot falls over; internal
/// actuator hardware (stators, gears, bearings, screws) never collides, so a
/// bulky bounding box inside a wheel cannot lift the robot onto its calf.
fn mjcf_contact_attrs(link_name : String, part_name : String) -> String {
  let part = part_name.to_lower()
  let internal = part.contains("stator") ||
    part.contains("_act_") ||
    part.contains("act_a12") ||
    part.contains("planet") ||
    part.contains("herringbone") ||
    part.contains("ring") ||
    part.contains("sun") ||
    part.contains("gear") ||
    part.contains("moteus") ||
    part.contains("pcb") ||
    part.contains("bearing") ||
    part.contains("_pin") ||
    part.contains("shaft") ||
    part.contains("screw") ||
    part.contains("bolt") ||
    part.contains("iso4762")
  if internal {
    return " contype=\"0\" conaffinity=\"0\""
  }
  let lower = link_name.to_lower()
  if lower.contains("tire") || lower.contains("wheel") || lower.contains("foot") {
    " contype=\"2\" conaffinity=\"1\" friction=\"1.2 0.01 0.001\""
  } else {
    " contype=\"4\" conaffinity=\"1\" friction=\"1.0 0.01 0.001\""
  }
}

///|
fn mjcf_geom_min_z(t : Transform, geometry : Geometry) -> Double? {
  match geometry {
    Box(size) => {
      let (sx, sy, sz) = size
      let xs : Array[Double] = [-sx / 2.0, sx / 2.0]
      let ys : Array[Double] = [-sy / 2.0, sy / 2.0]
      let zs : Array[Double] = [-sz / 2.0, sz / 2.0]
      let mut lowest = 1.0e18
      for x in xs {
        for y in ys {
          for z in zs {
            let p = t.apply_point((x, y, z))
            if p.2 < lowest {
              lowest = p.2
            }
          }
        }
      }
      Some(lowest)
    }
    Cylinder(radius, length) => {
      let axis = t.apply_dir((0.0, 0.0, 1.0))
      let vertical = if axis.2.abs() > 1.0 { 1.0 } else { axis.2.abs() }
      let horizontal = (1.0 - vertical * vertical).sqrt()
      Some(t.trans.2 - (length / 2.0 * vertical + radius * horizontal))
    }
    Sphere(radius) => Some(t.trans.2 - radius)
    Mesh(_, _) => None
  }
}

///|
fn mjcf_min_world_z(robot : Robot, qpos : Map[String, Double]) -> Double? {
  let fk = robot.forward_kinematics(joint_positions=qpos)
  let mut found = false
  let mut lowest = 1.0e18
  for link in robot.links {
    let world = fk.get(link.name).unwrap_or(Transform::identity())
    let geoms : Array[(Origin, Geometry)] = if link.collisions.length() > 0 {
      link.collisions.map(fn(collision) {
        (collision.origin, collision.geometry)
      })
    } else {
      link.visuals.map(fn(visual) { (visual.origin, visual.geometry) })
    }
    for entry in geoms {
      let frame = world.compose(Transform::from_origin(entry.0))
      match mjcf_geom_min_z(frame, entry.1) {
        Some(z) => {
          if z < lowest {
            lowest = z
          }
          found = true
        }
        None => ()
      }
    }
  }
  if found {
    Some(lowest)
  } else {
    None
  }
}

///|
/// Sites used by `<connect site1 site2>` for each loop constraint: the first
/// anchor lives on body1, the second on body2 (both in their link frames).
fn mjcf_loop_sites(robot : Robot) -> Map[String, Array[(String, Vec3)]] {
  let sites : Map[String, Array[(String, Vec3)]] = Map([])
  for constraint in robot.loop_constraints {
    let base = mjcf_name(constraint.name)
    let first = (base + "_site1", constraint.anchor1)
    let second = (base + "_site2", constraint.anchor2)
    match sites.get(constraint.body1) {
      Some(entries) => entries.push(first)
      None => sites.set(constraint.body1, [first])
    }
    match sites.get(constraint.body2) {
      Some(entries) => entries.push(second)
      None => sites.set(constraint.body2, [second])
    }
  }
  sites
}

///|
fn mjcf_pair_key(a : String, b : String) -> String {
  if a < b {
    a + "|" + b
  } else {
    b + "|" + a
  }
}

///|
fn mjcf_add_pair(
  pairs : Array[(String, String)],
  keys : Array[String],
  a : String,
  b : String,
) -> Unit {
  if a == b {
    return
  }
  let key = mjcf_pair_key(a, b)
  if !keys.contains(key) {
    keys.push(key)
    pairs.push((a, b))
  }
}

///|
fn mjcf_contact_pairs(robot : Robot) -> Array[(String, String)] {
  let pairs : Array[(String, String)] = []
  let keys : Array[String] = []
  for joint in robot.joints {
    mjcf_add_pair(pairs, keys, joint.parent, joint.child)
  }
  for constraint in robot.loop_constraints {
    mjcf_add_pair(pairs, keys, constraint.body1, constraint.body2)
  }
  let groups : Map[String, Array[String]] = Map([])
  for joint in robot.joints {
    match groups.get(joint.parent) {
      Some(children) => children.push(joint.child)
      None => groups.set(joint.parent, [joint.child])
    }
  }
  for _parent, children in groups {
    for i in 0..<children.length() {
      for j in (i + 1)..<children.length() {
        mjcf_add_pair(pairs, keys, children[i], children[j])
      }
    }
  }
  pairs
}

///|
fn mjcf_write_geom(
  sb : StringBuilder,
  name : String,
  origin : Origin,
  geometry : Geometry,
  material : Material?,
  assets : MjcfAssets,
  link_name : String,
  part_name : String,
  level : Int32,
) -> Unit {
  pad(sb, level)
  sb.write_string("<geom name=\"" + xml_escape(name) + "\"")
  match geometry {
    Box(size) => {
      let (sx, sy, sz) = size
      sb.write_string(
        " type=\"box\" size=\"" +
        fmt_vec3((sx / 2.0, sy / 2.0, sz / 2.0)) +
        "\"",
      )
    }
    Cylinder(radius, length) =>
      sb.write_string(
        " type=\"cylinder\" size=\"" +
        fmt_num(radius) +
        " " +
        fmt_num(length / 2.0) +
        "\"",
      )
    Sphere(radius) =>
      sb.write_string(" type=\"sphere\" size=\"" + fmt_num(radius) + "\"")
    Mesh(path, _) =>
      sb.write_string(
        " type=\"mesh\" mesh=\"" + xml_escape(mjcf_mesh_name(path)) + "\"",
      )
  }
  sb.write_string(" pos=\"" + fmt_vec3(origin.xyz) + "\"")
  let (roll, pitch, yaw) = origin.rpy
  if roll.abs() + pitch.abs() + yaw.abs() > 1.0e-12 {
    sb.write_string(" quat=\"" + mjcf_quat((roll, pitch, yaw)) + "\"")
  }
  match material {
    Some(mat) =>
      if assets.materials.contains(mat.name) {
        sb.write_string(" material=\"" + xml_escape(mat.name) + "\"")
      }
    None => ()
  }
  sb.write_string(mjcf_contact_attrs(link_name, part_name))
  sb.write_string("/>\n")
}

///|
fn mjcf_write_body(
  sb : StringBuilder,
  robot : Robot,
  link : Link,
  incoming_joint : Joint?,
  level : Int32,
  links : Map[String, Link],
  children : Map[String, Array[Joint]],
  assets : MjcfAssets,
  imu_sites : Array[String],
  options : MjcfOptions,
  root_z : Double,
  loop_sites : Map[String, Array[(String, Vec3)]],
  qpos_names : Array[String],
) -> Unit {
  pad(sb, level)
  sb.write_string("<body name=\"" + xml_escape(link.name) + "\"")
  match incoming_joint {
    Some(joint) =>
      sb.write_string(
        " pos=\"" +
        fmt_vec3(joint.origin.xyz) +
        "\" quat=\"" +
        mjcf_quat(joint.origin.rpy) +
        "\"",
      )
    None => sb.write_string(" pos=\"0 0 " + fmt_num(root_z) + "\"")
  }
  sb.write_string(">\n")
  match incoming_joint {
    None => {
      pad(sb, level + 1)
      sb.write_string("<freejoint/>\n")
    }
    Some(joint) => mjcf_write_joint(sb, joint, options, qpos_names, level + 1)
  }
  match link.inertial {
    Some(inertial) => mjcf_write_inertial(sb, inertial, level + 1)
    None => ()
  }
  let visual_materials : Map[String, Material] = Map([])
  let mut fallback_material : Material? = None
  for visual in link.visuals {
    match visual.material {
      Some(mat) => {
        if fallback_material is None {
          fallback_material = Some(mat)
        }
        match visual.name {
          Some(vname) => visual_materials.set(vname, mat)
          None => ()
        }
      }
      None => ()
    }
  }
  // MuJoCo requires unique geom names; link name + ordinal is unique across
  // the model even when a STEP component repeats a layer name.
  let mut geom_index = 0
  if link.collisions.length() > 0 {
    for collision in link.collisions {
      let part_name = match collision.name {
        Some(value) => value
        None => link.name
      }
      let mat = match visual_materials.get(part_name) {
        Some(m) => Some(m)
        None =>
          if geom_index < link.visuals.length() {
            match link.visuals[geom_index].material {
              Some(m) => Some(m)
              None => fallback_material
            }
          } else {
            fallback_material
          }
      }
      mjcf_write_geom(
        sb,
        link.name + "_geom_" + geom_index.to_string(),
        collision.origin,
        collision.geometry,
        mat,
        assets,
        link.name,
        part_name,
        level + 1,
      )
      geom_index = geom_index + 1
    }
  } else {
    for visual in link.visuals {
      mjcf_write_geom(
        sb,
        link.name + "_geom_" + geom_index.to_string(),
        visual.origin,
        visual.geometry,
        visual.material,
        assets,
        link.name,
        link.name,
        level + 1,
      )
      geom_index = geom_index + 1
    }
  }
  if link.name.to_lower().contains("imu") {
    let site = link.name + "_imu"
    pad(sb, level + 1)
    sb.write_string(
      "<site name=\"" + xml_escape(site) + "\" pos=\"0 0 0\" size=\"0.005\"/>\n",
    )
    imu_sites.push(site)
  }
  // Loop-constraint sites: `<connect site1 site2>` ties these exact points.
  match loop_sites.get(link.name) {
    Some(entries) =>
      for entry in entries {
        pad(sb, level + 1)
        sb.write_string(
          "<site name=\"" +
          xml_escape(entry.0) +
          "\" pos=\"" +
          fmt_vec3(entry.1) +
          "\" size=\"0.005\"/>\n",
        )
      }
    None => ()
  }
  match children.get(link.name) {
    Some(outgoing) =>
      for joint in outgoing {
        match links.get(joint.child) {
          Some(child_link) =>
            mjcf_write_body(
              sb,
              robot,
              child_link,
              Some(joint),
              level + 1,
              links,
              children,
              assets,
              imu_sites,
              options,
              root_z,
              loop_sites,
              qpos_names,
            )
          None => ()
        }
      }
    None => ()
  }
  pad(sb, level)
  sb.write_string("</body>\n")
}

///|
fn mjcf_forcerange(value : Double?) -> String {
  match value {
    Some(limit) =>
      " forcerange=\"-" + fmt_num(limit) + " " + fmt_num(limit) + "\""
    None => ""
  }
}

///|
fn mjcf_write_actuator(sb : StringBuilder, actuator : MjcfActuator) -> Unit {
  match actuator {
    Motor(joint, name, ctrl_min, ctrl_max, gear) => {
      pad(sb, 2)
      sb.write_string(
        "<motor name=\"" +
        xml_escape(if name == "" { joint + "_motor" } else { name }) +
        "\" joint=\"" +
        xml_escape(joint) +
        "\" ctrlrange=\"" +
        fmt_num(ctrl_min) +
        " " +
        fmt_num(ctrl_max) +
        "\"" +
        (if (gear - 1.0).abs() > 1.0e-9 {
          " gear=\"" + fmt_num(gear) + "\""
        } else {
          ""
        }) +
        "/>\n",
      )
    }
    Position(joint, name, kp, kv, ctrl_min, ctrl_max, forcerange) => {
      pad(sb, 2)
      sb.write_string(
        "<position name=\"" +
        xml_escape(if name == "" { joint + "_pos" } else { name }) +
        "\" joint=\"" +
        xml_escape(joint) +
        "\" kp=\"" +
        fmt_num(kp) +
        "\" kv=\"" +
        fmt_num(kv) +
        "\" ctrlrange=\"" +
        fmt_num(ctrl_min) +
        " " +
        fmt_num(ctrl_max) +
        "\"" +
        mjcf_forcerange(forcerange) +
        "/>\n",
      )
    }
    Velocity(joint, name, kv, ctrl_min, ctrl_max, forcerange) => {
      pad(sb, 2)
      sb.write_string(
        "<velocity name=\"" +
        xml_escape(if name == "" { joint + "_vel" } else { name }) +
        "\" joint=\"" +
        xml_escape(joint) +
        "\" kv=\"" +
        fmt_num(kv) +
        "\" ctrlrange=\"" +
        fmt_num(ctrl_min) +
        " " +
        fmt_num(ctrl_max) +
        "\"" +
        mjcf_forcerange(forcerange) +
        "/>\n",
      )
    }
  }
}

///|
fn mjcf_write_actuators(
  sb : StringBuilder,
  robot : Robot,
  options : MjcfOptions,
) -> Unit {
  let explicit = options.actuators
  let legacy : Array[Joint] = []
  if explicit.length() == 0 {
    for joint in robot.joints {
      let actuated = match joint.joint_type {
        Revolute | Continuous | Prismatic => true
        _ => false
      }
      if actuated && !mjcf_is_frozen(options, joint) {
        match joint.limit {
          Some(limit) => if limit.effort > 0.0 { legacy.push(joint) }
          None => ()
        }
      }
    }
  }
  if explicit.length() == 0 && legacy.length() == 0 {
    return
  }
  pad(sb, 1)
  sb.write_string("<actuator>\n")
  if explicit.length() > 0 {
    for actuator in explicit {
      mjcf_write_actuator(sb, actuator)
    }
  } else {
    for joint in legacy {
      let effort = match joint.limit {
        Some(limit) => limit.effort
        None => 0.0
      }
      mjcf_write_actuator(
        sb,
        MjcfActuator::motor(joint=joint.name, ctrl_min=-effort, ctrl_max=effort),
      )
    }
  }
  pad(sb, 1)
  sb.write_string("</actuator>\n")
}

///|
/// Converts the robot to MuJoCo MJCF XML with simulation options.
pub fn Robot::to_mjcf_with_options(
  self : Robot,
  options : MjcfOptions,
) -> String {
  let root_z = match options.root_z_m {
    Some(z) => z
    None =>
      if options.home_qpos.length() > 0 {
        match mjcf_min_world_z(self, options.home_qpos) {
          Some(lowest) => if lowest < 0.0 { -lowest + 0.004 } else { 0.0 }
          None => 0.0
        }
      } else {
        0.0
      }
  }
  let sb = StringBuilder()
  sb.write_string("<?xml version=\"1.0\"?>\n")
  sb.write_string("<mujoco model=\"" + xml_escape(self.name) + "\">\n")
  pad(sb, 1)
  sb.write_string(
    "<compiler angle=\"radian\" meshdir=\"" +
    xml_escape(options.mesh_dir) +
    "\"/>\n",
  )
  pad(sb, 1)
  sb.write_string("<option gravity=\"0 0 -9.81\" timestep=\"0.002\"/>\n")
  pad(sb, 1)
  sb.write_string(
    "<visual><headlight ambient=\"0.38 0.38 0.38\" diffuse=\"0.65 0.65 0.65\" specular=\"0.15 0.15 0.15\"/></visual>\n",
  )
  if options.default_damping > 0.0 || options.default_armature > 0.0 {
    pad(sb, 1)
    sb.write_string(
      "<default>\n" +
      "  <joint damping=\"" +
      fmt_num(options.default_damping) +
      "\" armature=\"" +
      fmt_num(options.default_armature) +
      "\"/>\n" +
      "</default>\n",
    )
  }

  let assets = mjcf_collect_assets(self)
  let mesh_names = mjcf_sorted_keys(assets.meshes)
  let material_names = mjcf_sorted_keys(assets.materials)
  if mesh_names.length() > 0 || material_names.length() > 0 {
    pad(sb, 1)
    sb.write_string("<asset>\n")
    for name in material_names {
      pad(sb, 2)
      sb.write_string(
        "<material name=\"" +
        xml_escape(name) +
        "\" rgba=\"" +
        assets.materials.get(name).unwrap_or("1 1 1 1") +
        "\"/>\n",
      )
    }
    for name in mesh_names {
      pad(sb, 2)
      sb.write_string(
        "<mesh name=\"" +
        xml_escape(name) +
        "\" file=\"" +
        xml_escape(assets.meshes.get(name).unwrap_or("")) +
        "\"/>\n",
      )
    }
    pad(sb, 1)
    sb.write_string("</asset>\n")
  }

  let links : Map[String, Link] = Map([])
  for link in self.links {
    links.set(link.name, link)
  }
  let children : Map[String, Array[Joint]] = Map([])
  for joint in self.joints {
    match children.get(joint.parent) {
      Some(outgoing) => outgoing.push(joint)
      None => children.set(joint.parent, [joint])
    }
  }
  let child_links : Array[String] = self.joints.map(fn(joint) { joint.child })
  let mut root_name = ""
  for link in self.links {
    if !child_links.contains(link.name) {
      root_name = link.name
      break
    }
  }
  if root_name == "" && self.links.length() > 0 {
    root_name = self.links[0].name
  }

  let loop_sites = mjcf_loop_sites(self)
  let imu_sites : Array[String] = []
  let qpos_names : Array[String] = []
  pad(sb, 1)
  sb.write_string("<worldbody>\n")
  pad(sb, 2)
  sb.write_string(
    "<light name=\"sun\" pos=\"0 0 3.5\" dir=\"0 0 -1\" diffuse=\"0.85 0.85 0.85\" specular=\"0.2 0.2 0.2\"/>\n",
  )
  if options.include_floor {
    pad(sb, 2)
    sb.write_string(
      "<geom name=\"floor\" type=\"plane\" size=\"0 0 0.05\" rgba=\"0.9 0.9 0.9 1\" contype=\"1\" conaffinity=\"1\" friction=\"1.0 0.01 0.001\"/>\n",
    )
  }
  match links.get(root_name) {
    Some(root_link) =>
      mjcf_write_body(
        sb,
        self,
        root_link,
        None,
        2,
        links,
        children,
        assets,
        imu_sites,
        options,
        root_z,
        loop_sites,
        qpos_names,
      )
    None => ()
  }
  pad(sb, 1)
  sb.write_string("</worldbody>\n")

  if options.exclude_adjacent_contacts {
    let pairs = mjcf_contact_pairs(self)
    if pairs.length() > 0 {
      pad(sb, 1)
      sb.write_string("<contact>\n")
      for pair in pairs {
        pad(sb, 2)
        sb.write_string(
          "<exclude body1=\"" +
          xml_escape(pair.0) +
          "\" body2=\"" +
          xml_escape(pair.1) +
          "\"/>\n",
        )
      }
      pad(sb, 1)
      sb.write_string("</contact>\n")
    }
  }

  if self.loop_constraints.length() > 0 {
    pad(sb, 1)
    sb.write_string("<equality>\n")
    for constraint in self.loop_constraints {
      let base = mjcf_name(constraint.name)
      pad(sb, 2)
      sb.write_string(
        "<connect name=\"" +
        xml_escape(constraint.name) +
        "\" site1=\"" +
        xml_escape(base + "_site1") +
        "\" site2=\"" +
        xml_escape(base + "_site2") +
        "\" solref=\"0.004 1\" solimp=\"0.99 0.999 0.001\"/>\n",
      )
    }
    pad(sb, 1)
    sb.write_string("</equality>\n")
  }

  mjcf_write_actuators(sb, self, options)

  if imu_sites.length() > 0 {
    pad(sb, 1)
    sb.write_string("<sensor>\n")
    for site in imu_sites {
      pad(sb, 2)
      sb.write_string(
        "<accelerometer name=\"" +
        xml_escape(site + "_accel") +
        "\" site=\"" +
        xml_escape(site) +
        "\"/>\n",
      )
      pad(sb, 2)
      sb.write_string(
        "<gyro name=\"" +
        xml_escape(site + "_gyro") +
        "\" site=\"" +
        xml_escape(site) +
        "\"/>\n",
      )
    }
    pad(sb, 1)
    sb.write_string("</sensor>\n")
  }

  if options.home_qpos.length() > 0 || options.root_z_m is Some(_) {
    let mut qpos = "0 0 " + fmt_num(root_z) + " 1 0 0 0"
    for name in qpos_names {
      qpos = qpos + " " + fmt_num(options.home_qpos.get(name).unwrap_or(0.0))
    }
    pad(sb, 1)
    sb.write_string("<keyframe>\n")
    pad(sb, 2)
    sb.write_string("<key name=\"home\" qpos=\"" + qpos + "\"/>\n")
    pad(sb, 1)
    sb.write_string("</keyframe>\n")
  }

  sb.write_string("</mujoco>\n")
  sb.to_string()
}

///|
/// Converts the robot to MuJoCo MJCF XML with the legacy defaults.
///
/// The root link is floating (`<freejoint/>`), collisions are used as geoms
/// when present (visuals otherwise), and every `LoopConstraint` becomes a
/// native `<equality><connect>` between two site frames (`site1`/`site2`)
/// placed at the constraint anchors.
pub fn Robot::to_mjcf_string(
  self : Robot,
  mesh_dir~ : String = "meshes",
  include_floor~ : Bool = true,
) -> String {
  self.to_mjcf_with_options(MjcfOptions::new(mesh_dir~, include_floor~))
}

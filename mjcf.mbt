///|
/// MuJoCo MJCF serialization.

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
    // Only the geoms that are actually emitted contribute assets: collisions
    // take precedence over visuals, matching `mjcf_write_body`. An unused
    // glTF visual asset would still be loaded by MuJoCo, so leave it out.
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
fn mjcf_write_joint(sb : StringBuilder, joint : Joint, level : Int32) -> Unit {
  match joint.joint_type {
    Fixed => ()
    Floating => {
      pad(sb, level)
      sb.write_string("<freejoint/>\n")
    }
    Revolute | Continuous => {
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
      pad(sb, level)
      sb.write_string(
        "<joint name=\"" +
        xml_escape(joint.name) +
        "\" type=\"slide\" axis=\"" +
        fmt_vec3(joint.axis) +
        "\"",
      )
      // Prismatic limits use the same range form as hinges.
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
fn mjcf_write_geom(
  sb : StringBuilder,
  name : String,
  origin : Origin,
  geometry : Geometry,
  material : Material?,
  assets : MjcfAssets,
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
    None => sb.write_string(" pos=\"0 0 0\"")
  }
  sb.write_string(">\n")
  match incoming_joint {
    None => {
      pad(sb, level + 1)
      sb.write_string("<freejoint/>\n")
    }
    Some(joint) => mjcf_write_joint(sb, joint, level + 1)
  }
  match link.inertial {
    Some(inertial) => mjcf_write_inertial(sb, inertial, level + 1)
    None => ()
  }
  // MuJoCo requires unique geom names; link name + ordinal is unique across
  // the model even when a STEP component repeats a layer name.
  let mut geom_index = 0
  if link.collisions.length() > 0 {
    for collision in link.collisions {
      mjcf_write_geom(
        sb,
        link.name + "_geom_" + geom_index.to_string(),
        collision.origin,
        collision.geometry,
        None,
        assets,
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
fn mjcf_write_actuators(sb : StringBuilder, robot : Robot) -> Unit {
  let motors : Array[Joint] = []
  for joint in robot.joints {
    let actuated = match joint.joint_type {
      Revolute | Continuous | Prismatic => true
      _ => false
    }
    match joint.limit {
      Some(limit) => if actuated && limit.effort > 0.0 { motors.push(joint) }
      None => ()
    }
  }
  if motors.length() > 0 {
    pad(sb, 1)
    sb.write_string("<actuator>\n")
    for joint in motors {
      let effort = match joint.limit {
        Some(limit) => limit.effort
        None => 0.0
      }
      pad(sb, 2)
      sb.write_string(
        "<motor name=\"" +
        xml_escape(joint.name + "_motor") +
        "\" joint=\"" +
        xml_escape(joint.name) +
        "\" ctrlrange=\"-" +
        fmt_num(effort) +
        " " +
        fmt_num(effort) +
        "\"/>\n",
      )
    }
    pad(sb, 1)
    sb.write_string("</actuator>\n")
  }
}

///|
/// Converts the robot to MuJoCo MJCF XML.
///
/// The root link is floating (`<freejoint/>`), collisions are used as geoms
/// when present (visuals otherwise), and every `LoopConstraint` becomes a
/// native `<equality><connect>`.
pub fn Robot::to_mjcf_string(
  self : Robot,
  mesh_dir~ : String = "meshes",
  include_floor~ : Bool = true,
) -> String {
  let sb = StringBuilder()
  sb.write_string("<?xml version=\"1.0\"?>\n")
  sb.write_string("<mujoco model=\"" + xml_escape(self.name) + "\">\n")
  pad(sb, 1)
  sb.write_string(
    "<compiler angle=\"radian\" meshdir=\"" + xml_escape(mesh_dir) + "\"/>\n",
  )
  pad(sb, 1)
  sb.write_string("<option gravity=\"0 0 -9.81\" timestep=\"0.002\"/>\n")

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

  let imu_sites : Array[String] = []
  pad(sb, 1)
  sb.write_string("<worldbody>\n")
  if include_floor {
    pad(sb, 2)
    sb.write_string(
      "<geom name=\"floor\" type=\"plane\" size=\"0 0 0.05\" rgba=\"0.9 0.9 0.9 1\"/>\n",
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
      )
    None => ()
  }
  pad(sb, 1)
  sb.write_string("</worldbody>\n")

  if self.loop_constraints.length() > 0 {
    pad(sb, 1)
    sb.write_string("<equality>\n")
    for constraint in self.loop_constraints {
      pad(sb, 2)
      sb.write_string(
        "<connect name=\"" +
        xml_escape(constraint.name) +
        "\" body1=\"" +
        xml_escape(constraint.body1) +
        "\" body2=\"" +
        xml_escape(constraint.body2) +
        "\" anchor=\"" +
        fmt_vec3(constraint.anchor1) +
        "\"/>\n",
      )
    }
    pad(sb, 1)
    sb.write_string("</equality>\n")
  }

  mjcf_write_actuators(sb, self)

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

  sb.write_string("</mujoco>\n")
  sb.to_string()
}

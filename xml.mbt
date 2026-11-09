///|
/// Deterministic URDF XML serialization.

///|
/// Formats a double for URDF attributes: trigonometric residue is rounded to
/// zero, near-integers print without a decimal point, and `-0` never appears.
pub fn fmt_num(n : Double) -> String {
  if n.abs() < 1.0e-12 {
    return "0"
  }
  let rounded = n.round()
  if (n - rounded).abs() < 1.0e-9 && n.abs() < 1.0e9 {
    return rounded.to_int().to_string()
  }
  n.to_string()
}

///|
pub fn fmt_vec3(v : Vec3) -> String {
  "\{fmt_num(v.0)} \{fmt_num(v.1)} \{fmt_num(v.2)}"
}

///|
pub fn fmt_rgba(c : Rgba) -> String {
  "\{fmt_num(c.r)} \{fmt_num(c.g)} \{fmt_num(c.b)} \{fmt_num(c.a)}"
}

///|
/// Escapes text or an attribute value for XML output.
pub fn xml_escape(s : String) -> String {
  s
  .replace_all(old="&", new="&amp;")
  .replace_all(old="<", new="&lt;")
  .replace_all(old=">", new="&gt;")
  .replace_all(old="\"", new="&quot;")
  .replace_all(old="'", new="&apos;")
}

///|
fn pad(sb : StringBuilder, level : Int32) -> Unit {
  for _ in 0..<level {
    sb.write_string("  ")
  }
}

///|
fn write_origin_line(
  sb : StringBuilder,
  level : Int32,
  origin : Origin,
  always : Bool,
) -> Unit {
  if always || !origin.is_identity() {
    pad(sb, level)
    sb.write_string(
      "<origin xyz=\"" +
      fmt_vec3(origin.xyz) +
      "\" rpy=\"" +
      fmt_vec3(origin.rpy) +
      "\"/>\n",
    )
  }
}

///|
fn write_geometry(
  sb : StringBuilder,
  level : Int32,
  geometry : Geometry,
) -> Unit {
  pad(sb, level)
  sb.write_string("<geometry>\n")
  let inner = level + 1
  pad(sb, inner)
  match geometry {
    Box((sx, sy, sz)) =>
      sb.write_string("<box size=\"" + fmt_vec3((sx, sy, sz)) + "\"/>\n")
    Cylinder(radius, length) =>
      sb.write_string(
        "<cylinder radius=\"" +
        fmt_num(radius) +
        "\" length=\"" +
        fmt_num(length) +
        "\"/>\n",
      )
    Sphere(radius) =>
      sb.write_string("<sphere radius=\"" + fmt_num(radius) + "\"/>\n")
    Mesh(filename, scale) => {
      sb.write_string("<mesh filename=\"" + xml_escape(filename) + "\"")
      match scale {
        Some(s) => sb.write_string(" scale=\"" + fmt_vec3(s) + "\"")
        None => ()
      }
      sb.write_string("/>\n")
    }
  }
  pad(sb, level)
  sb.write_string("</geometry>\n")
}

///|
fn write_material(
  sb : StringBuilder,
  level : Int32,
  material : Material,
) -> Unit {
  pad(sb, level)
  sb.write_string("<material name=\"" + xml_escape(material.name) + "\"")
  match (material.color, material.texture) {
    (None, None) => sb.write_string("/>\n")
    _ => {
      sb.write_string(">\n")
      match material.color {
        Some(color) => {
          pad(sb, level + 1)
          sb.write_string("<color rgba=\"" + fmt_rgba(color) + "\"/>\n")
        }
        None => ()
      }
      match material.texture {
        Some(texture) => {
          pad(sb, level + 1)
          sb.write_string(
            "<texture filename=\"" + xml_escape(texture) + "\"/>\n",
          )
        }
        None => ()
      }
      pad(sb, level)
      sb.write_string("</material>\n")
    }
  }
}

///|
fn write_extra(sb : StringBuilder, level : Int32, node : RawXmlNode) -> Unit {
  pad(sb, level)
  sb.write_string("<" + node.tag)
  for attr in node.attrs {
    sb.write_string(" " + attr.0 + "=\"" + xml_escape(attr.1) + "\"")
  }
  if node.children.is_empty() {
    if node.text == "" {
      sb.write_string("/>\n")
    } else {
      sb.write_string(">" + xml_escape(node.text) + "</" + node.tag + ">\n")
    }
  } else {
    sb.write_string(">\n")
    if node.text != "" {
      pad(sb, level + 1)
      sb.write_string(xml_escape(node.text) + "\n")
    }
    for child in node.children {
      write_extra(sb, level + 1, child)
    }
    pad(sb, level)
    sb.write_string("</" + node.tag + ">\n")
  }
}

///|
fn write_inertial(
  sb : StringBuilder,
  level : Int32,
  inertial : Inertial,
) -> Unit {
  pad(sb, level)
  sb.write_string("<inertial>\n")
  write_origin_line(sb, level + 1, inertial.origin, false)
  pad(sb, level + 1)
  sb.write_string("<mass value=\"" + fmt_num(inertial.mass) + "\"/>\n")
  let i = inertial.inertia
  pad(sb, level + 1)
  sb.write_string(
    "<inertia ixx=\"" +
    fmt_num(i.ixx) +
    "\" ixy=\"" +
    fmt_num(i.ixy) +
    "\" ixz=\"" +
    fmt_num(i.ixz) +
    "\" iyy=\"" +
    fmt_num(i.iyy) +
    "\" iyz=\"" +
    fmt_num(i.iyz) +
    "\" izz=\"" +
    fmt_num(i.izz) +
    "\"/>\n",
  )
  pad(sb, level)
  sb.write_string("</inertial>\n")
}

///|
fn write_visual(sb : StringBuilder, level : Int32, visual : Visual) -> Unit {
  pad(sb, level)
  sb.write_string("<visual")
  match visual.name {
    Some(name) => sb.write_string(" name=\"" + xml_escape(name) + "\"")
    None => ()
  }
  sb.write_string(">\n")
  write_origin_line(sb, level + 1, visual.origin, false)
  write_geometry(sb, level + 1, visual.geometry)
  match visual.material {
    Some(material) => write_material(sb, level + 1, material)
    None => ()
  }
  pad(sb, level)
  sb.write_string("</visual>\n")
}

///|
fn write_collision(
  sb : StringBuilder,
  level : Int32,
  collision : Collision,
) -> Unit {
  pad(sb, level)
  sb.write_string("<collision")
  match collision.name {
    Some(name) => sb.write_string(" name=\"" + xml_escape(name) + "\"")
    None => ()
  }
  sb.write_string(">\n")
  write_origin_line(sb, level + 1, collision.origin, false)
  write_geometry(sb, level + 1, collision.geometry)
  pad(sb, level)
  sb.write_string("</collision>\n")
}

///|
fn write_link(sb : StringBuilder, level : Int32, link : Link) -> Unit {
  let has_children = link.inertial is Some(_) ||
    !link.visuals.is_empty() ||
    !link.collisions.is_empty() ||
    !link.extra.is_empty()
  pad(sb, level)
  if !has_children {
    sb.write_string("<link name=\"" + xml_escape(link.name) + "\"/>\n")
    return
  }
  sb.write_string("<link name=\"" + xml_escape(link.name) + "\">\n")
  match link.inertial {
    Some(inertial) => write_inertial(sb, level + 1, inertial)
    None => ()
  }
  for visual in link.visuals {
    write_visual(sb, level + 1, visual)
  }
  for collision in link.collisions {
    write_collision(sb, level + 1, collision)
  }
  for node in link.extra {
    write_extra(sb, level + 1, node)
  }
  pad(sb, level)
  sb.write_string("</link>\n")
}

///|
fn write_joint(sb : StringBuilder, level : Int32, joint : Joint) -> Unit {
  pad(sb, level)
  sb.write_string(
    "<joint name=\"" +
    xml_escape(joint.name) +
    "\" type=\"" +
    joint.joint_type.to_urdf_str() +
    "\">\n",
  )
  write_origin_line(sb, level + 1, joint.origin, true)
  pad(sb, level + 1)
  sb.write_string("<parent link=\"" + xml_escape(joint.parent) + "\"/>\n")
  pad(sb, level + 1)
  sb.write_string("<child link=\"" + xml_escape(joint.child) + "\"/>\n")
  match joint.joint_type {
    Revolute | Continuous | Prismatic | Planar => {
      pad(sb, level + 1)
      sb.write_string("<axis xyz=\"" + fmt_vec3(joint.axis) + "\"/>\n")
    }
    _ => ()
  }
  match joint.limit {
    Some(limit) => {
      pad(sb, level + 1)
      sb.write_string("<limit")
      match limit.lower {
        Some(v) => sb.write_string(" lower=\"" + fmt_num(v) + "\"")
        None => ()
      }
      match limit.upper {
        Some(v) => sb.write_string(" upper=\"" + fmt_num(v) + "\"")
        None => ()
      }
      sb.write_string(
        " effort=\"" +
        fmt_num(limit.effort) +
        "\" velocity=\"" +
        fmt_num(limit.velocity) +
        "\"/>\n",
      )
    }
    None => ()
  }
  match joint.dynamics {
    Some(dynamics) => {
      pad(sb, level + 1)
      sb.write_string(
        "<dynamics damping=\"" +
        fmt_num(dynamics.damping) +
        "\" friction=\"" +
        fmt_num(dynamics.friction) +
        "\"/>\n",
      )
    }
    None => ()
  }
  match joint.calibration {
    Some(calibration) => {
      pad(sb, level + 1)
      sb.write_string("<calibration")
      match calibration.rising {
        Some(v) => sb.write_string(" rising=\"" + fmt_num(v) + "\"")
        None => ()
      }
      match calibration.falling {
        Some(v) => sb.write_string(" falling=\"" + fmt_num(v) + "\"")
        None => ()
      }
      sb.write_string("/>\n")
    }
    None => ()
  }
  match joint.safety_controller {
    Some(controller) => {
      pad(sb, level + 1)
      sb.write_string(
        "<safety_controller soft_lower_limit=\"" +
        fmt_num(controller.soft_lower_limit) +
        "\" soft_upper_limit=\"" +
        fmt_num(controller.soft_upper_limit) +
        "\" k_position=\"" +
        fmt_num(controller.k_position) +
        "\" k_velocity=\"" +
        fmt_num(controller.k_velocity) +
        "\"/>\n",
      )
    }
    None => ()
  }
  match joint.mimic {
    Some(mimic) => {
      pad(sb, level + 1)
      sb.write_string(
        "<mimic joint=\"" +
        xml_escape(mimic.joint) +
        "\" multiplier=\"" +
        fmt_num(mimic.multiplier) +
        "\" offset=\"" +
        fmt_num(mimic.offset) +
        "\"/>\n",
      )
    }
    None => ()
  }
  for node in joint.extra {
    write_extra(sb, level + 1, node)
  }
  pad(sb, level)
  sb.write_string("</joint>\n")
}

///|
fn write_transmission(
  sb : StringBuilder,
  level : Int32,
  transmission : Transmission,
) -> Unit {
  pad(sb, level)
  sb.write_string(
    "<transmission name=\"" + xml_escape(transmission.name) + "\">\n",
  )
  pad(sb, level + 1)
  sb.write_string("<type>" + xml_escape(transmission.trans_type) + "</type>\n")
  for joint in transmission.joints {
    pad(sb, level + 1)
    sb.write_string("<joint name=\"" + xml_escape(joint.name) + "\">\n")
    for interface in joint.hardware_interfaces {
      pad(sb, level + 2)
      sb.write_string(
        "<hardwareInterface>" + xml_escape(interface) + "</hardwareInterface>\n",
      )
    }
    pad(sb, level + 1)
    sb.write_string("</joint>\n")
  }
  for actuator in transmission.actuators {
    pad(sb, level + 1)
    sb.write_string("<actuator name=\"" + xml_escape(actuator.name) + "\">\n")
    for interface in actuator.hardware_interfaces {
      pad(sb, level + 2)
      sb.write_string(
        "<hardwareInterface>" + xml_escape(interface) + "</hardwareInterface>\n",
      )
    }
    match actuator.mechanical_reduction {
      Some(reduction) => {
        pad(sb, level + 2)
        sb.write_string(
          "<mechanicalReduction>" +
          fmt_num(reduction) +
          "</mechanicalReduction>\n",
        )
      }
      None => ()
    }
    pad(sb, level + 1)
    sb.write_string("</actuator>\n")
  }
  pad(sb, level)
  sb.write_string("</transmission>\n")
}

///|
/// Serializes the robot to a deterministic URDF XML string.
pub fn Robot::to_urdf_xml(self : Robot) -> String {
  let sb = StringBuilder()
  sb.write_string("<?xml version=\"1.0\"?>\n")
  if self.documentation != "" {
    let sanitized = self.documentation.replace_all(old="--", new="- -")
    sb.write_string("<!-- " + sanitized + " -->\n")
  }
  sb.write_string("<robot name=\"" + xml_escape(self.name) + "\"")
  match self.version {
    Some(version) => sb.write_string(" version=\"" + xml_escape(version) + "\"")
    None => ()
  }
  sb.write_string(">\n")
  for material in self.materials {
    write_material(sb, 1, material)
  }
  for link in self.links {
    write_link(sb, 1, link)
  }
  for joint in self.joints {
    write_joint(sb, 1, joint)
  }
  for transmission in self.transmissions {
    write_transmission(sb, 1, transmission)
  }
  for node in self.extra {
    write_extra(sb, 1, node)
  }
  sb.write_string("</robot>\n")
  sb.to_string()
}

///|
/// Alias for `to_urdf_xml`, matching the usual `to_string` convention.
pub fn Robot::to_string(self : Robot) -> String {
  self.to_urdf_xml()
}

///|
/// Writes text to `out_path`, or to stdout when `out_path` is `None`.
pub async fn write_text_file(text : String, out_path : String?) -> Unit {
  match out_path {
    Some(path) =>
      @fs.write_file(
        path,
        @encoding/utf8.encode(text),
        permission=0o644,
        create_mode=@fs.CreateMode::CreateOrTruncate,
      )
    None => println(text)
  }
}

///|
/// Writes the robot's URDF XML to a file or stdout. When `checked` is true,
/// validation diagnostics abort the write on the first error.
pub async fn Robot::write_urdf_file(
  self : Robot,
  out_path : String?,
  checked~ : Bool = false,
) -> Unit {
  let text = if checked {
    self.to_urdf_file_checked()
  } else {
    self.to_urdf_xml()
  }
  write_text_file(text, out_path)
}

///|
/// Alias for `write_urdf_file`.
pub async fn Robot::write_urdf(
  self : Robot,
  out_path : String?,
  checked~ : Bool = false,
) -> Unit {
  self.write_urdf_file(out_path, checked~)
}

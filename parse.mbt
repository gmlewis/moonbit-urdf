///|
/// Lossless URDF XML parsing.

///|
/// An XML element together with its source location, kept internally so that
/// conversion errors can point at the offending line and column.
struct XNode {
  tag : String
  attrs : Array[(String, String)]
  attr_pos : Array[(Int32, Int32)]
  text : String
  children : Array[XNode]
  line : Int32
  col : Int32
}

///|
struct XmlParser {
  text : String
  pos : Array[Int]
  line : Array[Int32]
  col : Array[Int32]
}

///|
fn xp_new(text : String) -> XmlParser {
  { text, pos: [0], line: [1], col: [1] }
}

///|
fn xp_len(p : XmlParser) -> Int {
  p.text.length()
}

///|
fn xp_at(p : XmlParser, i : Int) -> Int {
  if i < 0 || i >= xp_len(p) {
    -1
  } else {
    UInt16::to_int(String::code_unit_at(p.text, i))
  }
}

///|
fn xp_peek(p : XmlParser) -> Int {
  xp_at(p, p.pos[0])
}

///|
fn xp_advance(p : XmlParser) -> Unit {
  let c = xp_peek(p)
  if c < 0 {
    return
  }
  p.pos[0] = p.pos[0] + 1
  if c == Char::to_int('\n') {
    p.line[0] = p.line[0] + 1
    p.col[0] = 1
  } else {
    p.col[0] = p.col[0] + 1
  }
}

///|
fn xp_starts(p : XmlParser, s : String) -> Bool {
  let n = s.length()
  if p.pos[0] + n > xp_len(p) {
    return false
  }
  for i in 0..<n {
    if xp_at(p, p.pos[0] + i) != UInt16::to_int(String::code_unit_at(s, i)) {
      return false
    }
  }
  true
}

///|
fn xp_substr(p : XmlParser, start : Int, end : Int) -> String {
  String::unsafe_substring(p.text, start~, end~)
}

///|
fn xp_snippet(p : XmlParser) -> String {
  let n = xp_len(p)
  let i = p.pos[0]
  let mut start = i
  while start > 0 && xp_at(p, start - 1) != Char::to_int('\n') {
    start = start - 1
  }
  let mut end = i
  while end < n && xp_at(p, end) != Char::to_int('\n') {
    end = end + 1
  }
  xp_substr(p, start, end)
}

///|
fn xp_error(p : XmlParser, message : String) -> Unit raise UrdfParseError {
  raise UrdfParseError({
    message,
    line: Some(p.line[0]),
    col: Some(p.col[0]),
    snippet: Some(xp_snippet(p)),
  })
}

///|
fn is_space_code(c : Int) -> Bool {
  c == Char::to_int(' ') ||
  c == Char::to_int('\t') ||
  c == Char::to_int('\r') ||
  c == Char::to_int('\n')
}

///|
fn xp_skip_ws(p : XmlParser) -> Unit {
  while is_space_code(xp_peek(p)) {
    xp_advance(p)
  }
}

///|
fn is_name_char(c : Int) -> Bool {
  c >= 0 &&
  c != Char::to_int(' ') &&
  c != Char::to_int('\t') &&
  c != Char::to_int('\r') &&
  c != Char::to_int('\n') &&
  c != Char::to_int('<') &&
  c != Char::to_int('>') &&
  c != Char::to_int('/') &&
  c != Char::to_int('=')
}

///|
fn xp_skip_pi(p : XmlParser) -> Unit raise UrdfParseError {
  // Consume "<?", then everything up to "?>".
  xp_advance(p)
  xp_advance(p)
  while true {
    if p.pos[0] >= xp_len(p) {
      xp_error(p, "unterminated processing instruction")
    }
    if xp_starts(p, "?>") {
      xp_advance(p)
      xp_advance(p)
      return
    }
    xp_advance(p)
  }
}

///|
fn xp_skip_comment(p : XmlParser) -> String raise UrdfParseError {
  // Consume "<!--".
  for _ in 0..<4 {
    xp_advance(p)
  }
  let start = p.pos[0]
  while true {
    if p.pos[0] >= xp_len(p) {
      xp_error(p, "unterminated comment")
    }
    if xp_starts(p, "-->") {
      let content = xp_substr(p, start, p.pos[0])
      xp_advance(p)
      xp_advance(p)
      xp_advance(p)
      return content
    }
    xp_advance(p)
  }
  ""
}

///|
fn xp_skip_misc(p : XmlParser) -> Unit raise UrdfParseError {
  while true {
    xp_skip_ws(p)
    if xp_starts(p, "<!--") {
      let _ = xp_skip_comment(p)
      continue
    }
    if xp_starts(p, "<?") {
      xp_skip_pi(p)
      continue
    }
    break
  }
}

///|
fn xml_unescape(s : String) -> String {
  s.replace_all(old="&lt;", new="<")
  .replace_all(old="&gt;", new=">")
  .replace_all(old="&quot;", new="\"")
  .replace_all(old="&apos;", new="'")
  .replace_all(old="&amp;", new="&")
}

///|
fn xp_parse_element(p : XmlParser) -> XNode raise UrdfParseError {
  xp_skip_misc(p)
  if xp_peek(p) != Char::to_int('<') {
    xp_error(p, "expected element start '<'")
  }
  let line = p.line[0]
  let col = p.col[0]
  xp_advance(p)
  if xp_peek(p) == Char::to_int('/') {
    xp_error(p, "unexpected closing tag")
  }
  let name_start = p.pos[0]
  while is_name_char(xp_peek(p)) {
    xp_advance(p)
  }
  if p.pos[0] == name_start {
    xp_error(p, "expected element name")
  }
  let tag = xp_substr(p, name_start, p.pos[0])
  let attrs : Array[(String, String)] = []
  let attr_pos : Array[(Int32, Int32)] = []
  let mut self_closing = false
  while true {
    xp_skip_ws(p)
    let c = xp_peek(p)
    if c < 0 {
      xp_error(p, "unclosed tag <\{tag}>")
    }
    if c == Char::to_int('>') {
      xp_advance(p)
      break
    }
    if c == Char::to_int('/') {
      xp_advance(p)
      if xp_peek(p) != Char::to_int('>') {
        xp_error(p, "expected '>' after '/'")
      }
      xp_advance(p)
      self_closing = true
      break
    }
    let a_line = p.line[0]
    let a_col = p.col[0]
    let a_start = p.pos[0]
    while is_name_char(xp_peek(p)) {
      xp_advance(p)
    }
    if p.pos[0] == a_start {
      xp_error(p, "expected attribute name")
    }
    let attr_name = xp_substr(p, a_start, p.pos[0])
    xp_skip_ws(p)
    if xp_peek(p) != Char::to_int('=') {
      xp_error(p, "expected '=' after attribute \"\{attr_name}\"")
    }
    xp_advance(p)
    xp_skip_ws(p)
    let quote = xp_peek(p)
    if quote != Char::to_int('"') && quote != Char::to_int('\'') {
      xp_error(p, "expected quoted value for attribute \"\{attr_name}\"")
    }
    xp_advance(p)
    let v_start = p.pos[0]
    while true {
      if p.pos[0] >= xp_len(p) {
        xp_error(p, "unterminated attribute string")
      }
      if xp_peek(p) == quote {
        break
      }
      xp_advance(p)
    }
    let raw = xp_substr(p, v_start, p.pos[0])
    xp_advance(p)
    attrs.push((attr_name, xml_unescape(raw)))
    attr_pos.push((a_line, a_col))
  }
  if self_closing {
    return {
      tag,
      attrs,
      attr_pos,
      text: "",
      children: [],
      line,
      col,
    }
  }
  let children : Array[XNode] = []
  let texts : Array[String] = []
  while true {
    if p.pos[0] >= xp_len(p) {
      xp_error(p, "unclosed tag <\{tag}>")
    }
    if xp_starts(p, "</") {
      xp_advance(p)
      xp_advance(p)
      let close_start = p.pos[0]
      while is_name_char(xp_peek(p)) {
        xp_advance(p)
      }
      let close_tag = xp_substr(p, close_start, p.pos[0])
      xp_skip_ws(p)
      if xp_peek(p) != Char::to_int('>') {
        xp_error(p, "expected '>' in closing tag")
      }
      xp_advance(p)
      if close_tag != tag {
        xp_error(
          p,
          "mismatched closing tag: expected </\{tag}> but found </\{close_tag}>",
        )
      }
      break
    }
    if xp_starts(p, "<!--") {
      let _ = xp_skip_comment(p)
      continue
    }
    if xp_starts(p, "<?") {
      xp_skip_pi(p)
      continue
    }
    if xp_starts(p, "<![CDATA[") {
      for _ in 0..<9 {
        xp_advance(p)
      }
      let cd_start = p.pos[0]
      while !xp_starts(p, "]]>") {
        if p.pos[0] >= xp_len(p) {
          xp_error(p, "unterminated CDATA section")
        }
        xp_advance(p)
      }
      texts.push(xp_substr(p, cd_start, p.pos[0]))
      xp_advance(p)
      xp_advance(p)
      xp_advance(p)
      continue
    }
    if xp_peek(p) == Char::to_int('<') {
      children.push(xp_parse_element(p))
      continue
    }
    let t_start = p.pos[0]
    while p.pos[0] < xp_len(p) && xp_peek(p) != Char::to_int('<') {
      xp_advance(p)
    }
    texts.push(xml_unescape(xp_substr(p, t_start, p.pos[0])))
  }
  let joined = texts.join("").trim().to_owned()
  {
    tag,
    attrs,
    attr_pos,
    text: joined,
    children,
    line,
    col,
  }
}

///|
fn parse_located_document(xml : String) -> (XNode, String?) raise UrdfParseError {
  let p = xp_new(xml)
  xp_skip_ws(p)
  // Leading processing instructions may appear before any comment.
  while xp_starts(p, "<?") {
    xp_skip_pi(p)
    xp_skip_ws(p)
  }
  let mut documentation : String? = None
  if xp_starts(p, "<!--") {
    documentation = Some(xp_skip_comment(p).trim().to_owned())
  }
  xp_skip_misc(p)
  if !xp_starts(p, "<") {
    xp_error(p, "expected <robot> root element")
  }
  let root = xp_parse_element(p)
  xp_skip_misc(p)
  if p.pos[0] < xp_len(p) {
    xp_error(p, "unexpected content after root element")
  }
  (root, documentation)
}

///|
fn to_raw(node : XNode) -> RawXmlNode {
  let children : Array[RawXmlNode] = []
  for child in node.children {
    children.push(to_raw(child))
  }
  { tag: node.tag, attrs: node.attrs, text: node.text, children }
}

///|
fn xn_attr(node : XNode, name : String) -> String? {
  for (key, value) in node.attrs {
    if key == name {
      return Some(value)
    }
  }
  None
}

///|
fn xn_attr_pos(node : XNode, name : String) -> (Int32, Int32) {
  let mut i = 0
  for (key, _value) in node.attrs {
    if key == name {
      return node.attr_pos[i]
    }
    i = i + 1
  }
  (node.line, node.col)
}

///|
fn xn_attr_required(node : XNode, name : String) -> String raise UrdfParseError {
  match xn_attr(node, name) {
    Some(value) => value
    None =>
      raise UrdfParseError({
        message: "missing required attribute \"\{name}\" on <\{node.tag}>",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
}

///|
fn xn_child(node : XNode, tag : String) -> XNode? {
  for child in node.children {
    if child.tag == tag {
      return Some(child)
    }
  }
  None
}

///|
fn xn_children(node : XNode, tag : String) -> Array[XNode] {
  let out : Array[XNode] = []
  for child in node.children {
    if child.tag == tag {
      out.push(child)
    }
  }
  out
}

///|
fn xn_extra(node : XNode, known : Array[String]) -> Array[RawXmlNode] {
  let out : Array[RawXmlNode] = []
  for child in node.children {
    let mut is_known = false
    for tag in known {
      if child.tag == tag {
        is_known = true
      }
    }
    if !is_known {
      out.push(to_raw(child))
    }
  }
  out
}

///|
fn is_space_view(c : Int) -> Bool {
  c == Char::to_int(' ') ||
  c == Char::to_int('\t') ||
  c == Char::to_int('\r') ||
  c == Char::to_int('\n')
}

///|
fn split_ws(value : String) -> Array[String] {
  let out : Array[String] = []
  let n = value.length()
  let mut i = 0
  while i < n {
    while i < n && is_space_view(UInt16::to_int(String::code_unit_at(value, i))) {
      i = i + 1
    }
    if i >= n {
      break
    }
    let start = i
    while i < n && !is_space_view(UInt16::to_int(String::code_unit_at(value, i))) {
      i = i + 1
    }
    out.push(String::unsafe_substring(value, start~, end=i))
  }
  out
}

///|
fn parse_double_at(
  raw : String,
  line : Int32,
  col : Int32,
  context : String,
) -> Double raise UrdfParseError {
  let text = raw.trim().to_owned()
  try @string.parse_double(text) catch {
    _ =>
      raise UrdfParseError({
        message: "invalid float \"\{raw}\" in \{context}",
        line: Some(line),
        col: Some(col),
        snippet: None,
      })
  } noraise {
    value => value
  }
}

///|
fn parse_vec3_attr(
  node : XNode,
  name : String,
  default : Vec3,
) -> Vec3 raise UrdfParseError {
  match xn_attr(node, name) {
    None => default
    Some(raw) => {
      let parts = split_ws(raw)
      let (line, col) = xn_attr_pos(node, name)
      if parts.length() != 3 {
        raise UrdfParseError({
          message: "expected 3 numbers in \{name}=\"\{raw}\"",
          line: Some(line),
          col: Some(col),
          snippet: None,
        })
      }
      (
        parse_double_at(parts[0], line, col, name),
        parse_double_at(parts[1], line, col, name),
        parse_double_at(parts[2], line, col, name),
      )
    }
  }
}

///|
fn parse_rgba_attr(
  node : XNode,
  name : String,
  default : Rgba,
) -> Rgba raise UrdfParseError {
  match xn_attr(node, name) {
    None => default
    Some(raw) => {
      let parts = split_ws(raw)
      let (line, col) = xn_attr_pos(node, name)
      if parts.length() != 4 {
        raise UrdfParseError({
          message: "expected 4 numbers in \{name}=\"\{raw}\"",
          line: Some(line),
          col: Some(col),
          snippet: None,
        })
      }
      Rgba::new(
        parse_double_at(parts[0], line, col, name),
        parse_double_at(parts[1], line, col, name),
        parse_double_at(parts[2], line, col, name),
        a=parse_double_at(parts[3], line, col, name),
      )
    }
  }
}

///|
fn parse_double_attr(
  node : XNode,
  name : String,
  default : Double,
) -> Double raise UrdfParseError {
  match xn_attr(node, name) {
    None => default
    Some(raw) => {
      let (line, col) = xn_attr_pos(node, name)
      parse_double_at(raw, line, col, name)
    }
  }
}

///|
fn parse_origin(node : XNode?) -> Origin raise UrdfParseError {
  match node {
    None => Origin::identity()
    Some(o) =>
      Origin::new(
        xyz=parse_vec3_attr(o, "xyz", (0.0, 0.0, 0.0)),
        rpy=parse_vec3_attr(o, "rpy", (0.0, 0.0, 0.0)),
      )
  }
}

///|
fn parse_geometry(node : XNode) -> Geometry raise UrdfParseError {
  for child in node.children {
    match child.tag {
      "box" => {
        let size = parse_vec3_attr(child, "size", (0.0, 0.0, 0.0))
        return Box(size)
      }
      "cylinder" => {
        let radius = parse_double_attr(child, "radius", 0.0)
        let length = parse_double_attr(child, "length", 0.0)
        return Cylinder(radius, length)
      }
      "sphere" => {
        let radius = parse_double_attr(child, "radius", 0.0)
        return Sphere(radius)
      }
      "mesh" => {
        let filename = xn_attr_required(child, "filename")
        let scale = match xn_attr(child, "scale") {
          None => None
          Some(_) => Some(parse_vec3_attr(child, "scale", (1.0, 1.0, 1.0)))
        }
        return Mesh(filename, scale)
      }
      _ => ()
    }
  }
  raise UrdfParseError({
    message: "missing geometry element (box, cylinder, sphere, or mesh)",
    line: Some(node.line),
    col: Some(node.col),
    snippet: None,
  })
}

///|
fn parse_material(node : XNode) -> Material raise UrdfParseError {
  let name = xn_attr_required(node, "name")
  let color = match xn_child(node, "color") {
    Some(c) => Some(parse_rgba_attr(c, "rgba", Rgba::new(1.0, 1.0, 1.0)))
    None => None
  }
  let texture = match xn_child(node, "texture") {
    Some(t) => Some(xn_attr_required(t, "filename"))
    None => None
  }
  Material::new(name, color~, texture~)
}

///|
fn parse_inertial(node : XNode) -> Inertial raise UrdfParseError {
  let origin = parse_origin(xn_child(node, "origin"))
  let mass = match xn_child(node, "mass") {
    Some(m) => parse_double_attr(m, "value", 0.0)
    None => 0.0
  }
  let inertia = match xn_child(node, "inertia") {
    Some(i) =>
      Inertia::new(
        ixx=parse_double_attr(i, "ixx", 0.0),
        iyy=parse_double_attr(i, "iyy", 0.0),
        izz=parse_double_attr(i, "izz", 0.0),
        ixy=parse_double_attr(i, "ixy", 0.0),
        ixz=parse_double_attr(i, "ixz", 0.0),
        iyz=parse_double_attr(i, "iyz", 0.0),
      )
    None => Inertia::zero()
  }
  Inertial::new(mass, inertia, origin~)
}

///|
fn parse_visual(node : XNode) -> Visual raise UrdfParseError {
  let name = xn_attr(node, "name")
  let origin = parse_origin(xn_child(node, "origin"))
  let geometry = match xn_child(node, "geometry") {
    Some(g) => parse_geometry(g)
    None =>
      raise UrdfParseError({
        message: "missing <geometry> in <visual>",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
  let material = match xn_child(node, "material") {
    Some(m) => Some(parse_material(m))
    None => None
  }
  Visual::new(geometry, name~, origin~, material~)
}

///|
fn parse_collision(node : XNode) -> Collision raise UrdfParseError {
  let name = xn_attr(node, "name")
  let origin = parse_origin(xn_child(node, "origin"))
  let geometry = match xn_child(node, "geometry") {
    Some(g) => parse_geometry(g)
    None =>
      raise UrdfParseError({
        message: "missing <geometry> in <collision>",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
  Collision::new(geometry, name~, origin~)
}

///|
fn parse_link(node : XNode) -> Link raise UrdfParseError {
  let name = xn_attr_required(node, "name")
  let inertial = match xn_child(node, "inertial") {
    Some(i) => Some(parse_inertial(i))
    None => None
  }
  let visuals : Array[Visual] = []
  for child in xn_children(node, "visual") {
    visuals.push(parse_visual(child))
  }
  let collisions : Array[Collision] = []
  for child in xn_children(node, "collision") {
    collisions.push(parse_collision(child))
  }
  let extra = xn_extra(node, ["inertial", "visual", "collision"])
  { name, inertial, visuals, collisions, extra }
}

///|
fn parse_joint_type(node : XNode) -> JointType raise UrdfParseError {
  let raw = xn_attr_required(node, "type")
  match JointType::from_urdf_str(raw) {
    Some(jt) => jt
    None =>
      raise UrdfParseError({
        message: "unknown joint type \"\{raw}\"",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
}

///|
fn parse_limit(node : XNode) -> JointLimit raise UrdfParseError {
  let lower = match xn_attr(node, "lower") {
    Some(_) => Some(parse_double_attr(node, "lower", 0.0))
    None => None
  }
  let upper = match xn_attr(node, "upper") {
    Some(_) => Some(parse_double_attr(node, "upper", 0.0))
    None => None
  }
  JointLimit::new(
    effort=parse_double_attr(node, "effort", 0.0),
    velocity=parse_double_attr(node, "velocity", 0.0),
    lower~,
    upper~,
  )
}

///|
fn parse_joint(node : XNode) -> Joint raise UrdfParseError {
  let name = xn_attr_required(node, "name")
  let joint_type = parse_joint_type(node)
  let origin = parse_origin(xn_child(node, "origin"))
  let parent = match xn_child(node, "parent") {
    Some(p) => xn_attr_required(p, "link")
    None =>
      raise UrdfParseError({
        message: "missing <parent> in <joint> \"\{name}\"",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
  let child = match xn_child(node, "child") {
    Some(c) => xn_attr_required(c, "link")
    None =>
      raise UrdfParseError({
        message: "missing <child> in <joint> \"\{name}\"",
        line: Some(node.line),
        col: Some(node.col),
        snippet: None,
      })
  }
  let axis = match xn_child(node, "axis") {
    Some(a) => parse_vec3_attr(a, "xyz", (1.0, 0.0, 0.0))
    None => (1.0, 0.0, 0.0)
  }
  let limit = match xn_child(node, "limit") {
    Some(l) => Some(parse_limit(l))
    None => None
  }
  let dynamics = match xn_child(node, "dynamics") {
    Some(d) =>
      Some(
        JointDynamics::new(
          damping=parse_double_attr(d, "damping", 0.0),
          friction=parse_double_attr(d, "friction", 0.0),
        ),
      )
    None => None
  }
  let calibration = match xn_child(node, "calibration") {
    Some(c) => {
      let rising = match xn_attr(c, "rising") {
        Some(_) => Some(parse_double_attr(c, "rising", 0.0))
        None => None
      }
      let falling = match xn_attr(c, "falling") {
        Some(_) => Some(parse_double_attr(c, "falling", 0.0))
        None => None
      }
      Some({ rising, falling })
    }
    None => None
  }
  let safety_controller = match xn_child(node, "safety_controller") {
    Some(sc) =>
      Some({
        soft_lower_limit: parse_double_attr(sc, "soft_lower_limit", 0.0),
        soft_upper_limit: parse_double_attr(sc, "soft_upper_limit", 0.0),
        k_position: parse_double_attr(sc, "k_position", 0.0),
        k_velocity: parse_double_attr(sc, "k_velocity", 0.0),
      })
    None => None
  }
  let mimic = match xn_child(node, "mimic") {
    Some(m) =>
      Some(
        JointMimic::new(
          xn_attr_required(m, "joint"),
          multiplier=parse_double_attr(m, "multiplier", 1.0),
          offset=parse_double_attr(m, "offset", 0.0),
        ),
      )
    None => None
  }
  let extra = xn_extra(
    node,
    [
      "origin",
      "parent",
      "child",
      "axis",
      "limit",
      "dynamics",
      "calibration",
      "safety_controller",
      "mimic",
    ],
  )
  {
    name,
    joint_type,
    origin,
    parent,
    child,
    axis,
    limit,
    dynamics,
    calibration,
    safety_controller,
    mimic,
    extra,
  }
}

///|
fn parse_transmission(node : XNode) -> Transmission raise UrdfParseError {
  let name = xn_attr_required(node, "name")
  let trans_type = match xn_child(node, "type") {
    Some(t) => t.text
    None => ""
  }
  let joints : Array[TransmissionJoint] = []
  for joint_node in xn_children(node, "joint") {
    let interfaces : Array[String] = []
    for child in xn_children(joint_node, "hardwareInterface") {
      interfaces.push(child.text)
    }
    joints.push({
      name: xn_attr_required(joint_node, "name"),
      hardware_interfaces: interfaces,
    })
  }
  let actuators : Array[Actuator] = []
  for actuator_node in xn_children(node, "actuator") {
    let interfaces : Array[String] = []
    for child in xn_children(actuator_node, "hardwareInterface") {
      interfaces.push(child.text)
    }
    let reduction = match xn_child(actuator_node, "mechanicalReduction") {
      Some(r) => Some(parse_double_at(r.text, r.line, r.col, "mechanicalReduction"))
      None => None
    }
    actuators.push({
      name: xn_attr_required(actuator_node, "name"),
      hardware_interfaces: interfaces,
      mechanical_reduction: reduction,
    })
  }
  { name, trans_type, joints, actuators }
}

///|
fn robot_from_node(root : XNode, documentation : String) -> Robot raise UrdfParseError {
  if root.tag != "robot" {
    raise UrdfParseError({
      message: "expected <robot> root element, found <\{root.tag}>",
      line: Some(root.line),
      col: Some(root.col),
      snippet: None,
    })
  }
  let name = xn_attr_required(root, "name")
  let version = xn_attr(root, "version")
  let materials : Array[Material] = []
  for child in xn_children(root, "material") {
    materials.push(parse_material(child))
  }
  let links : Array[Link] = []
  for child in xn_children(root, "link") {
    links.push(parse_link(child))
  }
  let joints : Array[Joint] = []
  for child in xn_children(root, "joint") {
    joints.push(parse_joint(child))
  }
  let transmissions : Array[Transmission] = []
  for child in xn_children(root, "transmission") {
    transmissions.push(parse_transmission(child))
  }
  let extra = xn_extra(
    root,
    ["material", "link", "joint", "transmission"],
  )
  {
    name,
    version,
    documentation,
    materials,
    links,
    joints,
    transmissions,
    extra,
  }
}

///|
/// Parses XML text into a generic `RawXmlNode` tree.
pub fn parse_xml_node(xml : String) -> RawXmlNode raise UrdfParseError {
  let (root, _documentation) = parse_located_document(xml)
  to_raw(root)
}

///|
/// Parses a URDF XML string into a `Robot`.
pub fn parse_urdf_from_string(xml : String) -> Robot raise UrdfParseError {
  let (root, documentation) = parse_located_document(xml)
  robot_from_node(root, documentation.unwrap_or(""))
}

///|
/// Parses a URDF XML string, returning `ParseUrdfResult` instead of raising.
pub fn parse_urdf_from_string_result(xml : String) -> ParseUrdfResult {
  try parse_urdf_from_string(xml) catch {
    UrdfParseError(info) => Err(info)
  } noraise {
    robot => Ok(robot)
  }
}

///|
/// Reads and parses a URDF file.
pub async fn parse_urdf_from_file(path : String) -> Robot raise {
  let text = @fs.read_file(path).text()
  parse_urdf_from_string(text)
}

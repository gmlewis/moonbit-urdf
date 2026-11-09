///|
test "number formatting" {
  assert_eq(@urdf.fmt_num(1.0), "1")
  assert_eq(@urdf.fmt_num(-0.0), "0")
  assert_eq(@urdf.fmt_num(1.0e-13), "0")
  assert_eq(@urdf.fmt_num(0.5), "0.5")
  assert_eq(@urdf.fmt_num(2.0000000001), "2")
  assert_eq(@urdf.fmt_num(-3.25), "-3.25")
}

///|
test "vector and color formatting" {
  assert_eq(@urdf.fmt_vec3((1.0, 2.5, -0.0)), "1 2.5 0")
  assert_eq(@urdf.fmt_rgba(@urdf.Rgba::new(1.0, 0.5, 0.0, a=1.0)), "1 0.5 0 1")
}

///|
test "xml escaping" {
  assert_eq(
    @urdf.xml_escape("a&b<c>\"d\"'e'"),
    "a&amp;b&lt;c&gt;&quot;d&quot;&apos;e&apos;",
  )
}

///|
test "serializes a complete robot deterministically" {
  let robot = @urdf.Robot::new(
      "demo",
      version=Some("1.0"),
      documentation="hello",
    )
    .add_material(@urdf.Material::color_only("orange", @urdf.Rgba::orange()))
    .add_link(
      @urdf.Link::new("base")
      .with_inertial(@urdf.Inertial::from_box(1.0, 2.0, 3.0, mass=6.0))
      .add_visual(
        @urdf.Visual::new(@urdf.Geometry::box(1.0, 2.0, 3.0))
        .with_name("body")
        .with_material(
          @urdf.Material::color_only("orange", @urdf.Rgba::orange()),
        ),
      ),
    )
    .add_link(@urdf.Link::new("empty"))
    .add_joint(@urdf.Joint::fixed("j", parent="base", child="empty"))
  let expected = "<?xml version=\"1.0\"?>\n" +
    "<!-- hello -->\n" +
    "<robot name=\"demo\" version=\"1.0\">\n" +
    "  <material name=\"orange\">\n" +
    "    <color rgba=\"1 0.6 0 1\"/>\n" +
    "  </material>\n" +
    "  <link name=\"base\">\n" +
    "    <inertial>\n" +
    "      <mass value=\"6\"/>\n" +
    "      <inertia ixx=\"6.5\" ixy=\"0\" ixz=\"0\" iyy=\"5\" iyz=\"0\" izz=\"2.5\"/>\n" +
    "    </inertial>\n" +
    "    <visual name=\"body\">\n" +
    "      <geometry>\n" +
    "        <box size=\"1 2 3\"/>\n" +
    "      </geometry>\n" +
    "      <material name=\"orange\">\n" +
    "        <color rgba=\"1 0.6 0 1\"/>\n" +
    "      </material>\n" +
    "    </visual>\n" +
    "  </link>\n" +
    "  <link name=\"empty\"/>\n" +
    "  <joint name=\"j\" type=\"fixed\">\n" +
    "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n" +
    "    <parent link=\"base\"/>\n" +
    "    <child link=\"empty\"/>\n" +
    "  </joint>\n" +
    "</robot>\n"
  assert_eq(robot.to_urdf_xml(), expected)
  assert_eq(robot.to_string(), expected)
}

///|
test "serializes meshes limits dynamics and mimic" {
  let robot = @urdf.Robot::new("r")
    .add_link(
      @urdf.Link::new("a").add_visual(
        @urdf.Visual::new(
          @urdf.Geometry::mesh(
            "package://pkg/a.gltf",
            scale=Some((0.001, 0.001, 0.001)),
          ),
        ),
      ),
    )
    .add_link(@urdf.Link::new("b"))
    .add_joint(
      @urdf.Joint::revolute(
        "j",
        parent="a",
        child="b",
        axis=(0.0, 0.0, 1.0),
        limit=@urdf.JointLimit::new(
          lower=Some(-1.0),
          upper=Some(1.0),
          effort=2.0,
          velocity=3.0,
        ),
      )
      .with_dynamics(@urdf.JointDynamics::new(damping=0.1, friction=0.2))
      .with_mimic(@urdf.JointMimic::new("other", multiplier=1.0)),
    )
  let xml = robot.to_urdf_xml()
  assert_true(
    xml.contains(
      "<mesh filename=\"package://pkg/a.gltf\" scale=\"0.001 0.001 0.001\"/>",
    ),
  )
  assert_true(
    xml.contains(
      "<limit lower=\"-1\" upper=\"1\" effort=\"2\" velocity=\"3\"/>",
    ),
  )
  assert_true(xml.contains("<dynamics damping=\"0.1\" friction=\"0.2\"/>"))
  assert_true(
    xml.contains("<mimic joint=\"other\" multiplier=\"1\" offset=\"0\"/>"),
  )
  assert_true(xml.contains("<axis xyz=\"0 0 1\"/>"))
}

///|
test "serializes transmissions and extra nodes" {
  let robot = @urdf.Robot::new("r")
    .add_link(@urdf.Link::new("a"))
    .add_link(@urdf.Link::new("b"))
    .add_joint(@urdf.Joint::fixed("j", parent="a", child="b"))
    .add_transmission(
      @urdf.Transmission::simple("t", "j", "motor", reduction=10.0),
    )
    .add_extra(@urdf.RawXmlNode::{
      tag: "gazebo",
      attrs: [("reference", "a")],
      text: "",
      children: [
        @urdf.RawXmlNode::{ tag: "mu1", attrs: [], text: "0.8", children: [], },
      ],
    })
  let xml = robot.to_urdf_xml()
  assert_true(
    xml.contains("<type>transmission_interface/SimpleTransmission</type>"),
  )
  assert_true(xml.contains("<mechanicalReduction>10</mechanicalReduction>"))
  assert_true(xml.contains("<gazebo reference=\"a\">"))
  assert_true(xml.contains("<mu1>0.8</mu1>"))
}

///|
test "sanitizes documentation and escapes attributes" {
  let robot = @urdf.Robot::new("r", documentation="a -- b").add_link(
    @urdf.Link::new("l&m"),
  )
  let xml = robot.to_urdf_xml()
  assert_true(xml.contains("<!-- a - - b -->"))
  assert_true(xml.contains("<link name=\"l&amp;m\"/>"))
}

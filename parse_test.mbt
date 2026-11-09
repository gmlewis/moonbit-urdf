///|
fn sample_robot() -> @urdf.Robot {
  let extra_child : @urdf.RawXmlNode = {
    tag: "mu1",
    attrs: [],
    text: "0.8",
    children: [],
  }
  let extra : @urdf.RawXmlNode = {
    tag: "gazebo",
    attrs: [("reference", "base")],
    text: "",
    children: [extra_child],
  }
  @urdf.Robot::new(
    "roundtrip",
    version=Some("1.0"),
    documentation="roundtrip doc",
  )
  .add_material(@urdf.Material::color_only("orange", @urdf.Rgba::orange()))
  .add_link(
    @urdf.Link::new("base").with_inertial(
      @urdf.Inertial::from_box(0.2, 0.3, 0.4, mass=5.0),
    ).add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(0.2, 0.3, 0.4))
      .with_name("body")
      .with_color("orange", @urdf.Rgba::orange()),
    ).add_collision(
      @urdf.Collision::new(@urdf.Geometry::box(0.2, 0.3, 0.4)),
    ).add_extra(
      {
        tag: "sensor",
        attrs: [("name", "imu")],
        text: "",
        children: [],
      },
    ),
  )
  .add_link(@urdf.Link::new("arm"))
  .add_joint(
    @urdf.Joint::revolute(
      "j1",
      parent="base",
      child="arm",
      axis=(0.0, 1.0, 0.0),
      limit=@urdf.JointLimit::new(
        lower=Some(-1.0),
        upper=Some(1.0),
        effort=10.0,
        velocity=2.0,
      ),
    )
    .with_dynamics(@urdf.JointDynamics::new(damping=0.5, friction=0.1))
    .with_mimic(@urdf.JointMimic::new("other", multiplier=2.0)),
  )
  .add_transmission(
    @urdf.Transmission::simple("t", "j1", "motor", reduction=9.0),
  )
  .add_extra(extra)
}

///|
test "roundtrips a complete robot" {
  let robot = sample_robot()
  let xml = robot.to_urdf_xml()
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Ok(parsed) => {
      assert_true(parsed == robot)
      assert_eq(parsed.to_urdf_xml(), xml)
    }
    @urdf.ParseUrdfResult::Err(info) =>
      fail("parse failed: " + @urdf.format_urdf_parse_error(info))
  }
}

///|
test "roundtrips twice to a fixed point" {
  let xml = sample_robot().to_urdf_xml()
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Ok(first) => {
      let xml2 = first.to_urdf_xml()
      assert_eq(xml2, xml)
      match @urdf.parse_urdf_from_string_result(xml2) {
        @urdf.ParseUrdfResult::Ok(second) => assert_true(second == first)
        @urdf.ParseUrdfResult::Err(info) =>
          fail("second parse failed: " + @urdf.format_urdf_parse_error(info))
      }
    }
    @urdf.ParseUrdfResult::Err(info) =>
      fail("first parse failed: " + @urdf.format_urdf_parse_error(info))
  }
}

///|
test "preserves gazebo and ros2_control blocks" {
  let xml = "<robot name=\"r\">\n" +
    "  <link name=\"base\">\n" +
    "    <gazebo reference=\"base\">\n" +
    "      <mu1>0.8</mu1>\n" +
    "    </gazebo>\n" +
    "  </link>\n" +
    "  <ros2_control name=\"system\" type=\"system\">\n" +
    "    <hardware><plugin>mock</plugin></hardware>\n" +
    "  </ros2_control>\n" +
    "</robot>\n"
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Ok(robot) => {
      assert_eq(robot.extra.length(), 1)
      assert_eq(robot.extra[0].tag, "ros2_control")
      assert_eq(robot.links[0].extra.length(), 1)
      assert_eq(robot.links[0].extra[0].tag, "gazebo")
      let out = robot.to_urdf_xml()
      assert_true(out.contains("<mu1>0.8</mu1>"))
      assert_true(out.contains("<plugin>mock</plugin>"))
    }
    @urdf.ParseUrdfResult::Err(info) =>
      fail("parse failed: " + @urdf.format_urdf_parse_error(info))
  }
}

///|
test "parses generic xml nodes" {
  let xml = "<?xml version=\"1.0\"?>\n<foo a=\"1&amp;2\"><bar>hi</bar>tail</foo>\n"
  let parsed : @urdf.RawXmlNode? = try {
    Some(@urdf.parse_xml_node(xml))
  } catch {
    _ => None
  } noraise {
    v => v
  }
  match parsed {
    Some(node) => {
      assert_eq(node.tag, "foo")
      assert_eq(node.attrs[0], ("a", "1&2"))
      assert_eq(node.children.length(), 1)
      assert_eq(node.children[0].tag, "bar")
      assert_eq(node.children[0].text, "hi")
      assert_eq(node.text, "tail")
    }
    None => fail("unexpected parse error")
  }
}

///|
test "reports mismatched closing tags with a position" {
  let xml = "<robot name=\"r\">\n  <link name=\"a\">\n  </robot>\n"
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Err(info) => {
      assert_true(info.line is Some(_))
      assert_true(info.col is Some(_))
      let text = @urdf.format_urdf_parse_error(info)
      assert_true(text.contains("mismatched closing tag"))
      assert_true(text.contains("@3:"))
      assert_true(text.contains("Snippet:"))
    }
    @urdf.ParseUrdfResult::Ok(_) => fail("expected a parse error")
  }
}

///|
test "reports a missing robot name" {
  let xml = "<robot>\n  <link name=\"a\"/>\n</robot>\n"
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Err(info) => {
      assert_true(info.line is Some(_))
      assert_true(info.col is Some(_))
      assert_true(info.message.contains("missing required attribute \"name\""))
    }
    @urdf.ParseUrdfResult::Ok(_) => fail("expected a parse error")
  }
}

///|
test "reports an invalid float with line and column" {
  let xml = "<robot name=\"r\">\n" +
    "  <link name=\"a\">\n" +
    "    <visual>\n" +
    "      <origin xyz=\"1.0 foo 3.0\"/>\n" +
    "      <geometry><box size=\"1 1 1\"/></geometry>\n" +
    "    </visual>\n" +
    "  </link>\n" +
    "</robot>\n"
  match @urdf.parse_urdf_from_string_result(xml) {
    @urdf.ParseUrdfResult::Err(info) => {
      assert_true(info.line is Some(_))
      assert_true(info.col is Some(_))
      assert_true(info.message.contains("invalid float \"foo\""))
      assert_true(@urdf.format_urdf_parse_error(info).contains("@4:"))
    }
    @urdf.ParseUrdfResult::Ok(_) => fail("expected a parse error")
  }
}

///|
test "reports unterminated and truncated input" {
  let cases = [
    "<robot name=\"r\"><link name=\"a\">",
    "<robot name=\"r\"",
    "<robot name=\"r\"><!-- unclosed comment </robot>",
    "",
  ]
  for xml in cases {
    match @urdf.parse_urdf_from_string_result(xml) {
      @urdf.ParseUrdfResult::Err(info) => {
        let text = @urdf.format_urdf_parse_error(info)
        assert_true(text.length() > 0)
      }
      @urdf.ParseUrdfResult::Ok(_) => fail("expected a parse error for: " + xml)
    }
  }
}

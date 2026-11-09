///|
test "bundle package.xml content" {
  let robot = @urdf.Robot::new("bundle_bot")
  let bundle = @urdf.UrdfBundle::new("galileo_quadruped", robot).add_mesh_text(
    "meshes/a.gltf", "{}\n",
  )
  assert_eq(bundle.meshes.length(), 1)
  let xml = bundle.package_xml(
    version="2.0.0",
    description="A test package",
    license="MIT",
  )
  assert_true(xml.contains("<package format=\"3\">"))
  assert_true(xml.contains("<name>galileo_quadruped</name>"))
  assert_true(xml.contains("<version>2.0.0</version>"))
  assert_true(xml.contains("<description>A test package</description>"))
  assert_true(xml.contains("<license>MIT</license>"))
  assert_true(xml.contains("<build_type>ament_cmake</build_type>"))
}

///|
fn bundle_robot() -> @urdf.Robot {
  @urdf.Robot::new("bundle_bot")
  .add_link(
    @urdf.Link::new("base")
    .with_inertial(@urdf.Inertial::from_box(0.1, 0.1, 0.1, mass=1.0))
    .add_visual(
      @urdf.Visual::new(
        @urdf.Geometry::mesh("package://galileo_quadruped/meshes/part.gltf"),
      ),
    )
    .add_collision(@urdf.Collision::new(@urdf.Geometry::box(0.1, 0.1, 0.1))),
  )
  .add_link(
    @urdf.Link::new("arm").with_inertial(
      @urdf.Inertial::from_box(0.05, 0.05, 0.05, mass=0.2),
    ),
  )
  .add_joint(@urdf.Joint::fixed("j", parent="base", child="arm"))
}

///|
async test "writes a full bundle that passes the Python validator" {
  let dir = @fs.tmpdir(prefix="urdf-bundle-")
  defer @fs.rmdir(dir, recursive=true)
  let robot = bundle_robot()
  let bundle = @urdf.UrdfBundle::new("galileo_quadruped", robot).add_mesh_text(
    "meshes/part.gltf", "{}\n",
  )
  bundle.write_to_dir(dir, checked=true)
  let urdf_path = dir + "/urdf/" + robot.name + ".urdf"
  assert_true(@fs.exists(urdf_path))
  assert_true(@fs.exists(dir + "/package.xml"))
  assert_true(@fs.exists(dir + "/meshes/part.gltf"))
  let package_xml = @fs.read_file(dir + "/package.xml").text()
  assert_true(package_xml.contains("<name>galileo_quadruped</name>"))
  let urdf_text = @fs.read_file(urdf_path).text()
  assert_true(
    urdf_text.contains(
      "<mesh filename=\"package://galileo_quadruped/meshes/part.gltf\"/>",
    ),
  )
  let (code, output) = @process.collect_output_merged("python3", [
    "scripts/validate_urdf.py", "--check-meshes", urdf_path,
  ])
  let text = output.text()
  if code != 0 {
    fail("validator failed: " + text)
  }
}

///|
async test "bundle writes a parseable urdf" {
  let dir = @fs.tmpdir(prefix="urdf-bundle-parse-")
  defer @fs.rmdir(dir, recursive=true)
  let robot = bundle_robot()
  let bundle = @urdf.UrdfBundle::new("pkg", robot)
  bundle.write_to_dir(dir)
  let urdf_path = dir + "/urdf/" + robot.name + ".urdf"
  let text = @fs.read_file(urdf_path).text()
  match @urdf.parse_urdf_from_string_result(text) {
    @urdf.ParseUrdfResult::Ok(parsed) => assert_true(parsed == robot)
    @urdf.ParseUrdfResult::Err(info) =>
      fail("parse failed: " + @urdf.format_urdf_parse_error(info))
  }
}

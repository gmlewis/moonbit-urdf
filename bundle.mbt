///|
/// Multi-file ROS package bundle export.

///|
pub(all) struct MeshAsset {
  rel_path : String
  bytes : Bytes
}

///|
pub(all) struct UrdfBundle {
  package_name : String
  robot : Robot
  meshes : Array[MeshAsset]
}

///|
pub fn UrdfBundle::new(package_name : String, robot : Robot) -> UrdfBundle {
  { package_name, robot, meshes: [], }
}

///|
pub fn UrdfBundle::add_mesh_bytes(
  self : UrdfBundle,
  rel_path : String,
  bytes : Bytes,
) -> UrdfBundle {
  let meshes = self.meshes.copy()
  meshes.push({ rel_path, bytes, })
  { ..self, meshes, }
}

///|
pub fn UrdfBundle::add_mesh_text(
  self : UrdfBundle,
  rel_path : String,
  text : String,
) -> UrdfBundle {
  self.add_mesh_bytes(rel_path, @encoding/utf8.encode(text))
}

///|
/// Generates a standard ROS 2 `package.xml`.
pub fn UrdfBundle::package_xml(
  self : UrdfBundle,
  version~ : String = "0.1.0",
  description~ : String = "",
  maintainer~ : String = "gmlewis",
  email~ : String = "gmlewis@users.noreply.github.com",
  license~ : String = "Apache-2.0",
) -> String {
  let sb = StringBuilder()
  sb.write_string("<?xml version=\"1.0\"?>\n")
  sb.write_string("<package format=\"3\">\n")
  sb.write_string("  <name>" + xml_escape(self.package_name) + "</name>\n")
  sb.write_string("  <version>" + xml_escape(version) + "</version>\n")
  sb.write_string(
    "  <description>" + xml_escape(description) + "</description>\n",
  )
  sb.write_string(
    "  <maintainer email=\"" +
    xml_escape(email) +
    "\">" +
    xml_escape(maintainer) +
    "</maintainer>\n",
  )
  sb.write_string("  <license>" + xml_escape(license) + "</license>\n")
  sb.write_string("  <buildtool_depend>ament_cmake</buildtool_depend>\n")
  sb.write_string("  <export>\n")
  sb.write_string("    <build_type>ament_cmake</build_type>\n")
  sb.write_string("  </export>\n")
  sb.write_string("</package>\n")
  sb.to_string()
}

///|
fn dir_of(path : String) -> String {
  match path.rev_find("/") {
    Some(idx) if idx > 0 => String::unsafe_substring(path, start=0, end=idx)
    _ => ""
  }
}

///|
/// Writes `package.xml`, `urdf/<robot>.urdf`, and every mesh asset to
/// `out_dir`, creating parent directories as needed.
pub async fn UrdfBundle::write_to_dir(
  self : UrdfBundle,
  out_dir : String,
  checked~ : Bool = true,
) -> Unit {
  @fs.mkdir(out_dir, recursive=true, allow_exist=true)
  let urdf_dir = out_dir + "/urdf"
  @fs.mkdir(urdf_dir, recursive=true, allow_exist=true)
  let urdf_path = urdf_dir + "/" + self.robot.name + ".urdf"
  self.robot.write_urdf_file(Some(urdf_path), checked~)
  let package_path = out_dir + "/package.xml"
  write_text_file(self.package_xml(), Some(package_path))
  for mesh in self.meshes {
    let parent = dir_of(mesh.rel_path)
    if parent != "" {
      @fs.mkdir(out_dir + "/" + parent, recursive=true, allow_exist=true)
    }
    @fs.write_file(
      out_dir + "/" + mesh.rel_path,
      mesh.bytes,
      permission=0o644,
      create_mode=@fs.CreateMode::CreateOrTruncate,
    )
  }
}

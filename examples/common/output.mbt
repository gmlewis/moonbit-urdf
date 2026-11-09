///|
/// Shared output handling for the URDF examples.
pub struct OutputPaths {
  urdf : String?
  bundle_dir : String?
  unchecked : Bool
  nostdout : Bool
}

///|
/// Adds `--out`/`-o`, `--bundle`, `--unchecked`, and `--nostdout` to an
/// example's argument spec.
pub fn with_output_args(
  spec : Map[String, @cli.Arg],
  bundle~ : Bool = true,
  out_help~ : String = "Write URDF output to a file (default: stdout)",
) -> Map[String, @cli.Arg] {
  spec.set("out", @cli.opt_string(short='o', help=out_help))
  if bundle {
    spec.set(
      "bundle",
      @cli.opt_string(help="Write a full ROS package bundle to this directory"),
    )
  }
  spec.set(
    "unchecked",
    @cli.flag(help="Skip validation diagnostics when writing"),
  )
  spec.set("nostdout", @cli.flag(help="Do not write URDF to stdout"))
  spec
}

///|
/// Collects the standard output values from parsed arguments.
pub fn output_paths(args : @cli.Args) -> OutputPaths {
  {
    urdf: args.string_opt("out"),
    bundle_dir: args.string_opt("bundle"),
    unchecked: args.flag("unchecked"),
    nostdout: args.flag("nostdout"),
  }
}

///|
/// Dispatches to the bundle exporter and/or the URDF writer. `bundle` lets an
/// example supply mesh assets; otherwise a mesh-less bundle is created.
pub async fn write_output(
  robot : @urdf.Robot,
  paths : OutputPaths,
  bundle? : @urdf.UrdfBundle,
) -> Unit {
  match paths.bundle_dir {
    Some(dir) => {
      let bundle = match bundle {
        Some(b) => b
        None => @urdf.UrdfBundle::new(robot.name, robot)
      }
      bundle.write_to_dir(dir, checked=!paths.unchecked)
    }
    None => ()
  }
  match paths.urdf {
    Some(path) => robot.write_urdf(Some(path), checked=!paths.unchecked)
    None =>
      if !paths.nostdout && paths.bundle_dir is None {
        robot.write_urdf(None, checked=!paths.unchecked)
      }
  }
}

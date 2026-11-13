///|
/// # `gmlewis/urdf`
///
/// A standalone, dependency-clean URDF (Unified Robot Description Format)
/// library for MoonBit: a typed robot model, a fluent builder DSL, analytical
/// and triangle-mesh inertia math, mate-connector snapping, forward
/// kinematics, a lossless XML parser, deterministic serialization, validation
/// diagnostics, and multi-file ROS 2 package export.
///
/// `gmlewis/urdf` has **zero dependency on `gmlewis/step`**. It is the
/// robotics pillar that `gmlewis/cad` imports alongside `gmlewis/step`.
///
/// ## Robot model
///
/// | Type | Role |
/// | --- | --- |
/// | `Robot` | Name, version, documentation, materials, links, joints, transmissions, and preserved extra XML. |
/// | `Link` | Inertial, visuals, collisions, and preserved extra XML. |
/// | `Joint` | Type, origin, parent/child links, axis, limit, dynamics, calibration, safety controller, mimic, and extra XML. |
/// | `JointType` | `Revolute`, `Continuous`, `Prismatic`, `Fixed`, `Floating`, `Planar`. |
/// | `Geometry` | `Box((sx, sy, sz))`, `Cylinder((radius, length))`, `Sphere(radius)`, or `Mesh(path, scale)`. |
/// | `Visual` / `Collision` | A geometry plus an origin, name, and (for visuals) a material. |
/// | `Inertial` | Origin, mass in kg, and an `Inertia` tensor in `kg·m²`. |
/// | `Transmission` / `Actuator` / `TransmissionJoint` | ROS control transmissions. |
///
/// ## Units and conventions
///
/// - The model is stored in **SI**: lengths in meters, angles in radians, mass
///   in kilograms, and inertia in `kg·m²`.
/// - Convenience constructors take CAD-friendly units:
///   `Geometry::box_mm`, `Geometry::cylinder_mm`, `Geometry::sphere_mm`,
///   `Origin::from_mm_deg`, `MateConnector::from_mm_deg`, and
///   `Inertial::from_triangles_mm`.
/// - `Origin.rpy` is fixed-axis roll/pitch/yaw (`R = Rz * Ry * Rx`).
/// - Geometry is centered on its own frame, matching URDF and CAD conventions.
/// - The unit types `Length`, `Mass`, and `Angle` carry their own units and
///   convert explicitly (`Length::mm(25.0).to_m()`, `Angle::deg(90.0).to_rad()`).
/// - `Density` presets cover aluminum 6061, mild steel, titanium, carbon
///   fiber, PLA, PETG, TPU, and ABS.
///
/// ## Fluent builders
///
/// ```mbt
/// let robot = @urdf.Robot::new("demo")
///   .add_link(
///     @urdf.Link::new("base").with_inertial(
///       @urdf.Inertial::from_box_density(
///         0.2, 0.1, 0.05, density=@urdf.Density::aluminum_6061(),
///       ),
///     ).add_visual(
///       @urdf.Visual::new(@urdf.Geometry::box(0.2, 0.1, 0.05))
///         .with_color("body", @urdf.Rgba::orange()),
///     ),
///   )
///   .add_link(@urdf.Link::new("arm"))
///   .add_joint(
///     @urdf.Joint::revolute(
///       "shoulder",
///       parent="base",
///       child="arm",
///       axis=(0.0, 1.0, 0.0),
///       limit=@urdf.JointLimit::new(
///         lower=Some(-1.5), upper=Some(1.5), effort=10.0, velocity=2.0,
///       ),
///     ),
///   )
///
/// robot.write_urdf(Some("demo.urdf"), checked=true)
/// ```
///
/// `builders.mbt` adds higher-level constructors on top of the raw types, while
/// every `Robot`, `Link`, and `Joint` also supports method-chaining `with_*`
/// and `add_*` helpers.
///
/// ## Mate connectors and forward kinematics
///
/// A `MateConnector` is a named frame plus an axis. It is the canonical type
/// from `gmlewis/step/components/common`: the frame is stored in millimetres
/// and degrees and converted with `from_m_rad` / `to_m_rad` at the URDF
/// boundary. `Joint::from_mates()` snaps two connectors together with
/// `snap_mates()`, and
/// `Robot::forward_kinematics(joint_positions)` returns the world `Transform`
/// of every link. `Robot::link_world_origins()` returns the same data as
/// `Origin`s, and `Robot::root_link()` finds the kinematic root.
///
/// ```mbt
/// let hip = @urdf.MateConnector::from_mm_deg("hip", x=100.0, axis=(1.0, 0.0, 0.0))
/// let leg = @urdf.MateConnector::new("leg_mount")
/// let joint = @urdf.Joint::from_mates(
///   "hip_joint", @urdf.JointType::Revolute,
///   parent_link="base", parent_mate=hip,
///   child_link="leg", child_mate=leg,
///   limit=Some(@urdf.JointLimit::new(effort=20.0, velocity=5.0)),
/// )
/// let poses = robot.forward_kinematics(joint_positions={ "hip_joint": 0.5 })
/// ```
///
/// ## Inertia
///
/// `Inertial` provides analytical constructors for boxes, cylinders, tubes, and
/// spheres (with mass or density), plus `Inertial::from_triangles` and
/// `Inertial::from_triangles_mm` for triangle meshes. `Inertial::combine`
/// merges a list of inertials; `Inertial::transform_by` and
/// `Inertia::shift_by_offset` move them between frames.
///
/// ```mbt
/// // triangles is Array[(Vec3, Vec3, Vec3)] in millimeters.
/// let inertial = @urdf.Inertial::from_triangles_mm(
///   triangles,
///   density=@urdf.Density::pla(),
/// )
/// ```
///
/// ## Parsing and serialization
///
/// ```mbt
/// match @urdf.parse_urdf_from_string_result(xml) {
///   @urdf.ParseUrdfResult::Ok(robot) => { /* ... */ }
///   @urdf.ParseUrdfResult::Err(info) =>
///     println(@urdf.format_urdf_parse_error(info))
/// }
/// ```
///
/// `parse_urdf_from_string` raises `UrdfParseError`; the `_result` variant
/// returns it. `parse_urdf_from_file` reads from disk, and `parse_xml_node`
/// parses any XML subtree. Unknown elements such as `<gazebo>` and
/// `<ros2_control>` are preserved in `Robot.extra`, `Link.extra`, and
/// `Joint.extra` as `RawXmlNode`s and survive serialization.
///
/// `Robot::to_urdf_xml()` and `Robot::to_urdf_file_checked()` produce
/// deterministic XML; `Robot::write_urdf(Some(path), checked=true)` refuses to
/// write a model with error diagnostics.
///
/// ## Diagnostics
///
/// `Robot::diagnostics()` reports topological errors (missing links, multiple
/// parents, cycles, disconnected graphs) and physical warnings (zero joint
/// axes, missing limits, non-positive mass, non-physical inertia). Each
/// `Diagnostic` carries a `Severity` (`Info`, `Warning`, `Error`), and
/// `Robot::has_errors()` reports whether checked export would refuse the model.
///
/// ## ROS 2 bundles
///
/// `UrdfBundle::new(package_name, robot)` collects mesh assets
/// (`add_mesh_text`, `add_mesh_bytes`), generates `package.xml` with
/// `package_xml(...)`, and writes the complete package with
/// `write_to_dir(path, checked=true)`.
///
/// ## Module map
///
/// | File | Contents |
/// | --- | --- |
/// | `types.mbt` | `Robot`, `Link`, `Joint`, `Geometry`, `Visual`, `Collision`, `Material`, `Rgba`, `Origin`, and the core enums. |
/// | `units.mbt` | `Length`, `Mass`, `Angle`, and `Density` unit types. |
/// | `transform.mbt` | `Matrix3x3`, `Transform`, rotation helpers, `snap_mates`, and vector math. |
/// | `inertia.mbt` | `Inertia`, `Inertial`, analytical and triangle-mesh inertia, and combination. |
/// | `mate.mbt` | `MateConnector` and mate snapping. |
/// | `parse.mbt` | Lossless XML/URDF parsing. |
/// | `diagnostics.mbt` | `Diagnostic`, `Severity`, and validation helpers. |
/// | `bundle.mbt` | `UrdfBundle` and ROS 2 package export. |
/// | `builders.mbt` | Higher-level fluent constructors. |
/// | `extends.mbt` | Explicit `Eq`/`Debug` trait-method promotions. |
///
/// ## Tooling
///
/// ```bash
/// ./test-all.sh                                   # moon fmt, moon info, moon test
/// python3 scripts/validate_urdf.py robot.urdf     # structural/physics checks
/// python3 scripts/validate_urdf.py --check-meshes robot.urdf
/// ```

///|
// Explicit trait-method promotion declarations.
//
// MoonBit no longer promotes a trait's methods to direct methods on a type
// implicitly: every promoted method needs an explicit `extend` declaration,
// otherwise moonc warns (implicit_impl_as_method). The primary method of each
// trait is promoted plainly. The defaulted derivative that the `!=` operator
// already covers (`Eq::not_equal`) is promoted as well - moonc otherwise warns
// about its implicit promotion - but deprecated and hidden, since there is no
// reason to call it directly.

///|
pub extend Actuator with Debug::{to_repr}

///|
pub extend Actuator with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Actuator with Eq::{not_equal}

///|
pub extend Collision with Debug::{to_repr}

///|
pub extend Collision with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Collision with Eq::{not_equal}

///|
pub extend Geometry with Debug::{to_repr}

///|
pub extend Geometry with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Geometry with Eq::{not_equal}

///|
pub extend Inertia with Debug::{to_repr}

///|
pub extend Inertia with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Inertia with Eq::{not_equal}

///|
pub extend Inertial with Debug::{to_repr}

///|
pub extend Inertial with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Inertial with Eq::{not_equal}

///|
pub extend Joint with Debug::{to_repr}

///|
pub extend Joint with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Joint with Eq::{not_equal}

///|
pub extend JointCalibration with Debug::{to_repr}

///|
pub extend JointCalibration with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointCalibration with Eq::{not_equal}

///|
pub extend JointDynamics with Debug::{to_repr}

///|
pub extend JointDynamics with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointDynamics with Eq::{not_equal}

///|
pub extend JointLimit with Debug::{to_repr}

///|
pub extend JointLimit with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointLimit with Eq::{not_equal}

///|
pub extend JointMimic with Debug::{to_repr}

///|
pub extend JointMimic with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointMimic with Eq::{not_equal}

///|
pub extend JointType with Debug::{to_repr}

///|
pub extend JointType with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend JointType with Eq::{not_equal}

///|
pub extend Link with Debug::{to_repr}

///|
pub extend Link with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Link with Eq::{not_equal}

///|
pub extend Material with Debug::{to_repr}

///|
pub extend Material with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Material with Eq::{not_equal}

///|
pub extend Origin with Debug::{to_repr}

///|
pub extend Origin with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Origin with Eq::{not_equal}

///|
pub extend RawXmlNode with Debug::{to_repr}

///|
pub extend RawXmlNode with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend RawXmlNode with Eq::{not_equal}

///|
pub extend Rgba with Debug::{to_repr}

///|
pub extend Rgba with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Rgba with Eq::{not_equal}

///|
pub extend Robot with Debug::{to_repr}

///|
pub extend Robot with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Robot with Eq::{not_equal}

///|
pub extend SafetyController with Debug::{to_repr}

///|
pub extend SafetyController with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend SafetyController with Eq::{not_equal}

///|
pub extend Transmission with Debug::{to_repr}

///|
pub extend Transmission with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Transmission with Eq::{not_equal}

///|
pub extend TransmissionJoint with Debug::{to_repr}

///|
pub extend TransmissionJoint with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend TransmissionJoint with Eq::{not_equal}

///|
pub extend Visual with Debug::{to_repr}

///|
pub extend Visual with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Visual with Eq::{not_equal}

///|
pub extend Angle with Add::{add}

///|
pub extend Angle with Sub::{sub}

///|
pub extend Angle with Neg::{neg}

///|
pub extend Length with Add::{add}

///|
pub extend Length with Sub::{sub}

///|
pub extend Length with Neg::{neg}

///|
pub extend Mass with Add::{add}

///|
pub extend Mass with Sub::{sub}

///|
pub extend UrdfParseErrorInfo with Debug::{to_repr}

///|
pub extend UrdfParseErrorInfo with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend UrdfParseErrorInfo with Eq::{not_equal}

///|
pub extend Diagnostic with Debug::{to_repr}

///|
pub extend Diagnostic with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Diagnostic with Eq::{not_equal}

///|
pub extend Severity with Debug::{to_repr}

///|
pub extend Severity with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Severity with Eq::{not_equal}

///|
pub extend Matrix3x3 with Debug::{to_repr}

///|
pub extend Matrix3x3 with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Matrix3x3 with Eq::{not_equal}

///|
pub extend Transform with Debug::{to_repr}

///|
pub extend Transform with Eq::{equal}

///|
#deprecated("Use `!=` instead", skip_current_package=true)
#doc(hidden)
pub extend Transform with Eq::{not_equal}

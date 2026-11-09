# gmlewis/urdf (MoonBit)

A standalone, dependency-clean URDF (Unified Robot Description Format) library
for MoonBit: a typed robot model, a fluent builder DSL, analytical and
triangle-mesh inertia math, mate-connector snapping, forward kinematics, a
lossless XML parser, deterministic serialization, validation diagnostics, and
multi-file ROS package export.

`gmlewis/urdf` has **zero dependency on `gmlewis/step`**. It is the robotics
pillar that the upcoming `gmlewis/cad` package imports alongside `gmlewis/step`.

## Quick start

```moonbit
let robot = @urdf.Robot::new("demo")
  .add_link(
    @urdf.Link::new("base").with_inertial(
      @urdf.Inertial::from_box_density(
        0.2, 0.1, 0.05, density=@urdf.Density::aluminum_6061(),
      ),
    ).add_visual(
      @urdf.Visual::new(@urdf.Geometry::box(0.2, 0.1, 0.05))
        .with_color("body", @urdf.Rgba::orange()),
    ),
  )
  .add_link(@urdf.Link::new("arm"))
  .add_joint(
    @urdf.Joint::revolute(
      "shoulder",
      parent="base",
      child="arm",
      axis=(0.0, 1.0, 0.0),
      limit=@urdf.JointLimit::new(
        lower=Some(-1.5), upper=Some(1.5), effort=10.0, velocity=2.0,
      ),
    ),
  )

robot.write_urdf(Some("demo.urdf"), checked=true)
```

### Mate connectors and kinematics

```moonbit
let hip = @urdf.MateConnector::from_mm_deg("hip", x=100.0, axis=(1.0, 0.0, 0.0))
let leg = @urdf.MateConnector::new("leg_mount")
let joint = @urdf.Joint::from_mates(
  "hip_joint", @urdf.JointType::Revolute,
  parent_link="base", parent_mate=hip,
  child_link="leg", child_mate=leg,
  limit=Some(@urdf.JointLimit::new(effort=20.0, velocity=5.0)),
)

let poses = robot.forward_kinematics(
  joint_positions={ "hip_joint": 0.5 },
)
```

### Inertia from CAD meshes

```moonbit
// triangles is Array[(Vec3, Vec3, Vec3)] in millimeters.
let inertial = @urdf.Inertial::from_triangles_mm(
  triangles,
  density=@urdf.Density::pla(),
)
```

### Lossless parsing

```moonbit
match @urdf.parse_urdf_from_string_result(xml) {
  @urdf.ParseUrdfResult::Ok(robot) => { /* ... */ }
  @urdf.ParseUrdfResult::Err(info) =>
    println(@urdf.format_urdf_parse_error(info))
}
```

Unknown elements such as `<gazebo>` and `<ros2_control>` are preserved in
`Robot.extra`, `Link.extra`, and `Joint.extra` and survive serialization.

## Pairing with gmlewis/step

`gmlewis/cad` composes parameterized mechanical components and emits STEP,
glTF, and URDF from one model. `gmlewis/step` owns B-Rep/STEP, `gmlewis/urdf`
owns links, joints, inertial properties, mate snapping, forward kinematics, and
ROS package bundles. Neither depends on the other; `gmlewis/cad` imports both.

## Examples

| Example | Description |
| --- | --- |
| `01-single-link-box` | Parameterized box: millimeter units, density, analytical inertia |
| `02-two-link-pendulum` | Revolute pendulum assembled with mate connectors |
| `03-planetary-gearbox-urdf` | Sun/ring/planets/carrier with mimic coupling |
| `04-six-dof-arm` | Six-axis arm with `Inertial::combine`, safety controllers, transmissions |
| `05-wheeled-quadruped` | Galileo-style quadruped: 17 links, 16 joints, full mesh bundle |
| `06-assistive-scooter` | Rider-parameterized assistive scooter/wheelchair |

Run any example (numbers resolve to the matching folder):

```bash
./run-example.sh 01 -o box.urdf
./run-example.sh 02 --length1 200 -o pendulum.urdf --validate
./run-example.sh 05 -o galileo.urdf
./run-example.sh 05 --bundle /tmp/galileo_pkg
python3 scripts/validate_urdf.py --check-meshes \
  /tmp/galileo_pkg/urdf/galileo_quadruped.urdf
```

## Validation

```bash
./test-all.sh                                   # moon fmt, moon info, moon test
python3 scripts/validate_urdf_test.py           # validator self-tests
python3 scripts/validate_urdf.py robot.urdf     # structural/physics checks
```

`Robot::diagnostics()` reports topological errors (missing links, multiple
parents, cycles, disconnected roots) and physical warnings (non-positive mass,
non-physical inertia), while `Robot::write_urdf(..., checked=true)` refuses to
write an invalid model.

## License

Apache-2.0

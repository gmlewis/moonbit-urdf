# Project Agents.md Guide

This is a [MoonBit](https://docs.moonbitlang.com) project: `gmlewis/urdf`, a
typed URDF model, parser/serializer, inertia/kinematics library, and ROS package
bundle exporter.

## Project Structure

- MoonBit packages are organized per directory. Each directory has a `moon.pkg`
  file listing its dependencies, blackbox tests ending in `_test.mbt`, and
  whitebox tests ending in `_wbtest.mbt`.
- The toplevel `moon.mod` lists the module metadata and dependencies.
- Keep the root workspace as the only `moon.mod` unless a separate module is
  genuinely required by a large external dependency.

## Architecture

- `types.mbt` / `builders.mbt`: immutable typed URDF model and fluent builders.
  Every builder returns a new value; never mutate shared state.
- Units: builder APIs that take dimensions use metres for URDF values; helpers
  with `_mm` / `from_mm_deg` names accept millimetres and degrees. Preserve that
  boundary.
- `mate.mbt`: mate-connector snapping and joint construction. The canonical
  `MateConnector` / `MatePair` types live in
  `gmlewis/step/components/common`; this package aliases them. Joint axes follow
  the shared `MatePair::resolve` anti-parallel convention.
- `xml.mbt` / `parse.mbt`: deterministic serialization and lossless parsing.
  Unknown elements are preserved in `extra` fields and must survive a
  round-trip.
- `inertia.mbt`: analytical inertia, triangle-mesh inertia, Parallel Axis
  Theorem, and `Inertial::combine`.
- `diagnostics.mbt`: topological and physical validation. Do not weaken checks
  to make a fixture pass; fix the model or the diagnostic.
- `bundle.mbt`: multi-file ROS package export (`package.xml`, URDF, mesh
  assets). Mesh assets are supplied as bytes or text by the caller.

## Coding convention

- All source code and markdown files should end in a newline.
- MoonBit code is organized in block style; each block is separated by `///|`.
  Block order is irrelevant.
- Prefer functional-style programming:
  - Use `map`, `filter`, `fold`, etc., over manual `for i = 0...` loops.
  - For simple iteration, use `for x in array { ... }` or `for i in 0..<n { ... }`.
  - Use explicit parentheses in ranges so `moon fmt` cannot change the logic:
    `0..<(n - 1)`.
- Keep deprecated blocks in a file named `deprecated.mbt` in each directory.
- Keep filenames relatively short and consistent; 30+ characters is long.
- Keep `using` statements lexicographically sorted.
- Keep imports in `moon.pkg` and dependencies in `moon.mod` sorted
  lexicographically.

## Tooling & Verification

- `moon fmt` formats the code.
- `./test-all.sh` runs `moon fmt`, `moon info`, and the unit tests. Run it
  frequently.
- `python3 scripts/validate_urdf_test.py` runs the validator self-tests.
- `python3 scripts/validate_urdf.py <file.urdf>` checks XML syntax, links,
  joints, topology, physics, and optionally mesh references
  (`--check-meshes --package-root <dir>`).
- Do not run `./update.sh`; it is a maintainer-only dependency update script.
- Do not add generated URDF/STEP/mesh artifacts to the repository; write
  temporary validation output to `/tmp`.

## Error Handling

- **CLI/Main**: Never use `println` for errors. Use `@cli.eprintln("msg")`
  followed by `abort("")` so the process exits non-zero and short-circuits shell
  chains.
- **Library**: Use `Result` or `raise` for internal logic errors.

#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
run-example.sh — run a URDF example and optionally write/validate its output.

Usage:
  ./run-example.sh <example> [options...]
  ./run-example.sh <number>  [options...]     e.g. 01, 1, 05

Wrapper options:
  -h, --help             Show this help and exit.
  -o, --out <path>       Write the URDF file to <path> (default: stdout).
  --bundle <dir>         Write a full ROS package bundle to <dir>.
  --validate             Run scripts/validate_urdf.py on the generated URDF.
                         Implies an output file (a temporary one if -o is absent).
  --unchecked            Skip validation diagnostics in the example.
  --                     Optional separator (ignored).

Every other option is forwarded to the example (for example --sx, --radius,
--wheelRadius). Run `./run-example.sh <number> -- --help` to see the
example-specific options.

Examples:
  ./run-example.sh 01 -o box.urdf
  ./run-example.sh 02 --length1 200 -o pendulum.urdf --validate
  ./run-example.sh 05 -o galileo.urdf
  ./run-example.sh 05 --bundle /tmp/galileo_pkg
  python3 scripts/validate_urdf.py --check-meshes /tmp/galileo_pkg/urdf/galileo_quadruped.urdf
EOF
}

if [[ $# -gt 0 ]]; then
  case "$1" in
    -h | --help | help)
      usage
      exit 0
      ;;
  esac
fi

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

EXAMPLE_ARG="$1"
shift

if [[ $# -gt 0 && "${1:-}" == "--" ]]; then
  shift
fi

EXAMPLE_ARG_STRIPPED="${EXAMPLE_ARG%/}"
EXAMPLE_BASENAME="$(basename "$EXAMPLE_ARG_STRIPPED")"

# Resolve symlinks so the repository root is found no matter how we are called.
SOURCE="${BASH_SOURCE[0]}"
while [ -L "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ "$EXAMPLE_ARG_STRIPPED" == *"/"* ]]; then
  EXAMPLE_PATH_REL="$EXAMPLE_ARG_STRIPPED"
elif [[ "$EXAMPLE_BASENAME" =~ ^[0-9]+$ ]]; then
  SEARCH_NUM=$(printf "%02d" $((10#$EXAMPLE_BASENAME)))
  MATCHES=("$ROOT_DIR"/examples/"$SEARCH_NUM"-*)
  if [[ -d "${MATCHES[0]}" ]]; then
    EXAMPLE_PATH_REL="examples/$(basename "${MATCHES[0]}")"
  else
    EXAMPLE_PATH_REL="examples/$EXAMPLE_BASENAME"
  fi
else
  EXAMPLE_PATH_REL="examples/$EXAMPLE_BASENAME"
fi

if [[ ! -d "$ROOT_DIR/$EXAMPLE_PATH_REL" ]]; then
  echo "Error: example folder not found: $EXAMPLE_ARG" >&2
  echo "Looked in: $ROOT_DIR/$EXAMPLE_PATH_REL" >&2
  exit 1
fi

EXAMPLE_NAME="$(basename "$EXAMPLE_PATH_REL")"

FORWARD_ARGS=()
OUT_PATH=""
VALIDATE=0
TMP_DIR=""
i=0
ARGS=("$@")
while (( i < ${#ARGS[@]} )); do
  arg="${ARGS[$i]}"
  case "$arg" in
    --validate)
      VALIDATE=1
      ;;
    -o | --out)
      FORWARD_ARGS+=("$arg")
      if (( i + 1 < ${#ARGS[@]} )); then
        OUT_PATH="${ARGS[$((i + 1))]}"
        FORWARD_ARGS+=("$OUT_PATH")
        i=$((i + 1))
      fi
      ;;
    -o=* | --out=*)
      OUT_PATH="${arg#*=}"
      FORWARD_ARGS+=("$arg")
      ;;
    *)
      FORWARD_ARGS+=("$arg")
      ;;
  esac
  i=$((i + 1))
done

if (( VALIDATE )) && [[ -z "$OUT_PATH" ]]; then
  TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/urdf-example.XXXXXX")"
  trap 'rm -rf "$TMP_DIR"' EXIT
  OUT_PATH="$TMP_DIR/$EXAMPLE_NAME.urdf"
  FORWARD_ARGS+=(-o "$OUT_PATH")
fi

MOONBIT_URDF_ROOT="$ROOT_DIR" \
  moon run --quiet --target native "$ROOT_DIR/$EXAMPLE_PATH_REL" -- "${FORWARD_ARGS[@]}"

if (( VALIDATE )); then
  python3 "$ROOT_DIR/scripts/validate_urdf.py" "$OUT_PATH"
fi

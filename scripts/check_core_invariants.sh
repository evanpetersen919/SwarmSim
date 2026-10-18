#!/usr/bin/env bash
# Checks the architectural rules that the compiler cannot. Fails if:
#   - core/ reads wall-clock time or unseeded randomness (a deliberate exception needs
#     "allow-wallclock: <reason>" on the same line),
#   - a build file enables -ffast-math,
#   - core/format (compiled into the Unreal plugin) includes Eigen or yaml-cpp,
#   - estimator or controller sources include anything outside their allowed lists, which keeps
#     ground truth out of them: estimators may use only estimation headers, sensor output types,
#     vector types, and the byte helpers; controllers may also use control headers, the hash
#     helper, and the command type.
#
# usage: check_core_invariants.sh [repository root]     (default: the repository this lives in)
set -u
cd "${1:-$(dirname "$0")/..}" || exit 2

status=0

forbidden='system_clock|steady_clock|high_resolution_clock|random_device|gettimeofday|clock_gettime|std::rand|[^a-zA-Z_:]rand\(|[^a-zA-Z_:]srand\(|[^a-zA-Z_:.>]time\((NULL|nullptr|0)'
hits=$(grep -rEn --include='*.cpp' --include='*.hpp' --include='*.h' "$forbidden" core/ 2>/dev/null \
  | grep -v 'allow-wallclock')
if [ -n "$hits" ]; then
  echo "Wall-clock or unseeded randomness found in core/:"
  echo "$hits"
  status=1
fi

fast=$(grep -rEn 'ffast-math|Ofast|/fp:fast' CMakeLists.txt cmake core net apps tests 2>/dev/null \
  | grep -v 'check_core_invariants')
if [ -n "$fast" ]; then
  echo "Fast-math flags found:"
  echo "$fast"
  status=1
fi

# core/format is compiled into the Unreal plugin, which must not see Eigen or yaml-cpp.
third_party=$(grep -rEn --include='*.cpp' --include='*.hpp' --include='*.h' \
  '#include *[<"](Eigen/|yaml-cpp/)' core/format 2>/dev/null)
if [ -n "$third_party" ]; then
  echo "Eigen or yaml-cpp included from core/format:"
  echo "$third_party"
  status=1
fi

# Estimators and controllers must not read ground truth. Their project includes are limited to
# these lists, so dynamics, scene, simulation, and the sensor models themselves stay out of reach.
check_boundary() {
  local label=$1 allowed=$2
  shift 2
  local leaks
  leaks=$(grep -rEn --include='*.cpp' --include='*.hpp' --include='*.h' '#include *"swarm/' "$@" \
    2>/dev/null | grep -vE "#include *\"$allowed")
  if [ -n "$leaks" ]; then
    echo "$label sources include headers outside the allowed list (ground truth must stay out):"
    echo "$leaks"
    status=1
  fi
}

check_boundary Estimator \
  'swarm/(estimation/|sensor_data\.hpp|vec3\.hpp|format/byte_io\.hpp)' \
  core/sim/include/swarm/estimation core/sim/src/estimation
check_boundary Controller \
  'swarm/(control/|estimation/|sensor_data\.hpp|vec3\.hpp|drone_command\.hpp|format/(byte_io|hash)\.hpp)' \
  core/sim/include/swarm/control core/sim/src/control

[ "$status" -eq 0 ] && echo "core invariants: ok"
exit "$status"

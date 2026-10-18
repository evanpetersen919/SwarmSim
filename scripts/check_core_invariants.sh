#!/usr/bin/env bash
# Checks the architectural rules that the compiler cannot. Fails if:
#   - core/ reads wall-clock time or unseeded randomness (a deliberate exception needs
#     "allow-wallclock: <reason>" on the same line),
#   - a build file enables -ffast-math,
#   - core/format (compiled into the Unreal plugin) includes Eigen or yaml-cpp,
#   - estimator sources include anything outside the allowed list, which keeps ground truth out of
#     them: they may use only estimation headers, sensor output types, vector types, and the byte
#     helpers.
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

# Estimators must not read ground truth. Their project includes are limited to this list, so
# dynamics, scene, simulation, and the sensor models themselves stay out of reach.
estimator_dirs="core/sim/include/swarm/estimation core/sim/src/estimation"
allowed='swarm/(estimation/|sensor_data\.hpp|vec3\.hpp|format/byte_io\.hpp)'
leaks=$(grep -rEn --include='*.cpp' --include='*.hpp' --include='*.h' '#include *"swarm/' \
  $estimator_dirs 2>/dev/null | grep -vE "#include *\"$allowed")
if [ -n "$leaks" ]; then
  echo "Estimator sources include headers outside the allowed list (ground truth must stay out):"
  echo "$leaks"
  status=1
fi

[ "$status" -eq 0 ] && echo "core invariants: ok"
exit "$status"

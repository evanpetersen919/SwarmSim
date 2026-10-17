#!/usr/bin/env bash
# Fails if core/ reads wall-clock time or non-seeded randomness, or if a build file enables
# -ffast-math. A deliberate exception needs "allow-wallclock: <reason>" on the same line.
set -u
cd "$(dirname "$0")/.."

status=0

forbidden='system_clock|steady_clock|high_resolution_clock|random_device|gettimeofday|clock_gettime|std::rand|[^a-zA-Z_:]rand\(|[^a-zA-Z_:]srand\(|[^a-zA-Z_:.>]time\((NULL|nullptr|0)'
hits=$(grep -rEn --include='*.cpp' --include='*.hpp' --include='*.h' "$forbidden" core/ \
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

[ "$status" -eq 0 ] && echo "core invariants: ok"
exit "$status"

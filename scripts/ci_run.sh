#!/usr/bin/env bash
# Runs a command for CI. If it fails, the compiler and test diagnostics from its output are
# repeated as GitHub annotations, so the reason is visible on the run page and through the API
# without downloading logs.
#
# usage: ci_run.sh <command> [arguments...]
set -u

log=$(mktemp)
"$@" 2>&1 | tee "$log"
status=${PIPESTATUS[0]}

if [ "$status" -ne 0 ]; then
  grep -E 'error|warning:|Failed|FAILED|undefined reference' "$log" | head -n 15 | cut -c1-700 \
    | while IFS= read -r line; do
        # Workflow commands must stay on one line, and '%' starts an escape sequence.
        printf '::error::%s\n' "${line//%/%25}"
      done
fi

rm -f "$log"
exit "$status"

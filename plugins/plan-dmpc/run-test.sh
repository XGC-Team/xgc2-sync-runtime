#!/bin/bash
# Explicit prefixes only. Defaults live in CMakeLists.txt as standard
# install locations. This script does not infer a work-tree parent.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
core_prefix=""
acados_prefix=""
build_dir=""
formation_root=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --core-prefix) core_prefix="${2-}"; shift 2 ;;
    --acados-prefix) acados_prefix="${2-}"; shift 2 ;;
    --build-dir) build_dir="${2-}"; shift 2 ;;
    --formation-generator-root) formation_root="${2-}"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$core_prefix" || -z "$acados_prefix" || -z "$build_dir" || -z "$formation_root" ]]; then
  echo "usage: run-test.sh --core-prefix DIR --acados-prefix DIR --build-dir DIR --formation-generator-root DIR" >&2
  exit 2
fi
cmake -S "$here" -B "$build_dir" \
  -DPLAN_DMPC_CORE_PREFIX="$core_prefix" \
  -DPLAN_DMPC_ACADOS_PREFIX="$acados_prefix"
cmake --build "$build_dir" -j 2
export LD_LIBRARY_PATH="${acados_prefix}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$build_dir/test_plan_dmpc" "$formation_root"

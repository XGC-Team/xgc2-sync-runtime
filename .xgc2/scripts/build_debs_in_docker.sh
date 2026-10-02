#!/usr/bin/env bash
# One all-architecture Deb, then actual installed SDK checks on each native CPU.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
image=ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-dev:1.0.0
output=""
work=""
deb=""
architecture=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) output="$2"; shift 2;;
    --work-dir) work="$2"; shift 2;;
    --deb) deb="$2"; shift 2;;
    --architecture) architecture="$2"; shift 2;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
test -z "${XGC2_DEPENDENCY_SET_DIGEST:-}" || [[ "$XGC2_DEPENDENCY_SET_DIGEST" =~ ^[0-9a-f]{64}$ ]]
test -z "${XGC2_APT_OVERLAY_URL:-}" || [[ "${XGC2_DEPENDENCY_SET_DIGEST:-}" =~ ^[0-9a-f]{64}$ ]]
docker pull "$image"
if [[ -n "$deb" ]]; then
  test -f "$deb"
  case "$architecture" in amd64|arm64) ;; *) echo 'native check requires --architecture amd64|arm64' >&2; exit 2;; esac
  # Root only installs inside this disposable container. No writable checkout
  # or host-prefix mount; compiler outputs live in the container tmpfs.
  docker run --rm --cpus 1 --memory 2g --tmpfs /workspace/check:rw,exec \
    -e EXPECTED_ARCHITECTURE="$architecture" \
    -v "$root:/workspace/source:ro" -v "$(realpath "$deb"):/workspace/sdk.deb:ro" \
    "$image" bash -lc '
      set -euo pipefail
      test "$(dpkg --print-architecture)" = "$EXPECTED_ARCHITECTURE"
      for tool in cmake cc c++ dpkg-deb python3; do command -v "$tool" >/dev/null; done
      dpkg -i /workspace/sdk.deb
      python3 /workspace/source/.xgc2/scripts/check_sdk_deb.py \
        --deb /workspace/sdk.deb --work-dir /workspace/check/result
    '
else
  test -n "$output" && test -n "$work"
  mkdir -p "$output" "$work"
  output="$(cd "$output" && pwd -P)"
  work="$(cd "$work" && pwd -P)"
  docker run --rm --cpus 1 --memory 2g \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e SOURCE_DATE_EPOCH="$(git -C "$root" log -1 --format=%ct)" \
    -v "$root:/workspace/source:ro" -v "$output:/workspace/out" -v "$work:/workspace/work" \
    "$image" bash -lc '
      set -euo pipefail
      for tool in cmake dpkg-deb git python3; do command -v "$tool" >/dev/null; done
      /workspace/source/.xgc2/scripts/build_deb.sh --output /workspace/out
      mapfile -t debs < <(find /workspace/out -maxdepth 1 -type f -name "*.deb")
      test "${#debs[@]}" -eq 1
      before="$(sha256sum "${debs[0]}")"
      if /workspace/source/.xgc2/scripts/build_deb.sh --output /workspace/out; then
        echo "owning builder unexpectedly overwrote existing SDK artifact" >&2
        exit 1
      fi
      test "$(sha256sum "${debs[0]}")" = "$before"
    '
fi

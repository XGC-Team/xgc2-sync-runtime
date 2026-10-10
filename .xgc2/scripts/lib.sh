# Shared by the package scripts: read product metadata and set up a reproducible build.

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"

# apt_version DISTRIBUTION: the Debian version declared for the distribution in product.yml.
apt_version() {
  local distribution="$1" version base
  version="$(awk -v key="$distribution:" '$1 == key { print $2; exit }' "$root/.xgc2/product.yml")"
  base="$(awk '/^version:/ { print $2; exit }' "$root/.xgc2/product.yml")"
  [[ -n "$version" && -n "$base" && "$version" == "$base~$distribution" ]] || {
    echo "product.yml: version and apt_versions.$distribution are missing or inconsistent" >&2
    return 1
  }
  dpkg --validate-version "$version"
  printf '%s\n' "$version"
}

# reproducible DIRECTORY: fix the timestamps of a package tree.
reproducible() {
  local epoch="${SOURCE_DATE_EPOCH:-$(git -C "$root" log -1 --format=%ct)}"
  [[ "$epoch" =~ ^[0-9]+$ ]] || { echo 'invalid SOURCE_DATE_EPOCH' >&2; return 1; }
  export SOURCE_DATE_EPOCH="$epoch"
  find "$1" -exec touch -h -d "@$epoch" {} +
}

# install_deb TREE DEB: build the package and refuse to overwrite an existing artifact.
install_deb() {
  local tree="$1" deb="$2" temporary
  [[ ! -e "$deb" && ! -L "$deb" ]] || { echo "refusing existing artifact: $deb" >&2; return 1; }
  temporary="$(mktemp)"
  dpkg-deb --root-owner-group --build "$tree" "$temporary" > /dev/null
  chmod 0644 "$temporary"
  cp --no-clobber "$temporary" "$deb"
  cmp -s "$temporary" "$deb" || { echo "artifact appeared during the build: $deb" >&2; return 1; }
  rm -f -- "$temporary"
}

#!/usr/bin/env bash
# Build the antz_core release archive for one antz-core-v* tag, and its sha256.
#
# A consumer (the Streamdog C SDK, the Streamdog firmware) takes antz_core by
# tag and sha256 and compiles it: libs/antz_core and the licence, without the
# applications, the vendored ANT SDKs or the tests. The layout under the prefix
# is the repository's own (libs/antz_core), so CMake's FetchContent with
# SOURCE_SUBDIR libs/antz_core reads it the way it reads a checkout.
#
# git archive of a commit stamps every entry with the commit time and gzip -n
# drops the name and time from the header, so the same tag always gives the
# same tar, wherever it is run. The compressed file is not the same everywhere:
# GNU gzip (the release runner) and Apple gzip compress the same tar to different
# bytes, so the sha256 to pin is the one the release publishes, and a local run
# is checked by comparing the uncompressed tar (gzip -dc), not the hash.
#
# Usage: antz-core-archive.sh <tag> [out-dir]     e.g. antz-core-archive.sh antz-core-v0.1.0 dist
set -euo pipefail

tag="${1:?usage: antz-core-archive.sh <antz-core-vX.Y.Z tag> [out-dir]}"
out="${2:-.}"
case "${tag}" in antz-core-v*) ;; *) echo "not an antz-core-v* tag: ${tag}" >&2; exit 2 ;; esac
version="${tag#antz-core-v}"
name="antz-core-${version}"

git rev-parse --verify --quiet "${tag}^{commit}" >/dev/null \
  || { echo "tag ${tag} not found" >&2; exit 2; }

mkdir -p "${out}"
git -c tar.tar.gz.command="gzip -n" archive \
  --format=tar.gz \
  --prefix="${name}/" \
  -o "${out}/${name}.tar.gz" \
  "${tag}" -- libs/antz_core LICENSE

( cd "${out}" && sha256sum "${name}.tar.gz" > "${name}.tar.gz.sha256" )
cat "${out}/${name}.tar.gz.sha256"

# Releasing antz_core

`antz_core` (`libs/antz_core`) is tagged `antz-core-v<version>` and published as a
GitHub Release with a source archive and its sha256. The antz application's own
Raspberry Pi package keeps its `v*` tags and is not part of this.

The version is carried in two places. The release workflow checks that both
equal the tag, and fails if they do not.

| Where | What |
|---|---|
| `libs/antz_core/CMakeLists.txt` | `set(ANTZ_CORE_VERSION "x.y.z")` |
| `libs/antz_core/CHANGELOG.md` | the top section `## [x.y.z]`, which becomes the release notes |

A breaking change to a public header bumps the minor version while the major
version is 0.

## Steps

1. On a clean checkout of `main`, run the tests: `cmake -S . -B build -DANTZ_PLATFORM=linux && cmake --build build --target antz_tests && ctest --test-dir build --output-on-failure`.
2. Set the version in both places above, with the changes since the last tag in the CHANGELOG section. Land it through a pull request.
3. Build the archive locally from the tag you are about to push and note the hash:
   `git tag antz-core-v<version> && .github/scripts/antz-core-archive.sh antz-core-v<version> /tmp/antz-core-dist`
   Delete that local tag if the hash is not what you expected; the workflow rebuilds the same bytes from the pushed tag.
4. Push the tag: `git push origin antz-core-v<version>`. The workflow publishes the release.
5. Check the release page shows the archive and the `.sha256`, and that the hash matches step 3.
6. In a consumer (the Streamdog C SDK's CMake), replace the commit pin with the archive URL and this hash.

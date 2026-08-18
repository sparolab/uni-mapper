# Third-party notices

OpenLMM incorporates or links to third-party software. Each dependency remains subject to its own
license; this notice is an inventory, not a replacement for those terms.

Bundled license texts are retained under `open_lmm/thirdparty/<dependency>/LICENSE` for Eigen,
GKCM, GTSAM, KISS-Matcher, nanoflann, nlohmann/json, PCL, small_gicp, oneTBB and tqdm-cpp.
Installed packages copy them to `share/open_lmm/licenses/thirdparty`. ERASOR's GPL-3.0 license
text is retained in its source directory and installed at `share/open_lmm/licenses/erasor/LICENCE`.

Optional/system-resolved dependencies include ROS 2, Boost, OpenCV, OpenMP, spdlog, fmt,
Iridescence and Tracy. Distributors must preserve the notices supplied by the exact binary/source
versions included in their package and must review the dependency graph before redistribution.

The reproducible release environment and pinned source revisions are recorded in
`docker/open_lmm.Dockerfile` and the dependency CMake declarations. The OpenLMM project license is
GPL-3.0-only; see `LICENCE`.

# Sanitizer checks

Run the complete TSan lane in the ROS Humble toolchain image:

```sh
bash scripts/ci/build_sanitizer_tests.sh clang15-tsan TSAN /usr/bin/clang-15 /usr/bin/clang++-15
```

The build directory must be clean. Core and GUI run with the existing strict
TSan options and the OpenNI2 mutex suppression. ROS then runs the goal-admission
and graph contracts, including simultaneous action/visualization snapshots and
chained action requests. The required `sanitizer / tsan` job includes both phases.
Each selected ROS test runs ten times, stopping at the first failure; the graph
contract stops its executor before destroying callback state and subscriptions.
The sanitizer-only Docker job allows `setarch -R` via
`--security-opt seccomp=unconfined` to avoid shadow-memory mapping collisions.
This relaxes syscall filtering for that disposable CI container; it does not
change the production image defaults. Docker's profile behavior is documented
at https://docs.docker.com/engine/security/seccomp/.

The ROS phase relinks the installed LLVM Archer archive because some packaged
DSOs bind their TSan annotation stubs locally. Before testing ROS, a correct
OpenMP reduction must pass and an injected application race must fail; both
must report Archer activation. It uses LLVM's documented
`ignore_noninstrumented_modules=1` setting:
https://openmp.llvm.org/doxygen/dir_d25e7fcf0ab543d5b35a958546f34a9d.html

To observe DDS synchronization, the ROS phase builds Fast-CDR 1.0.29 and
Fast-DDS 2.6.11 with TSan at pinned commits. The overlay stays inside the CI
directory. Its public Fast-DDS configuration must match the system RMW's
configuration, and the test executable must load the overlay libraries.
Changing the Humble DDS version requires reviewing these pins together with
that configuration check. This is a test-only overlay, not a new distribution
or plugin ABI compatibility guarantee.

The ROS phase does not claim coverage of all prebuilt ROS, OpenMP, or system
library internals. The broad runtime exclusion is confined to this phase;
passing it alone cannot replace the strict core/GUI checks. No OpenLMM race
suppression is used.

For an existing instrumented core installation, rerun just the ROS phase:

```sh
bash scripts/ci/run_ros_tsan_tests.sh /tmp/ros-tsan /usr/bin/clang-15 /usr/bin/clang++-15 /absolute/path/to/core-install
```

Logs, positive/negative OpenMP controls, loaded library paths, the ROS test
manifest, and `ctest-ros.xml` are retained below the ROS build root. DDS needs
local sockets; concurrent runs must use distinct `ROS_DOMAIN_ID` values.

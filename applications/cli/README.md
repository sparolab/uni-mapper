# OpenLMM CLI

`open_lmm_batch` is the standalone C++ command-line application for running an
OpenLMM pipeline from a configuration directory. It is a leaf consumer of the
installed `open_lmm::client` target; it does not embed or build the core source
tree.

## Supported build contract

- OpenLMM core and CLI version: exact match (`3.0.0`)
- C++ standard: C++20
- supported release platform: Ubuntu 22.04 x86-64
- supported compilers: GCC 12, GCC 13, or Clang 15 with the documented
  libstdc++ 12 compatibility configuration

```bash
cmake -S applications/cli -B build/cli \
  -DCMAKE_PREFIX_PATH=/path/to/open_lmm/prefix
cmake --build build/cli
ctest --test-dir build/cli --output-on-failure
cmake --install build/cli --prefix /path/to/open_lmm/prefix \
  --component Tools
```

The combined prefix must already contain the matching OpenLMM core Runtime and
Plugins artifacts. The CLI owns `bin/open_lmm_batch`; the core artifact does
not own that path.

## Command contract

```text
open_lmm_batch <config_dir_path>
open_lmm_batch --help
```

Success returns `0`. Usage, bootstrap, submission, and execution failures
return `1` and write their diagnostic to stderr. Progress is written to stderr.
The application currently installs no signal handler; SIGINT and SIGTERM retain
the operating system's default process-termination behavior.

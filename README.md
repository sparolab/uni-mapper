# OpenLMM

OpenLMM is a modularized LiDAR map merging and long-term map management
framework. It is the generalized implementation of
[Uni-Mapper](https://ieeexplore.ieee.org/abstract/document/11057931) and
includes loop closure detection, robust optimization, multi-map alignment and
dynamic-object removal.

[![Uni-Mapper demo](https://img.youtube.com/vi/SK0TU9Vy3Is/maxresdefault.jpg)](https://www.youtube.com/watch?v=SK0TU9Vy3Is)

## Features

- Data loaders for PCD, KITTI BIN, KITTI poses, TUM poses and custom formats
- Dynamically loaded Scan Context and SOLiD loop detectors
- iSAM2-based backend optimization
- Online HMM-MOS and DUFOMap dynamic removal
- Offline ERASOR dynamic removal

## Official v3 artifacts

The supported binary surface is intentionally small:

- `ghcr.io/sparolab/uni-mapper:3.0.0`: Ubuntu 22.04 x86-64 Headless Core + CLI
- `open_lmm-3.0.0-cp310-*-linux_x86_64.whl`: CPython 3.10 Linux x86-64 SDK

ROS 2 Humble and the native GUI remain tested source-build adapters; they are
not official v3 binary artifacts. Candidate artifacts use the corresponding
`3.0.0-rc.N` version and are promoted without rebuilding.

Run the batch image with a mounted configuration and data directory:

```bash
docker run --rm \
  --volume "$PWD/open_lmm/config:/config:ro" \
  --volume "/path/to/dataset:/data:ro" \
  --volume "$PWD/output:/output" \
  ghcr.io/sparolab/uni-mapper:3.0.0 /config
```

Configuration paths inside the mounted files must refer to `/data` and
`/output`. Verify release files with `SHA256SUMS` and GitHub artifact
attestations before deployment.

Install the Python SDK into a clean CPython 3.10 environment:

```bash
python3.10 -m venv .venv
.venv/bin/pip install ./open_lmm-3.0.0-cp310-*-linux_x86_64.whl
.venv/bin/python -c 'import open_lmm; print(open_lmm.__version__)'
```

## Build from source

The reproducible CI toolchain image contains the supported compilers and ROS
dependencies:

```bash
docker build -f docker/open_lmm.Dockerfile -t open-lmm-dev .
docker run --rm -v "$PWD:/root/workspace" -w /root/workspace \
  open-lmm-dev scripts/ci/build_and_test.sh local \
  /usr/bin/gcc-12 /usr/bin/g++-12 OFF
```

Local owner-specific entry points are also available:

```bash
make core-build
make cli-build
make gui-build
make python-build
make ros-build
```

Use `make gui CONFIG=/absolute/path/to/config` to build and run the standalone
GUI, or `make ros CONFIG=/absolute/path/to/config` to launch the ROS 2 adapter
with RViz. See [applications/gui/README.md](applications/gui/README.md),
[ros/README.md](ros/README.md) and [bindings/python/README.md](bindings/python/README.md)
for the owner-specific instructions.

System PCL is required. Other dependencies may use the pinned fallbacks under
`open_lmm/thirdparty/`. The public C++ API uses `SOVERSION=3`; plugin ABI v1 is
same-toolchain compatible, and the Python API reports `API_VERSION = 1`.

## Dataset format

The example dataset is available from
[Google Drive](https://drive.google.com/drive/folders/1MiwAkoHn0tzPc5O6FQhFEykhTY4JBqxU?usp=sharing).

```text
dataset_root/
├── agent1/
│   ├── Scans/
│   │   ├── 000000.pcd
│   │   ├── 000001.pcd
│   │   └── ...
│   └── optimized_poses.txt
└── agent2/
    ├── Scans/
    └── optimized_poses.txt
```

Scan directory names, pose filenames and formats are configured in
`open_lmm/config/core/data_loader/file_based.json`.

## Correctness contract

Committed runtime state has one owner, `RuntimeStateStore`, and changes follow
candidate → validate → commit. Valid presentations remain visible until their
replacement is ready. These guarantees are process-level; OpenLMM does not
claim crash- or power-loss durability across a host failure.

See [RELEASE_POLICY.md](RELEASE_POLICY.md), [distribution/README.md](distribution/README.md)
and [AGENTS.md](AGENTS.md) for compatibility, packaging and architecture rules.

## Citation

```bibtex
@article{kang2025uni,
  title={Uni-Mapper: Unified Mapping Framework for Multi-modal LiDARs in Complex and Dynamic Environments},
  author={Kang, Gilhwan and Kim, Hogyun and Choi, Byunghee and Jeong, Seokhwan and Shin, Young-Sik and Cho, Younggun},
  journal={IEEE Transactions on Intelligent Vehicles},
  year={2025},
  publisher={IEEE}
}
```

## Acknowledgments

OpenLMM builds on LT-Mapper, GLIM, KISS-ICP, KISS-Matcher, ScanContext, SOLiD,
DynamicMap Benchmark, ERASOR, DUFOMap and HMM-MOS. Their authors and maintainers
made this project possible; dependency licenses are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## License

GPL-3.0-only. See [LICENCE](LICENCE).

Maintainer: Gilhwan Kang (`gilhwan@hyundai.com`).

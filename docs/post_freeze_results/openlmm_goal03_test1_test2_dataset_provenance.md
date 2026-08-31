# OpenLMM Goal 03 — `test1` / `test2` Dataset Provenance

## 1. Record status

```text
record_version: 1
recorded_at: 2026-08-25
dataset_id: openlmm-test1-test2-source-v1
status: PROVENANCE_COMPLETE_FOR_INTERNAL_USE
```

This record captures the dataset owner's declaration that `test1` and `test2`
were self-collected and may be used as the canonical source for OpenLMM Goal
03 replay formalization.

This is the canonical provenance and usage declaration rather than the replay
baseline itself. Dataset hashes, tier manifests, numeric tolerances, and local
replay evidence have since been produced by the onboarding steps referenced
below.

## 2. Ownership and usage rights

```text
source: self-collected
owner: requesting OpenLMM operator (dataset collector)
ownership_basis: first-party owner declaration recorded on 2026-08-25
license: LicenseRef-OpenLMM-Internal
redistributable: false
internal_use: permitted
modification_and_subset_generation: permitted
private_ci_use: permitted
public_release: requires a separate explicit owner decision
```

`LicenseRef-OpenLMM-Internal` is the initial conservative policy. It permits
owner-controlled development, regression testing, deterministic subset
generation, and use on owner-approved private CI/storage. It does not grant
public redistribution. The owner may replace it with an appropriate public
SPDX license in a separately reviewed dataset-release change.

The requesting operator states that they collected and own the dataset and
therefore have authority to approve its use, modification, subset generation,
and private distribution for this project. The owner's legal name or
organization need not be published for internal use. It must be recorded in
the release metadata before a public dataset release if the selected public
license or distribution channel requires it.

## 3. Canonical source inventory

The local paths below are discovery locations only. They must not appear as the
portable source location in a replay case or baseline manifest.

| Session | Discovery path | Runtime scans | Pose rows | Runtime input size |
|---|---|---:|---:|---:|
| `test1` | `/root/dataset_root/example/test1` | 310 binary XYZI PCD files | 310 | approximately 295 MiB plus poses |
| `test2` | `/root/dataset_root/example/test2` | 358 binary XYZI PCD files | 358 | approximately 337 MiB plus poses |

The authoritative runtime inputs for source version 1 are:

```text
test1/Scans/000000.pcd ... test1/Scans/000309.pcd
test1/optimized_poses.txt
test2/Scans/000000.pcd ... test2/Scans/000357.pcd
test2/optimized_poses.txt
```

Observed format:

```text
scan format: PCD v0.7, binary, x/y/z/intensity float32
pose format: KITTI-style 3x4 matrix, 12 values per row
scan/pose cardinality: exact for both sessions
```

The current OpenLMM file loader configuration selects `Scans` and
`optimized_poses.txt`. The following observed files are not canonical replay
inputs unless a later manifest explicitly promotes them:

```text
Scans_org/
cloudGlobal.pcd
Scans.zip
```

Their origin and relationship to `Scans` must not be inferred from their names.

## 4. Acquisition and processing disclosure

The owner has not retained or supplied the following historical acquisition
details. They are recorded as unavailable instead of being inferred from file
timestamps, directory names, or payload contents:

```text
acquisition_date: not recorded
acquisition_location: owner-private / not disclosed
sensor_vendor_model: not recorded
sensor_configuration: not recorded
pose_generation_pipeline: not recorded; optimized_poses.txt is the canonical supplied pose product
Scans preprocessing history: not recorded; current Scans payloads are the canonical supplied scans
Scans_org relationship: not recorded; excluded from the canonical replay source
```

Filesystem modification/birth times are not acquisition evidence and must not
be substituted for these fields.

This explicit unknown-value disclosure is sufficient for the internal replay
use authorized by the owner: the canonical byte set is independently locked,
and the replay does not claim sensor-calibration or absolute-accuracy ground
truth. A public dataset release must revisit these fields and the license; it
must not silently replace `not recorded` with inferred values.

## 5. Canonical replay tiers

All initial Goal 03 tiers are deterministically derived from this single
canonical source:

| Tier | Source selection | Purpose |
|---|---|---|
| `tiny-v1` | `test1` frames 110–149 and `test2` frames 100–139 | PR correctness replay |
| `small-v1` | `test1` frames 90–169 and `test2` frames 80–159 | PR/main full workflow and config rerun |
| `representative-v1` | complete 310 + 358 frame pair | nightly two-session real workload |
| `failure-corrupt-pcd-v1` | `tiny-v1` with `test2/Scans/000120.pcd` deterministically truncated to 64 bytes | pre-commit rollback regression |

`representative-v1` is explicitly scoped as a two-session representative
workload. It does not claim to cover 3–7 agent scale. A larger multi-agent tier
may be added later without changing this source version.

Frame ranges for `tiny-v1` and `small-v1` were selected from the source lock,
then approved after replay confirmed cross-session overlap and a real
alignment/loop outcome. The exact arrays, pose hashes, and scan-index hashes
live in the versioned selection and case manifests.

## 6. Permitted onboarding transformations

The versioned subset generator may perform only the following transformations
unless this record is revised:

```text
- select an exact, versioned list of source frames
- select the matching pose rows in the same order
- create the replay directory layout and configuration
- create file-level SHA-256 indexes
- normalize archive ordering, mtime, uid/gid, and compression options
- derive the failure tier by a versioned deterministic corruption recipe
```

It must not silently resample, filter, voxelize, alter pose values, or rewrite
PCD payloads for normal tiers. Any such operation requires a new transformation
record and a new source or tier version.

## 7. Immutable distribution policy

The canonical local discovery paths are not a distribution mechanism. Every
admitted replay tier has:

```text
- an immutable owner-controlled private artifact location
- archive SHA-256
- file-level SHA-256, including pose and scan indexes
- config SHA-256
- an attribution file copied into the bundle
- a versioned replay case manifest
```

Public CI is not authorized to fetch this internal dataset under the current
policy. Required public CI must either use an owner-approved public release of
the relevant subsets or execute on an authorized private runner.

## 8. Attribution template for generated bundles

Each generated bundle includes an `ATTRIBUTION.md` containing at least:

```text
Dataset: OpenLMM test1/test2 replay dataset
Source version: openlmm-test1-test2-source-v1
Source: self-collected
Owner: requesting OpenLMM operator (dataset collector)
License: LicenseRef-OpenLMM-Internal
Redistribution: prohibited unless separately authorized by the owner
Transformations: see the tier manifest and generator version
```

## 9. Admission decision

```text
canonical source selection: APPROVED
local inventory/cardinality: VERIFIED
technical replay suitability: VERIFIED_BY_PRIOR_REPLAY_EVIDENCE
ownership declaration: RECORDED
internal use authorization: APPROVED
public redistribution: NOT_APPROVED
acquisition/sensor metadata: DISCLOSED_AS_NOT_RECORDED
content/archive hash lock: COMPLETE
tier manifests: COMPLETE_FOR_TINY_SMALL_REPRESENTATIVE_FAILURE
failure tier: COMPLETE_AND_REPLAY_VERIFIED
tiny five-run replay baseline: COMPLETE_AND_REVIEWED
internal CI workflow contract: COMPLETE_CONFIGURATION_PENDING
```

The required repository-side onboarding work is complete. The remaining
operational checkpoint is configuring the owner-controlled immutable artifact
URLs, archive hashes, and pinned replay image in repository settings, then
recording the first successful workflow run. Public redistribution remains a
separate owner decision and is not implied by internal provenance completion.

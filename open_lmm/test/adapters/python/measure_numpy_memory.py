from __future__ import annotations

import json
import resource

import open_lmm

from python_test_fixture import RuntimeFixture


def main() -> None:
    fixture = RuntimeFixture(
        "numpy-memory",
        scan_count=3,
        points_per_scan=20000,
        voxel_size=0.001,
    )
    try:
        with open_lmm.Runtime(1) as runtime:
            runtime.open(fixture.config, label="numpy-memory")
            runtime.run_stage(open_lmm.Stage.DATA_LOAD).wait()
            before_kib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            visualization = runtime.visualization(
                "agent1",
                preview_voxel_size_m=0.001,
            )
            after_kib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            print(
                json.dumps(
                    {
                        "displayed_point_count": visualization.displayed_point_count,
                        "numpy_owner_bytes": visualization.points.nbytes,
                        "max_rss_before_kib": before_kib,
                        "max_rss_after_kib": after_kib,
                        "max_rss_delta_kib": max(0, after_kib - before_kib),
                    },
                    sort_keys=True,
                )
            )
    finally:
        fixture.cleanup()


if __name__ == "__main__":
    main()

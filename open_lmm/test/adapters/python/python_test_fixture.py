from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path


class RuntimeFixture:
    def __init__(
        self,
        name: str,
        scan_count: int = 1,
        *,
        points_per_scan: int = 1,
        voxel_size: float = 0.5,
    ) -> None:
        root = Path(tempfile.mkdtemp(prefix=f"open_lmm_python_{name}_"))
        self.root = root
        self.config = root / "config"
        self.data = root / "data"
        self.output = root / "output"
        self.scan_count = scan_count
        self.points_per_scan = points_per_scan
        self.voxel_size = voxel_size
        self._write()

    def cleanup(self) -> None:
        shutil.rmtree(self.root, ignore_errors=True)

    def _write(self) -> None:
        scans = self.data / "agent1" / "Scans"
        scans.mkdir(parents=True)
        (self.config / "server").mkdir(parents=True)
        (self.config / "core").mkdir(parents=True)

        poses = []
        pcd = (
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
            "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            f"WIDTH {self.points_per_scan}\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {self.points_per_scan}\nDATA ascii\n"
        )
        points = "".join(
            f"{10.0 + point_index * 0.001:.6f} 0 0 1\n"
            for point_index in range(self.points_per_scan)
        )
        for index in range(self.scan_count):
            poses.append(f"1 0 0 {index * 0.1} 0 1 0 0 0 0 1 0\n")
            (scans / f"{index:06d}.pcd").write_text(
                pcd + points, encoding="utf-8"
            )
        (self.data / "agent1" / "poses.txt").write_text(
            "".join(poses), encoding="utf-8"
        )

        root_config = {
            "global": {
                "config_map_server": "server/map.json",
                "config_data_loader": "core/data.json",
                "config_loop_detector": "core/loop.json",
                "config_backend_optimizer": "core/optimizer.json",
                "config_dynamic_remover": "core/remover.json",
            },
            "directory": {
                "root_dir_path": str(self.data),
                "sub_dir_list": ["agent1"],
                "root_save_dir": str(self.output),
            },
        }
        (self.config / "config.json").write_text(
            json.dumps(root_config), encoding="utf-8"
        )
        (self.config / "server" / "map.json").write_text(
            json.dumps(
                {
                    "map_server": {
                        "enable_map_updater": False,
                        "anchor_agent_index": 0,
                        "save_voxel_size": 0.2,
                        "parallel_data_load": False,
                        "parallel_map_update": False,
                        "max_parallel_agents": 1,
                    }
                }
            ),
            encoding="utf-8",
        )
        (self.config / "core" / "data.json").write_text(
            json.dumps(
                {
                    "data_loader": {
                        "data_loader_type": "file_based",
                        "pose_format": "kitti",
                        "pose_file_name": "poses.txt",
                        "extrinsic": [0, 0, 0, 0, 0, 0, 1],
                        "scan_type": "pcd",
                        "scan_dir_name": "Scans",
                        "voxel_size": self.voxel_size,
                        "min_range": 1.0,
                        "max_range": 60.0,
                        "delimiter": " ",
                    }
                }
            ),
            encoding="utf-8",
        )
        (self.config / "core" / "loop.json").write_text(
            json.dumps(
                {
                    "loop_detector": {
                        "loop_detector_type": "kdtree",
                        "model": "scan_context",
                    },
                    "database": {
                        "descriptor_vector_dim": 20,
                        "distance_threshold": 0.15,
                        "num_candidates": 3,
                        "rebuild_threshold": 50,
                    },
                    "alignment": {
                        "pcm_translation_threshold": 10.0,
                        "pcm_rotation_threshold_deg": 20.0,
                        "pcm_solver": "heuristic",
                        "pcm_threads": 1,
                        "pcm_max_candidates": 0,
                    },
                }
            ),
            encoding="utf-8",
        )
        (self.config / "core" / "optimizer.json").write_text(
            json.dumps(
                {
                    "backend_optimizer": {
                        "backend_optimizer_type": "incremental",
                        "relinearizeThreshold": 0.1,
                        "relinearizeSkip": 1,
                        "isam_extra_updates": 1,
                        "min_loop_frame_gap": 30,
                        "icp_search_num": 1,
                    }
                }
            ),
            encoding="utf-8",
        )
        (self.config / "core" / "remover.json").write_text(
            json.dumps(
                {
                    "dynamic_remover": {
                        "dynamic_remover_type": "offline",
                        "model": "free_dom",
                    }
                }
            ),
            encoding="utf-8",
        )

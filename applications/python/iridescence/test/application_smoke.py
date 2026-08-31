from __future__ import annotations

from types import SimpleNamespace
import threading

import numpy as np

from open_lmm_iridescence.application import IridescenceApplication


class Subscription:
    def close(self) -> None:
        pass


class Runtime:
    def __init__(self) -> None:
        self.open = True

    def is_open(self) -> bool:
        return self.open

    def set_alignment_feedback_enabled(self, _enabled: bool) -> None:
        pass

    def subscribe_events(self, _callback) -> Subscription:
        return Subscription()

    def node_descriptors(self):
        return ()

    def snapshot(self):
        pipeline = SimpleNamespace(
            runtime_revision=1,
            config_revision=1,
            agents=("agent1",),
            artifacts=(),
            job=None,
        )
        return SimpleNamespace(status="READY", pipeline=pipeline)

    def alignment_feedback(self):
        return None

    def recent_logs(self):
        return ("smoke log",)

    def visualization(self, agent, **_options):
        poses = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], 2, axis=0)
        poses[1, 0, 3] = 1.0
        return SimpleNamespace(
            agent=agent,
            revision=1,
            phase="DATA_LOAD",
            point_kind="FILTERED_SCAN_PREVIEW",
            poses=poses,
            edges=(),
            points=np.asarray([[0, 0, 0, 0], [1, 0, 0, 1]], dtype=np.float32),
            points_available=True,
            has_bounds=True,
            min_bound=np.asarray([0, 0, 0], dtype=np.float32),
            max_bound=np.asarray([1, 0, 0], dtype=np.float32),
            displayed_point_count=2,
            source_point_count=2,
        )


stop = threading.Event()
threading.Timer(0.75, stop.set).start()
IridescenceApplication(Runtime()).run(stop)

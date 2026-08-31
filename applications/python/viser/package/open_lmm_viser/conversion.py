from __future__ import annotations

import colorsys
from dataclasses import dataclass
from urllib.parse import quote

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True, slots=True)
class PointCloudPayload:
    points: npt.NDArray[np.float32]
    colors: npt.NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class VisualizationCandidate:
    agent: str
    revision: int
    point_cloud: PointCloudPayload | None
    trajectory: npt.NDArray[np.float32] | None
    color: tuple[int, int, int]


_IRIDESCENCE_PALETTE = (
    (230, 51, 51),
    (51, 166, 255),
    (51, 217, 89),
    (255, 166, 26),
    (166, 89, 255),
    (26, 217, 217),
    (255, 89, 191),
    (191, 191, 51),
)


def agent_color(index: int) -> tuple[int, int, int]:
    if not isinstance(index, int) or isinstance(index, bool) or index < 0:
        raise ValueError("agent color index must be a non-negative integer")
    if index < len(_IRIDESCENCE_PALETTE):
        return _IRIDESCENCE_PALETTE[index]
    hue = ((index - len(_IRIDESCENCE_PALETTE)) * 0.6180339887498949 + 0.08) % 1.0
    rgb = colorsys.hsv_to_rgb(hue, 0.72, 1.0)
    return tuple(int(round(component * 255.0)) for component in rgb)


def scene_key(agent: str) -> str:
    if not isinstance(agent, str) or not agent:
        raise ValueError("agent must be a non-empty string")
    return quote(agent, safe="-_.~")


def point_cloud_payload(
    points: object, base_color: tuple[int, int, int] | None = None
) -> PointCloudPayload:
    if not isinstance(points, np.ndarray):
        raise TypeError("points must be a NumPy array")
    if points.dtype != np.float32 or points.ndim != 2 or points.shape[1:] != (4,):
        raise ValueError("points must have dtype float32 and shape (N, 4)")

    xyz = np.ascontiguousarray(points[:, :3], dtype=np.float32)
    if not np.all(np.isfinite(xyz)):
        raise ValueError("point coordinates must be finite")

    intensity = points[:, 3]
    if not np.all(np.isfinite(intensity)):
        raise ValueError("point intensity must be finite")
    if intensity.size == 0:
        colors = np.empty((0, 3), dtype=np.uint8)
    else:
        minimum = float(np.min(intensity))
        maximum = float(np.max(intensity))
        if minimum == maximum:
            grayscale = np.full(intensity.shape, 192, dtype=np.uint8)
        else:
            normalized = (intensity.astype(np.float64) - minimum) / (
                maximum - minimum
            )
            grayscale = np.clip(np.rint(normalized * 255.0), 0, 255).astype(
                np.uint8
            )
        if base_color is None:
            colors = np.ascontiguousarray(np.repeat(grayscale[:, None], 3, axis=1))
        else:
            color = np.asarray(base_color, dtype=np.uint8)
            if color.shape != (3,):
                raise ValueError("base_color must contain exactly three RGB values")
            brightness = 0.35 + grayscale.astype(np.float32) / 255.0 * 0.65
            colors = np.ascontiguousarray(
                np.rint(brightness[:, None] * color[None, :]).clip(0, 255),
                dtype=np.uint8,
            )
    return PointCloudPayload(xyz, colors)


def trajectory_segments(poses: object) -> npt.NDArray[np.float32] | None:
    if not isinstance(poses, np.ndarray):
        raise TypeError("poses must be a NumPy array")
    if poses.dtype != np.float32 or poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("poses must have dtype float32 and shape (M, 4, 4)")
    if not np.all(np.isfinite(poses)):
        raise ValueError("pose matrices must be finite")
    if poses.shape[0] < 2:
        return None
    positions = np.ascontiguousarray(poses[:, :3, 3], dtype=np.float32)
    return np.ascontiguousarray(
        np.stack((positions[:-1], positions[1:]), axis=1), dtype=np.float32
    )


def visualization_candidate(
    snapshot: object, color: tuple[int, int, int] = _IRIDESCENCE_PALETTE[0]
) -> VisualizationCandidate:
    agent = getattr(snapshot, "agent", None)
    revision = getattr(snapshot, "revision", None)
    if not isinstance(agent, str) or not agent:
        raise ValueError("visualization snapshot has no valid agent")
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 0:
        raise ValueError("visualization snapshot has no valid revision")

    point_cloud = None
    if bool(getattr(snapshot, "points_available", False)):
        point_cloud = point_cloud_payload(getattr(snapshot, "points", None), color)
    trajectory = trajectory_segments(getattr(snapshot, "poses", None))
    return VisualizationCandidate(agent, revision, point_cloud, trajectory, color)

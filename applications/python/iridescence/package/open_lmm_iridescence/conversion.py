from __future__ import annotations

import colorsys
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from .state import PresentationMetadata, PresentationToken, enum_name


PALETTE = (
    (0.90, 0.20, 0.20, 1.0),
    (0.20, 0.65, 1.00, 1.0),
    (0.20, 0.85, 0.35, 1.0),
    (1.00, 0.65, 0.10, 1.0),
    (0.65, 0.35, 1.00, 1.0),
    (0.10, 0.85, 0.85, 1.0),
    (1.00, 0.35, 0.75, 1.0),
    (0.75, 0.75, 0.20, 1.0),
)


def agent_color(index: int) -> tuple[float, float, float, float]:
    if index < 0:
        raise ValueError("agent color index must be non-negative")
    if index < len(PALETTE):
        return PALETTE[index]
    hue = ((index - len(PALETTE)) * 0.6180339887498949 + 0.08) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.72, 1.0)
    return red, green, blue, 1.0


@dataclass(slots=True)
class RenderCandidate:
    token: PresentationToken
    agent: str
    phase: str
    point_kind: str
    points: npt.NDArray[np.float32] | None
    intensity_colors: npt.NDArray[np.float32] | None
    trajectory: npt.NDArray[np.float32] | None
    pose_positions: npt.NDArray[np.float32]
    intra_loops: npt.NDArray[np.float32] | None
    inter_loops: npt.NDArray[np.float32] | None
    inter_edges: tuple[tuple[str, int, str, int], ...]
    pose_axes: tuple[npt.NDArray[np.float32], ...]
    metadata: PresentationMetadata


def _poses(value: object) -> npt.NDArray[np.float32]:
    poses = np.asarray(value)
    if poses.dtype != np.float32 or poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError("poses must have dtype float32 and shape (N, 4, 4)")
    if not np.all(np.isfinite(poses)):
        raise ValueError("poses must be finite")
    return poses


def _points(value: object) -> npt.NDArray[np.float32]:
    points = np.asarray(value)
    if points.dtype != np.float32 or points.ndim != 2 or points.shape[1:] != (4,):
        raise ValueError("points must have dtype float32 and shape (N, 4)")
    if not np.all(np.isfinite(points)):
        raise ValueError("points must be finite")
    return points


def intensity_colors(points: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
    intensity = points[:, 3]
    if intensity.size == 0:
        return np.empty((0, 4), dtype=np.float32)
    minimum = float(np.min(intensity))
    maximum = float(np.max(intensity))
    normalized = (
        np.full(intensity.shape, 0.5, dtype=np.float32)
        if minimum == maximum
        else np.asarray((intensity - minimum) / (maximum - minimum), dtype=np.float32)
    )
    colors = np.empty((points.shape[0], 4), dtype=np.float32)
    colors[:, 0] = normalized
    colors[:, 1] = 0.15
    colors[:, 2] = 1.0 - normalized
    colors[:, 3] = 1.0
    return np.ascontiguousarray(colors)


def _edge_vertices(snapshot: Any, poses: npt.NDArray[np.float32], kind: str) -> npt.NDArray[np.float32] | None:
    vertices: list[npt.NDArray[np.float32]] = []
    agent = str(snapshot.agent)
    for edge in snapshot.edges:
        if enum_name(edge.type).upper() != kind:
            continue
        if edge.from_agent != agent or edge.to_agent != agent:
            continue
        if edge.from_index < 0 or edge.to_index < 0:
            continue
        if edge.from_index >= len(poses) or edge.to_index >= len(poses):
            continue
        vertices.extend((poses[edge.from_index, :3, 3], poses[edge.to_index, :3, 3]))
    if not vertices:
        return None
    return np.ascontiguousarray(vertices, dtype=np.float32)


def _inter_edges(snapshot: Any) -> tuple[tuple[str, int, str, int], ...]:
    return tuple(
        (edge.from_agent, int(edge.from_index), edge.to_agent, int(edge.to_index))
        for edge in snapshot.edges
        if enum_name(edge.type).upper() == "INTER_LOOP"
        and edge.from_index >= 0
        and edge.to_index >= 0
    )


def render_candidate(snapshot: Any, token: PresentationToken, color_mode: str) -> RenderCandidate:
    if str(snapshot.agent) != token.agent:
        raise ValueError("visualization result agent does not match request")
    poses = _poses(snapshot.poses)
    point_kind = enum_name(snapshot.point_kind).upper()
    point_array: npt.NDArray[np.float32] | None = None
    colors: npt.NDArray[np.float32] | None = None
    if bool(snapshot.points_available):
        source = _points(snapshot.points)
        point_array = np.ascontiguousarray(source[:, :3], dtype=np.float32)
        if color_mode == "INTENSITY" and point_kind == "FINAL_STATIC_MAP":
            colors = intensity_colors(source)
    positions = np.ascontiguousarray(poses[:, :3, 3], dtype=np.float32)
    trajectory = positions if len(positions) >= 2 else None
    stride = max(1, len(poses) // 100) if len(poses) else 1
    axes = tuple(np.ascontiguousarray(pose) for pose in poses[::stride])
    minimum = None
    maximum = None
    if bool(snapshot.has_bounds):
        minimum = tuple(float(value) for value in np.asarray(snapshot.min_bound)[:3])
        maximum = tuple(float(value) for value in np.asarray(snapshot.max_bound)[:3])
    result_token = PresentationToken(
        token.epoch, token.agent, int(snapshot.revision), token.generation
    )
    metadata = PresentationMetadata(
        result_token,
        int(snapshot.displayed_point_count),
        int(snapshot.source_point_count),
        minimum,
        maximum,
    )
    return RenderCandidate(
        result_token,
        token.agent,
        enum_name(snapshot.phase).upper(),
        point_kind,
        point_array,
        colors,
        trajectory,
        positions,
        _edge_vertices(snapshot, poses, "INTRA_LOOP"),
        None,
        _inter_edges(snapshot),
        axes,
        metadata,
    )

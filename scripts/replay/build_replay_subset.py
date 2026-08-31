#!/usr/bin/env python3
"""Lock a replay source and build deterministic OpenLMM replay subsets."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile
from typing import Any


def fail(message: str) -> None:
    raise ValueError(message)


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        fail(f"JSON root must be an object: {path}")
    return value


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def safe_component(value: Any, owner: str) -> str:
    if not isinstance(value, str) or not value:
        fail(f"{owner} must be a non-empty string")
    path = PurePosixPath(value)
    if path.is_absolute() or len(path.parts) != 1 or path.parts[0] in {".", ".."}:
        fail(f"{owner} must be one safe path component")
    return value


def require_fields(value: dict[str, Any], required: set[str], owner: str) -> None:
    if set(value) != required:
        fail(
            f"{owner} fields must be exactly {sorted(required)}, got {sorted(value)}"
        )


def canonical_files(root: Path, agents: list[dict[str, Any]]) -> list[Path]:
    files: list[Path] = []
    for entry in agents:
        require_fields(entry, {"id", "source_directory"}, "source agent")
        safe_component(entry["id"], "source agent id")
        source_directory = safe_component(
            entry["source_directory"], "source agent directory"
        )
        agent_root = root / source_directory
        pose = agent_root / "optimized_poses.txt"
        scan_root = agent_root / "Scans"
        if not pose.is_file() or not scan_root.is_dir():
            fail(f"source agent layout is incomplete: {agent_root}")
        scans = sorted(scan_root.glob("*.pcd"))
        if not scans:
            fail(f"source agent has no PCD scans: {agent_root}")
        if any(not scan.is_file() or scan.is_symlink() for scan in scans):
            fail(f"source scan must be a regular non-symlink file: {agent_root}")
        pose_count = sum(
            1 for line in pose.read_text(encoding="utf-8").splitlines() if line
        )
        if pose_count != len(scans):
            fail(
                f"scan/pose count mismatch for {source_directory}: "
                f"{len(scans)} scans vs {pose_count} poses"
            )
        files.extend([pose, *scans])
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def command_lock(args: argparse.Namespace) -> None:
    root = args.source_root.resolve(strict=True)
    specification = load_json(args.source_spec)
    require_fields(
        specification,
        {
            "schema_version",
            "dataset_id",
            "source",
            "owner",
            "license",
            "redistributable",
            "agents",
        },
        "source specification",
    )
    if specification["schema_version"] != 1:
        fail("source specification schema_version must be 1")
    if not isinstance(specification["agents"], list) or not specification["agents"]:
        fail("source specification agents must be a non-empty array")
    if args.output.exists():
        fail(f"source lock output already exists: {args.output}")

    locked_files = []
    for path in canonical_files(root, specification["agents"]):
        locked_files.append(
            {
                "path": path.relative_to(root).as_posix(),
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    content_index = "".join(
        f"{entry['sha256']}  {entry['path']}\n" for entry in locked_files
    )
    output = {
        "schema_version": 1,
        "dataset_id": specification["dataset_id"],
        "source": specification["source"],
        "owner": specification["owner"],
        "license": specification["license"],
        "redistributable": specification["redistributable"],
        "content_index_sha256": sha256_text(content_index),
        "files": locked_files,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, output)
    print(json.dumps({"source_lock": str(args.output), **output}, indent=2))


def read_lock(path: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    lock = load_json(path)
    require_fields(
        lock,
        {
            "schema_version",
            "dataset_id",
            "source",
            "owner",
            "license",
            "redistributable",
            "content_index_sha256",
            "files",
        },
        "source lock",
    )
    if lock["schema_version"] != 1 or not isinstance(lock["files"], list):
        fail("invalid source lock")
    files: dict[str, dict[str, Any]] = {}
    lines = []
    for entry in lock["files"]:
        require_fields(entry, {"path", "sha256", "size_bytes"}, "source lock file")
        relative = entry["path"]
        if not isinstance(relative, str) or relative in files:
            fail("source lock contains an invalid or duplicate path")
        if not isinstance(entry["sha256"], str) or len(entry["sha256"]) != 64:
            fail("source lock contains an invalid SHA-256")
        files[relative] = entry
        lines.append(f"{entry['sha256']}  {relative}\n")
    if sha256_text("".join(lines)) != lock["content_index_sha256"]:
        fail("source lock content index digest does not match")
    return lock, files


def verify_locked_file(root: Path, relative: str, entry: dict[str, Any]) -> Path:
    path = root / PurePosixPath(relative)
    if not path.is_file() or path.is_symlink():
        fail(f"locked source file is unavailable: {relative}")
    if path.stat().st_size != entry["size_bytes"] or sha256_file(path) != entry["sha256"]:
        fail(f"locked source file changed: {relative}")
    return path


def add_deterministic_tar_entry(archive: tarfile.TarFile, root: Path, path: Path) -> None:
    relative = path.relative_to(root).as_posix()
    info = archive.gettarinfo(str(path), arcname=relative)
    info.mtime = 0
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mode = 0o755 if path.is_dir() else 0o644
    if path.is_file():
        with path.open("rb") as stream:
            archive.addfile(info, stream)
    else:
        archive.addfile(info)


def create_archive(root: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
            with tarfile.open(
                fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT
            ) as archive:
                entries = [root, *sorted(root.rglob("*"), key=lambda p: p.relative_to(root).as_posix())]
                for entry in entries:
                    if entry == root:
                        continue
                    add_deterministic_tar_entry(archive, root, entry)


def command_build(args: argparse.Namespace) -> None:
    root = args.source_root.resolve(strict=True)
    source_lock, locked_files = read_lock(args.source_lock)
    selection = load_json(args.selection)
    required_selection_fields = {
        "schema_version",
        "tier",
        "tier_id",
        "source_dataset_id",
        "generator_version",
        "agents",
    }
    allowed_selection_fields = required_selection_fields | {"corruption"}
    if not required_selection_fields.issubset(selection) or not set(selection).issubset(
        allowed_selection_fields
    ):
        fail(
            "subset selection fields must contain exactly the required fields "
            "and optional corruption metadata"
        )
    if selection["schema_version"] != 1 or selection["generator_version"] != 1:
        fail("unsupported subset selection or generator version")
    if selection["source_dataset_id"] != source_lock["dataset_id"]:
        fail("selection source_dataset_id does not match source lock")
    if selection["tier"] not in {"tiny", "small", "representative", "failure"}:
        fail("unsupported replay tier")
    corruption = selection.get("corruption")
    if selection["tier"] == "failure":
        if not isinstance(corruption, dict):
            fail("failure tier requires deterministic corruption metadata")
        require_fields(
            corruption, {"agent_id", "frame", "method", "keep_bytes"}, "corruption"
        )
        safe_component(corruption["agent_id"], "corruption agent id")
        if corruption["method"] != "truncate":
            fail("unsupported corruption method")
        if (
            not isinstance(corruption["frame"], int)
            or corruption["frame"] < 0
            or not isinstance(corruption["keep_bytes"], int)
            or corruption["keep_bytes"] < 1
        ):
            fail("corruption frame and keep_bytes must be valid positive integers")
    elif corruption is not None:
        fail("corruption metadata is allowed only for the failure tier")
    if not isinstance(selection["agents"], list) or not selection["agents"]:
        fail("subset selection agents must be a non-empty array")
    if args.output.exists() or args.archive.exists():
        fail("subset output and archive paths must not already exist")

    args.output.mkdir(parents=True)
    generated_agents = []
    corruption_applied = False
    try:
        for agent in selection["agents"]:
            require_fields(
                agent, {"id", "source_directory", "frames"}, "subset agent"
            )
            agent_id = safe_component(agent["id"], "subset agent id")
            source_directory = safe_component(
                agent["source_directory"], "subset source directory"
            )
            frames = agent["frames"]
            if (
                not isinstance(frames, list)
                or not frames
                or any(not isinstance(frame, int) or frame < 0 for frame in frames)
                or frames != sorted(set(frames))
            ):
                fail("subset frames must be unique increasing non-negative integers")

            source_pose_relative = f"{source_directory}/optimized_poses.txt"
            pose_lock = locked_files.get(source_pose_relative)
            if pose_lock is None:
                fail(f"pose is absent from source lock: {source_pose_relative}")
            source_pose = verify_locked_file(root, source_pose_relative, pose_lock)
            pose_lines = source_pose.read_text(encoding="utf-8").splitlines()
            if frames[-1] >= len(pose_lines):
                fail(f"subset frame exceeds pose count for {agent_id}")

            agent_root = args.output / agent_id
            scan_root = agent_root / "Scans"
            scan_root.mkdir(parents=True)
            selected_pose = "".join(f"{pose_lines[frame]}\n" for frame in frames)
            pose_path = agent_root / "optimized_poses.txt"
            pose_path.write_text(selected_pose, encoding="utf-8", newline="\n")

            scan_index_lines = []
            for frame in frames:
                filename = f"{frame:06d}.pcd"
                source_relative = f"{source_directory}/Scans/{filename}"
                source_entry = locked_files.get(source_relative)
                if source_entry is None:
                    fail(f"scan is absent from source lock: {source_relative}")
                source_scan = verify_locked_file(root, source_relative, source_entry)
                destination_scan = scan_root / filename
                shutil.copyfile(source_scan, destination_scan)
                if (
                    corruption is not None
                    and corruption["agent_id"] == agent_id
                    and corruption["frame"] == frame
                ):
                    source_size = destination_scan.stat().st_size
                    if corruption["keep_bytes"] >= source_size:
                        fail("corruption keep_bytes must be smaller than the source scan")
                    with destination_scan.open("r+b") as stream:
                        stream.truncate(corruption["keep_bytes"])
                    corruption_applied = True
                relative = destination_scan.relative_to(args.output).as_posix()
                scan_index_lines.append(
                    f"{frame} {sha256_file(destination_scan)}  {relative}\n"
                )
            scan_index_path = agent_root / "scans.sha256"
            scan_index_path.write_text(
                "".join(scan_index_lines), encoding="utf-8", newline="\n"
            )
            generated_agents.append(
                {
                    "id": agent_id,
                    "frames": frames,
                    "pose_file": {
                        "path": pose_path.relative_to(args.output).as_posix(),
                        "sha256": sha256_file(pose_path),
                    },
                    "scan_index": {
                        "path": scan_index_path.relative_to(args.output).as_posix(),
                        "sha256": sha256_file(scan_index_path),
                    },
                }
            )

        if corruption is not None and not corruption_applied:
            fail("corruption target is absent from the selected frames")

        transformation = "exact frame/pose selection and deterministic packaging only"
        if corruption is not None:
            transformation = (
                "exact frame/pose selection, deterministic packaging, and "
                f"truncate {corruption['agent_id']} frame {corruption['frame']} "
                f"to {corruption['keep_bytes']} bytes"
            )
        attribution = (
            "# OpenLMM replay dataset attribution\n\n"
            f"- Dataset: {source_lock['dataset_id']}\n"
            f"- Source: {source_lock['source']}\n"
            f"- Owner: {source_lock['owner']}\n"
            f"- License: {source_lock['license']}\n"
            f"- Redistributable: {str(source_lock['redistributable']).lower()}\n"
            f"- Tier: {selection['tier_id']}\n"
            f"- Transformations: {transformation}\n"
        )
        (args.output / "ATTRIBUTION.md").write_text(
            attribution, encoding="utf-8", newline="\n"
        )
        metadata = {
            "schema_version": 1,
            "bundle_id": selection["tier_id"],
            "tier": selection["tier"],
            "source_dataset_id": source_lock["dataset_id"],
            "source_content_index_sha256": source_lock["content_index_sha256"],
            "selection_sha256": sha256_file(args.selection),
            "generator_version": 1,
            "agents": generated_agents,
        }
        if corruption is not None:
            metadata["corruption"] = corruption
        write_json(args.output / "BUNDLE.json", metadata)
        create_archive(args.output, args.archive)
    except Exception:
        shutil.rmtree(args.output, ignore_errors=True)
        if args.archive.exists():
            args.archive.unlink()
        raise

    result = {
        **metadata,
        "bundle_sha256": sha256_file(args.archive),
        "archive": str(args.archive),
        "data_root": str(args.output),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest="command", required=True)
    lock = commands.add_parser("lock-source")
    lock.add_argument("--source-root", type=Path, required=True)
    lock.add_argument("--source-spec", type=Path, required=True)
    lock.add_argument("--output", type=Path, required=True)
    lock.set_defaults(handler=command_lock)
    build = commands.add_parser("build")
    build.add_argument("--source-root", type=Path, required=True)
    build.add_argument("--source-lock", type=Path, required=True)
    build.add_argument("--selection", type=Path, required=True)
    build.add_argument("--output", type=Path, required=True)
    build.add_argument("--archive", type=Path, required=True)
    build.set_defaults(handler=command_build)
    return root


def main() -> int:
    try:
        arguments = parser().parse_args()
        arguments.handler(arguments)
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"replay subset error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

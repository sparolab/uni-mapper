#!/usr/bin/env python3
"""Create or verify the reviewed Goal 08 critical branch-coverage baseline."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import subprocess
import sys
from typing import Any


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: pathlib.Path) -> str:
    return sha256_bytes(path.read_bytes())


def read_sources(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    if not rows or set(rows[0]) != {"owner", "path"}:
        raise ValueError(f"invalid critical source manifest: {path}")
    owners = [row["owner"] for row in rows]
    sources = [row["path"] for row in rows]
    if len(owners) != len(set(owners)) or len(sources) != len(set(sources)):
        raise ValueError("critical source owner and path values must be unique")
    return rows


def command_output(command: list[str], cwd: pathlib.Path) -> str:
    result = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


def git_metadata(repo: pathlib.Path) -> dict[str, str]:
    status = command_output(["git", "status", "--porcelain=v1"], repo)
    return {
        "head": command_output(["git", "rev-parse", "HEAD"], repo),
        "worktree_status_sha256": sha256_bytes(status.encode()),
        "worktree_state": "clean" if not status else "modified",
    }


def summary_count(summary: dict[str, Any], category: str, field: str) -> int:
    try:
        return int(summary[category][field])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"missing LLVM coverage summary {category}.{field}") from exc


def load_export(path: pathlib.Path) -> list[dict[str, Any]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("type") != "llvm.coverage.json.export":
        raise ValueError("input is not an LLVM coverage JSON export")
    files: list[dict[str, Any]] = []
    for data in document.get("data", []):
        files.extend(data.get("files", []))
    if not files:
        raise ValueError("LLVM coverage export contains no file records")
    return files


def canonical_filename(value: str) -> pathlib.Path:
    return pathlib.Path(value).resolve()


def collect_measurements(
    export_path: pathlib.Path,
    source_root: pathlib.Path,
    rows: list[dict[str, str]],
) -> dict[str, dict[str, Any]]:
    indexed: dict[pathlib.Path, list[dict[str, Any]]] = {}
    for file_record in load_export(export_path):
        indexed.setdefault(canonical_filename(file_record["filename"]), []).append(file_record)

    owners: dict[str, dict[str, Any]] = {}
    for row in rows:
        source = (source_root / row["path"]).resolve()
        records = indexed.get(source, [])
        if len(records) != 1:
            raise ValueError(
                f"expected one coverage record for {row['owner']} ({source}), found {len(records)}"
            )
        summary = records[0]["summary"]
        branch_count = summary_count(summary, "branches", "count")
        branch_covered = summary_count(summary, "branches", "covered")
        if branch_count <= 0:
            raise ValueError(f"critical source has no instrumented branches: {source}")
        owners[row["owner"]] = {
            "path": row["path"],
            "source_sha256": sha256_file(source),
            "branches": {
                "count": branch_count,
                "covered": branch_covered,
                "rate": round(branch_covered / branch_count, 6),
            },
            "lines": {
                "count": summary_count(summary, "lines", "count"),
                "covered": summary_count(summary, "lines", "covered"),
            },
            "functions": {
                "count": summary_count(summary, "functions", "count"),
                "covered": summary_count(summary, "functions", "covered"),
            },
        }
    return owners


def verify_baseline(
    candidate: dict[str, Any], baseline: dict[str, Any]
) -> list[str]:
    failures: list[str] = []
    if baseline.get("schema_version") != 1 or baseline.get("status") != "reviewed":
        failures.append("baseline must have schema_version=1 and status=reviewed")

    candidate_owners = candidate["owners"]
    baseline_owners = baseline.get("owners", {})
    if set(candidate_owners) != set(baseline_owners):
        failures.append("critical owner set differs from the reviewed baseline")
        return failures

    if candidate["manifests"] != baseline.get("manifests"):
        failures.append("critical source/test manifest digest differs from the reviewed baseline")

    for owner, current in candidate_owners.items():
        reviewed = baseline_owners[owner]
        if current["path"] != reviewed.get("path"):
            failures.append(f"{owner}: source path differs from baseline")
        if current["source_sha256"] != reviewed.get("source_sha256"):
            failures.append(f"{owner}: critical source changed; review and recalibrate")
        reviewed_branches = reviewed.get("branches", {})
        if current["branches"]["count"] != reviewed_branches.get("count"):
            failures.append(f"{owner}: instrumented branch denominator changed")
        if current["branches"]["covered"] < reviewed_branches.get("covered", 0):
            failures.append(f"{owner}: covered branch count regressed")
        if current["branches"]["rate"] < reviewed_branches.get("rate", 0.0):
            failures.append(f"{owner}: branch coverage rate regressed")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--export", required=True, type=pathlib.Path)
    parser.add_argument("--sources", required=True, type=pathlib.Path)
    parser.add_argument("--tests", required=True, type=pathlib.Path)
    parser.add_argument("--source-root", required=True, type=pathlib.Path)
    parser.add_argument("--repo", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--mode", required=True, choices=("calibrate", "verify"))
    parser.add_argument("--baseline", type=pathlib.Path)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--llvm-cov", required=True)
    parser.add_argument("--image-digest", default="local-unpinned")
    args = parser.parse_args()

    try:
        rows = read_sources(args.sources)
        candidate: dict[str, Any] = {
            "schema_version": 1,
            "status": "candidate" if args.mode == "calibrate" else "measured",
            "git": git_metadata(args.repo.resolve()),
            "toolchain": {
                "compiler": args.compiler,
                "llvm_cov": args.llvm_cov,
                "image_digest": args.image_digest,
            },
            "manifests": {
                "critical_sources_sha256": sha256_file(args.sources),
                "critical_tests_sha256": sha256_file(args.tests),
            },
            "owners": collect_measurements(
                args.export, args.source_root.resolve(), rows
            ),
        }

        if args.mode == "verify":
            if args.baseline is None or not args.baseline.is_file():
                raise ValueError("verify mode requires an existing reviewed baseline")
            baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
            failures = verify_baseline(candidate, baseline)
            candidate["gate"] = {
                "baseline": str(args.baseline),
                "result": "fail" if failures else "pass",
                "failures": failures,
            }
        else:
            failures = []

        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        if failures:
            for failure in failures:
                print(f"coverage gate: {failure}", file=sys.stderr)
            return 1
    except (OSError, ValueError, KeyError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"critical coverage error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

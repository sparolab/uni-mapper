#!/usr/bin/env python3
"""Deterministic RuntimeStateStore mutation-pilot transformations and report."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import sys


MUTANTS = (
    (
        "M01",
        "normalize revision comparison inverted",
        "recovery_required->context.runtime_revision != committed_->revision",
        "recovery_required->context.runtime_revision == committed_->revision",
    ),
    (
        "M02",
        "normalization skips non-null recovery",
        "if (recovery_required && committed_ &&",
        "if (!recovery_required && committed_ &&",
    ),
    (
        "M03",
        "normalization skips committed authority",
        "if (recovery_required && committed_ &&",
        "if (recovery_required && !committed_ &&",
    ),
    (
        "M04",
        "initialize drops recovery authority",
        "recovery_required_ = std::move(recovery_required);",
        "recovery_required_.reset();",
    ),
    (
        "M05",
        "matches drops committed-state guard",
        "return committed_ && expected && committed_.get() == expected.get() &&",
        "return expected && committed_.get() == expected.get() &&",
    ),
    (
        "M06",
        "matches drops expected-state guard",
        "return committed_ && expected && committed_.get() == expected.get() &&",
        "return committed_ && committed_.get() == expected.get() &&",
    ),
    (
        "M07",
        "matches pointer identity inverted",
        "committed_.get() == expected.get() &&",
        "committed_.get() != expected.get() &&",
    ),
    (
        "M08",
        "matches revision equality inverted",
        "committed_->revision == expected->revision;",
        "committed_->revision != expected->revision;",
    ),
    (
        "M09",
        "recovery latch committed guard inverted",
        "if (committed_ && error) recovery_required_ = std::move(error);",
        "if (!committed_ && error) recovery_required_ = std::move(error);",
    ),
    (
        "M10",
        "commit recovery gate removed",
        "if (recovery_required_) {",
        "if (false && recovery_required_) {",
    ),
    (
        "M11",
        "expected pointer identity check removed",
        "committed_.get() != expected.get() ||",
        "false ||",
    ),
    (
        "M12",
        "expected revision conflict inverted",
        "committed_->revision != expected->revision) {",
        "committed_->revision == expected->revision) {",
    ),
    (
        "M13",
        "candidate null and revision checks conjoined",
        "if (!candidate || candidate->revision",
        "if (!candidate && candidate->revision",
    ),
    (
        "M14",
        "candidate revision validity inverted",
        "candidate->revision != expected->revision + 1",
        "candidate->revision == expected->revision + 1",
    ),
    (
        "M15",
        "candidate must advance by two",
        "expected->revision + 1) {",
        "expected->revision + 2) {",
    ),
    (
        "M16",
        "successful side effect returned before publication",
        "if (!side_effects) return side_effects;",
        "if (side_effects) return side_effects;",
    ),
    (
        "M17",
        "candidate publication removed",
        "committed_ = std::move(candidate);",
        "(void)candidate;",
    ),
    (
        "M18",
        "recovery outcome guard inverted",
        "if (outcome.recovery_required) {",
        "if (!outcome.recovery_required) {",
    ),
    (
        "M19",
        "recovery outcome publication removed",
        "recovery_required_ = outcome.recovery_required;",
        "recovery_required_.reset();",
    ),
    (
        "M20",
        "Commit result success polarity inverted",
        "return committed ? Result<void>::Ok()",
        "return !committed ? Result<void>::Ok()",
    ),
)


def sha256_file(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def list_mutants() -> int:
    writer = csv.writer(sys.stdout, delimiter="\t", lineterminator="\n")
    writer.writerow(("id", "description"))
    for mutant_id, description, _, _ in MUTANTS:
        writer.writerow((mutant_id, description))
    return 0


def apply_mutant(mutant_id: str, source: pathlib.Path, output: pathlib.Path) -> int:
    matches = [mutant for mutant in MUTANTS if mutant[0] == mutant_id]
    if len(matches) != 1:
        raise ValueError(f"unknown mutant: {mutant_id}")
    _, _, before, after = matches[0]
    text = source.read_text(encoding="utf-8")
    count = text.count(before)
    if count != 1:
        raise ValueError(
            f"{mutant_id} expected one transformation site, found {count}"
        )
    output.write_text(text.replace(before, after, 1), encoding="utf-8")
    return 0


def write_report(
    results_path: pathlib.Path,
    output: pathlib.Path,
    source: pathlib.Path,
    compiler: str,
    git_head: str,
    elapsed_seconds: float,
) -> int:
    with results_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    statuses = {name: 0 for name in ("killed", "survived", "compile-error", "timeout")}
    maximum_rss = 0
    equivalent = 0
    for row in rows:
        status = row["status"]
        if status not in statuses:
            raise ValueError(f"unexpected mutation status: {status}")
        statuses[status] += 1
        maximum_rss = max(maximum_rss, int(row["peak_rss_kb"]))
        if row["id"] in {"M05", "M06"} and status == "survived":
            statuses["survived"] -= 1
            equivalent += 1
            row["review_classification"] = "equivalent"
            row["review_reason"] = (
                "the following pointer-identity comparison already implies both "
                "shared_ptr operands are non-null"
            )
    document = {
        "schema_version": 1,
        "status": "feasibility-only",
        "tool": "openlmm-goal08-manual-mutator-v1",
        "compiler": compiler,
        "git_head": git_head,
        "source": str(source),
        "source_sha256": sha256_file(source),
        "mutants_requested": len(rows),
        "counts": {
            **statuses,
            "equivalent": equivalent,
            "equivalent_reviewed": equivalent,
        },
        "elapsed_seconds": round(elapsed_seconds, 3),
        "peak_build_rss_kb": maximum_rss,
        "required_gate_decision": "excluded",
        "exclusion_reasons": [
            "Mull has no pinned Ubuntu 22.04/Clang 15 package in the supported image",
            "manual transformations require reviewer adjudication of surviving/equivalent mutants",
            "the pilot is suitable for periodic feasibility evidence, not per-PR enforcement",
        ],
        "results": rows,
    }
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list")
    apply_parser = subparsers.add_parser("apply")
    apply_parser.add_argument("--mutant", required=True)
    apply_parser.add_argument("--source", required=True, type=pathlib.Path)
    apply_parser.add_argument("--output", required=True, type=pathlib.Path)
    report_parser = subparsers.add_parser("report")
    report_parser.add_argument("--results", required=True, type=pathlib.Path)
    report_parser.add_argument("--output", required=True, type=pathlib.Path)
    report_parser.add_argument("--source", required=True, type=pathlib.Path)
    report_parser.add_argument("--compiler", required=True)
    report_parser.add_argument("--git-head", required=True)
    report_parser.add_argument("--elapsed-seconds", required=True, type=float)
    args = parser.parse_args()
    try:
        if args.command == "list":
            return list_mutants()
        if args.command == "apply":
            return apply_mutant(args.mutant, args.source, args.output)
        return write_report(
            args.results,
            args.output,
            args.source,
            args.compiler,
            args.git_head,
            args.elapsed_seconds,
        )
    except (OSError, ValueError, KeyError) as exc:
        print(f"mutation pilot error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

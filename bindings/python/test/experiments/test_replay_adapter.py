from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path

from open_lmm.experiments import SoftwareIdentity
from open_lmm.experiments.replay import ReplayAdapter, ReplayRequest, ReplayStatus, ReplayToolchain


class ReplayAdapterTests(unittest.TestCase):
    def test_delegation_status_and_baseline_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runner = root / "runner"
            comparator = root / "comparator"
            report_template = {
                "schema_version": 1,
                "case_id": "fixture",
                "case_manifest_sha256": "0" * 64,
                "dataset_sha256": "1" * 64,
                "config_sha256": "2" * 64,
                "git": {"commit": "1" * 40, "dirty": False},
                "environment": {},
                "agents": ["agent1"],
                "steps": [],
                "health": {},
                "metrics": {"pose": {"translation_max_m": 0.1}},
                "artifacts": [],
                "diagnostics": {},
                "close_result": "succeeded",
            }
            runner.write_text(
                "#!/usr/bin/python3\nimport json,sys\na=sys.argv\np=a[a.index('--report')+1]\njson.dump(" + repr(report_template) + ",open(p,'w'))\n",
                encoding="utf-8",
            )
            comparator.write_text("#!/usr/bin/python3\nimport json,sys\na=sys.argv\np=a[a.index('--diff')+1]\njson.dump({'result':'pass'},open(p,'w'))\n", encoding="utf-8")
            runner.chmod(0o755)
            comparator.chmod(0o755)
            case = root / "case.json"
            baseline = root / "baseline.json"
            case.write_text("{}", encoding="utf-8")
            baseline.write_text("{}", encoding="utf-8")
            before = baseline.read_bytes()
            request = ReplayRequest(
                case, root, root, baseline, root / "evidence with spaces;safe",
                SoftwareIdentity(source_kind="git", source_identity="1" * 40),
                "sha256:" + "2" * 64,
            )
            result = ReplayAdapter(ReplayToolchain(runner, comparator)).run(request)
            self.assertEqual(result.status, ReplayStatus.SUCCEEDED)
            self.assertEqual(baseline.read_bytes(), before)
            self.assertTrue(result.metrics)
            comparator.write_text("#!/usr/bin/python3\nraise SystemExit(1)\n", encoding="utf-8")
            failed_request = ReplayRequest(
                case, root, root, baseline, root / "failed-evidence",
                request.software, request.container_digest,
            )
            self.assertEqual(
                ReplayAdapter(ReplayToolchain(runner, comparator)).run(failed_request).status,
                ReplayStatus.FAILED,
            )
            runner.write_text("#!/usr/bin/python3\nraise SystemExit(77)\n", encoding="utf-8")
            unavailable = ReplayRequest(
                case, root, root, baseline, root / "unavailable-evidence",
                request.software, request.container_digest,
            )
            self.assertEqual(
                ReplayAdapter(ReplayToolchain(runner, comparator)).run(unavailable).status,
                ReplayStatus.NOT_AVAILABLE,
            )

    def test_existing_goal03_comparator_contract(self) -> None:
        repository = Path(os.environ["OPEN_LMM_REPOSITORY_ROOT"])
        comparator = Path(os.environ["OPEN_LMM_REPLAY_COMPARE"])
        self.assertTrue(comparator.is_file())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            template = repository / "open_lmm/test/replay/fixtures/tiny_fixture_report_v1.json"
            baseline = repository / "open_lmm/test/replay/fixtures/tiny_fixture_baseline_v1.json"
            runner = root / "runner"
            runner.write_text(
                "#!/usr/bin/python3\nimport shutil,sys\na=sys.argv\nout=a[a.index('--report')+1]\nshutil.copyfile(" + repr(str(template)) + ",out)\n",
                encoding="utf-8",
            )
            runner.chmod(0o755)
            case = root / "case.json"
            case.write_text("{}", encoding="utf-8")
            request = ReplayRequest(
                case,
                root,
                root,
                baseline,
                root / "evidence",
                SoftwareIdentity(source_kind="git", source_identity="4" * 40),
                "sha256:" + "5" * 64,
            )
            result = ReplayAdapter(ReplayToolchain(runner, comparator)).run(request)
            self.assertEqual(result.status, ReplayStatus.SUCCEEDED)
            self.assertIsNotNone(result.diff_sha256)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from open_lmm.experiments import ExperimentResult, ExperimentStatus, MetricRecord, MetricSource, TrialResult
from open_lmm.experiments._export import CSV_COLUMNS


class ExportTests(unittest.TestCase):
    def test_json_csv_records_and_no_overwrite(self) -> None:
        digest = "sha256:" + "1" * 64
        trial = TrialResult("a" * 64, 1, 3, "succeeded", None, {"z": 1}, digest, digest, digest, (MetricRecord("wall", 4, "ns", MetricSource.PYTHON_CALLBACK, "trial"),))
        result = ExperimentResult("export", digest, (trial,), (), (), ExperimentStatus.SUCCEEDED)
        with tempfile.TemporaryDirectory() as directory:
            json_path = Path(directory) / "result.json"
            csv_path = Path(directory) / "metrics.csv"
            result.write_json(json_path)
            result.write_csv(csv_path)
            self.assertEqual(tuple(result.records()[0]), CSV_COLUMNS)
            with csv_path.open(newline="", encoding="utf-8") as stream:
                self.assertEqual(tuple(next(csv.reader(stream))), CSV_COLUMNS)
            with self.assertRaises(FileExistsError):
                result.write_json(json_path)


if __name__ == "__main__":
    unittest.main()

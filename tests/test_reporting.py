import csv
import io
import json
from html.parser import HTMLParser
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch

from PIL import Image

from grasp_demo.cli import main
from grasp_demo.reporting import OUTPUT_NAMES, report_html, write_report


def sample_result():
    return {
        "metrics": {"seed": 7, "case": "nominal", "success": True,
                    "failure_reason": "", "localization_error_m": 0.002,
                    "lift_height_m": 0.22, "simulated_seconds": 2.5,
                    "grasp_mode": "constraint-assisted; not contact-only",
                    "stage_events": [{"stage": "detect", "step": 1}, {"stage": "hold", "step": 600}]},
        "frames": [Image.new("RGB", (40, 30), "white"), Image.new("RGB", (40, 30), "red")],
        "stages": [{"stage": "detect", "frame_index": 0}, {"stage": "hold", "frame_index": 1}],
        "trajectory": [{"step": 1, "time_s": 0.01, "stage": "detect"},
                       {"step": 600, "time_s": 2.5, "stage": "hold"}],
    }


class ReportingTests(unittest.TestCase):
    def test_exported_numbers_and_trajectory_are_the_actual_result(self):
        result = sample_result()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_report(result, output)
            self.assertEqual(json.loads((output / "report.json").read_text()), result["metrics"])
            with (output / "trajectory.csv").open(newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([int(row["step"]) for row in rows], [1, 600])
            self.assertEqual(set(path.name for path in output.iterdir()), set(OUTPUT_NAMES))

    def test_replay_and_overview_are_decodable_with_no_personal_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_report(sample_result(), output)
            with Image.open(output / "preview.gif") as image:
                self.assertGreater(image.n_frames, 1)
                self.assertNotIn("comment", image.info)
                self.assertEqual(image.size, (40, 86))
            with Image.open(output / "overview.png") as image:
                image.verify()

    def test_html_escapes_metric_text_and_has_no_external_dependencies(self):
        result = sample_result()
        result["metrics"]["failure_reason"] = '<script>synthetic</script>'
        page = report_html(result["metrics"])
        self.assertNotIn("<script>", page)
        self.assertIn("&lt;script&gt;", page)
        class Resources(HTMLParser):
            external = []
            def handle_starttag(self, tag, attributes):
                for key, value in attributes:
                    if key in {"href", "src"} and value.startswith(("http:", "https:", "//")):
                        self.external.append(value)
        resources = Resources()
        resources.feed(page)
        self.assertEqual(resources.external, [])
        self.assertIn("固定约束辅助夹持", page)

    def test_nonfinite_metrics_do_not_create_a_report_directory(self):
        result = sample_result()
        result["metrics"]["lift_height_m"] = float("nan")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "new-report"
            with self.assertRaises(ValueError):
                write_report(result, output)
            self.assertFalse(output.exists())

    def test_absent_frames_are_not_presented_as_a_recorded_run(self):
        result = sample_result()
        result["frames"] = []
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "new-report"
            with self.assertRaises(ValueError):
                write_report(result, output)
            self.assertFalse(output.exists())

    def test_cli_rejects_seed_before_running_physics(self):
        with patch("grasp_demo.cli.run_episode") as run, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(["--seed", "-1"])
            run.assert_not_called()

    def test_cli_marks_unexpected_nominal_failure_nonzero(self):
        result = sample_result()
        result["metrics"]["success"] = False
        with patch("grasp_demo.cli.run_episode", return_value=result), patch("grasp_demo.cli.write_report", return_value=Path("report.html")), redirect_stdout(io.StringIO()):
            self.assertEqual(main([]), 1)

    def test_cli_expected_negative_case_generates_evidence_and_exits_normally(self):
        result = sample_result()
        result["metrics"]["success"] = False
        with patch("grasp_demo.cli.run_episode", return_value=result) as run, patch("grasp_demo.cli.write_report", return_value=Path("report.html")), redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--case", "no-target"]), 0)
            run.assert_called_once_with(seed=7, case="no-target", record=True)


if __name__ == "__main__":
    unittest.main()

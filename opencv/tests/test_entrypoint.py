"""验证迁入 Smart-Community 后的入口、样例资源和标定默认路径。"""

import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from opencv.src import calibration, cli


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class EntryPointTests(unittest.TestCase):
    def run_module(self, *arguments):
        return subprocess.run([sys.executable, "-m", "opencv", *arguments],
                              cwd=REPOSITORY_ROOT, capture_output=True, text=True,
                              encoding="utf-8", env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                              timeout=30)

    def test_module_help_from_repository_root(self):
        result = self.run_module("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("python -m opencv", result.stdout)
        self.assertIn("calibrate", result.stdout)

    def test_calibration_default_is_inside_opencv_configs(self):
        with patch.object(sys, "argv", ["opencv", "calibrate", "opencv/photo/clear3.png"]), \
                patch.object(calibration, "calibrate", return_value={"ticks": [{}, {}]}) as calibrate, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(), 0)
        self.assertEqual(calibrate.call_args.args[1], cli.MODULE_ROOT / "configs/meter.json")
        self.assertEqual(calibrate.call_args.args[2], list(range(-30, 51, 10)))

    def test_bundled_read_command_and_report(self):
        with tempfile.TemporaryDirectory() as folder:
            result = self.run_module("read", "opencv/photo/clear3.png", "--config",
                                     "opencv/configs/competition.json", "--output", folder)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "ok", report)
            self.assertIsInstance(report["rounded"], int)
            self.assertEqual(Path(report["image"]), REPOSITORY_ROOT / "opencv/photo/clear3.png")
            self.assertTrue((Path(folder) / "result.json").exists())
            self.assertTrue((Path(folder) / "debug.png").exists())

    def test_demo_uses_module_photo_and_geometry_config(self):
        with tempfile.TemporaryDirectory() as folder:
            result = self.run_module("demo", "--output", folder)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report["status"], "needs_calibration", report)
            self.assertIsNone(report["rounded"])
            self.assertEqual(Path(report["image"]), REPOSITORY_ROOT / "opencv/photo/clear3.png")
            self.assertEqual(Path(report["config"]), cli.MODULE_ROOT / "configs/competition_geometry.json")


if __name__ == "__main__":
    unittest.main()

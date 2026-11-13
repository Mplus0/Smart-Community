"""输出规则测试：按图片名保存、重复运行更新、演示独立；全部使用临时目录。"""

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from opencv.src import cli
from opencv.src.core import write_image
from .test_reader import fixture


class OutputTests(unittest.TestCase):
    def test_group_by_image_and_replace_latest_result(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config_path = root / "meter.json"
            image, config, _ = fixture(23)
            config_path.write_text(json.dumps(config), encoding="utf-8")
            output_root = root / "outputs"
            with patch.object(cli, "OUTPUTS_ROOT", output_root), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                for name, value in (("clear3", 23), ("clear2", 23), ("clear3", 24)):
                    image, _, _ = fixture(value)
                    image_path = root / (name + ".png")
                    write_image(image_path, image)
                    self.assertEqual(cli.process(image_path, config_path), 0)
            self.assertEqual({p.name for p in output_root.iterdir()}, {"read"})
            self.assertEqual({p.name for p in (output_root / "read").iterdir()}, {"clear2", "clear3"})
            latest = output_root / "read/clear3"
            self.assertEqual({p.name for p in latest.iterdir()}, {"debug.png", "result.json"})
            report = json.loads((latest / "result.json").read_text(encoding="utf-8"))
            self.assertEqual(report["rounded"], 24)
            self.assertEqual(report["config"], str(config_path.resolve()))
            self.assertEqual(report["image"], str((root / "clear3.png").resolve()))
            self.assertIn("processed_at_utc", report)

    def test_explicit_output_keeps_custom_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            image, config, _ = fixture(23)
            image_path = root / "input.png"
            config_path = root / "meter.json"
            write_image(image_path, image)
            config_path.write_text(json.dumps(config), encoding="utf-8")
            target = root / "comparison"
            with patch.object(cli, "OUTPUTS_ROOT", root / "outputs"), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(cli.process(image_path, config_path, target), 0)
            self.assertTrue((target / "result.json").exists())
            self.assertFalse((root / "outputs").exists())

    def test_demo_uses_its_own_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            output_root = Path(folder) / "outputs"
            with patch.object(cli, "OUTPUTS_ROOT", output_root), \
                    patch.object(cli.sys, "argv", ["opencv", "demo"]), \
                    patch.object(cli, "process", return_value=0) as process:
                self.assertEqual(cli.main(), 0)
            self.assertEqual(process.call_args.args[2], output_root / "demo")
            self.assertFalse(output_root.exists())


if __name__ == "__main__":
    unittest.main()

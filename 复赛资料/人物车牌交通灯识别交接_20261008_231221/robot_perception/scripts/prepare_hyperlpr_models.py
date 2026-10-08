#!/usr/bin/env python3
"""Seed HyperLPR3 0.1.3's model cache from the handoff, without a download.

Run once in the receiving inference environment, before importing hyperlpr3.
Existing identical files are reused; conflicting files are never overwritten.
"""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path


MODEL_VERSION = "20230229"
REQUIRED = {
    "y5fu_320x_sim.onnx", "y5fu_640x_sim.onnx",
    "rpv3_mdict_160_r3.onnx", "litemodel_cls_96x_r1.onnx",
}


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def verify(path, entry):
    if not path.is_file():
        raise FileNotFoundError(str(path))
    if path.stat().st_size != entry["bytes"] or digest(path) != entry["sha256"]:
        raise ValueError(f"Model size/hash mismatch: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True,
                        help="The package's models/hyperlpr3 directory")
    args = parser.parse_args()
    installed = importlib.metadata.version("hyperlpr3")
    if installed != "0.1.3":
        raise RuntimeError(f"This model cache layout targets hyperlpr3 0.1.3, found {installed}")
    if os.name != "posix" or not os.environ.get("HOME"):
        raise RuntimeError("Run this preparation script in Linux/WSL with HOME set")
    source = Path(args.source).expanduser().resolve()
    manifest = json.loads((source / "models_manifest.json").read_text(encoding="utf-8"))
    if manifest["model_version"] != MODEL_VERSION:
        raise ValueError("Unexpected model version")
    entries = manifest["models"]
    if len(entries) != len(REQUIRED) or {item["name"] for item in entries} != REQUIRED:
        raise ValueError("Expected exactly the four bundled ONNX models")
    cache = Path(os.environ["HOME"]) / ".hyperlpr3" / MODEL_VERSION / "onnx"
    # Validate every source and any existing destination before writing.
    for entry in entries:
        verify(source / MODEL_VERSION / "onnx" / entry["name"], entry)
        target = cache / entry["name"]
        if target.exists():
            verify(target, entry)
    cache.mkdir(parents=True, exist_ok=True)
    for entry in entries:
        target = cache / entry["name"]
        if target.exists():
            print("Reused:", target)
            continue
        data = (source / MODEL_VERSION / "onnx" / entry["name"]).read_bytes()
        # Exclusive creation protects any cache that appeared after validation.
        with target.open("xb") as stream:
            stream.write(data)
        verify(target, entry)
        print("Installed:", target)
    print("HyperLPR3 local models ready:", cache.parent)
    print("No network request or model inference was performed.")


if __name__ == "__main__":
    main()

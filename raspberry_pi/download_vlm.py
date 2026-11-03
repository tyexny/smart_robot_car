#!/usr/bin/env python3
"""Download the official quantized LFM2.5-VL-450M and vision projector."""

from __future__ import annotations

import argparse
from pathlib import Path

REPOSITORY = "LiquidAI/LFM2.5-VL-450M-GGUF"
FILES = (
    "LFM2.5-VL-450M-Q4_K_M.gguf",
    "mmproj-LFM2.5-VL-450m-Q8_0.gguf",
    "LICENSE",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path,
                        default=Path(__file__).resolve().parent / "models" / "lfm2.5-vl-450m")
    args = parser.parse_args()
    try:
        from huggingface_hub import hf_hub_download
    except ImportError as error:
        raise SystemExit("Install first: python -m pip install huggingface_hub") from error
    args.directory.mkdir(parents=True, exist_ok=True)
    for filename in FILES:
        path = hf_hub_download(repo_id=REPOSITORY, filename=filename,
                               local_dir=args.directory)
        print(f"Ready: {path}")


if __name__ == "__main__":
    main()

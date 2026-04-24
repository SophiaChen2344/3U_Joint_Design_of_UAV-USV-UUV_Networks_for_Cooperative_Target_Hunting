from __future__ import annotations

from pathlib import Path


def ensure_output_dirs(root: str | Path = "outputs") -> None:
    root_path = Path(root)
    for child in ("figures", "logs", "checkpoints"):
        (root_path / child).mkdir(parents=True, exist_ok=True)

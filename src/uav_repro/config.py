from __future__ import annotations

import json
from pathlib import Path
from typing import Any

def load_json(path: str | Path) -> dict[str, Any]:
    """Load a JSON configuration file into a dictionary."""
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)

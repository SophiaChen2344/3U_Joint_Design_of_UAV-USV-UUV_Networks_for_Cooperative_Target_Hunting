from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uav_repro.config import load_json


def main() -> None:
    cfg = load_json(ROOT / "configs" / "sim_main.json")
    print("Topology switching scaffold loaded.")
    print(f"Dwell time: {cfg['dwell_time']}")
    print("Next step: wire manual topology schedules before RL training.")


if __name__ == "__main__":
    main()

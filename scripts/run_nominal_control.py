from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uav_repro.config import load_json
from uav_repro.utils.logger import ensure_output_dirs


def main() -> None:
    ensure_output_dirs()
    cfg = load_json(ROOT / "configs" / "sim_main.json")
    print("Nominal control scaffold loaded.")
    print(f"Experiment: {cfg['experiment_name']}")
    print("Next step: implement FormationEnv and fixed-topology control loop.")


if __name__ == "__main__":
    main()

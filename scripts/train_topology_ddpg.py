from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from uav_repro.config import load_json


def main() -> None:
    cfg = load_json(ROOT / "configs" / "ddpg.json")
    print("DDPG training scaffold loaded.")
    print(f"Episodes: {cfg['episodes']}")
    print("Next step: implement Actor, Critic, and DDPGAgent.update().")


if __name__ == "__main__":
    main()

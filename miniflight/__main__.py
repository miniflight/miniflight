"""Miniflight command line."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

from miniflight.replay import print_replay


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(prog="miniflight")
    commands = parser.add_subparsers(dest="command", required=True)
    replay_parser = commands.add_parser("replay", help="replay one flight record")
    replay_parser.add_argument("path", type=Path)
    arguments = parser.parse_args(argv)

    if arguments.command == "replay":
        print_replay(arguments.path)


if __name__ == "__main__":
    main()

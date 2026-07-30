#!/usr/bin/env python3
"""Count logical Python lines in the Miniflight clean core.

Inspired by tinygrad/sz.py. Comments and docstrings do not count. Use
MAX_LINE_COUNT to make a size budget fail in local checks or CI.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import token
import tokenize
from typing import Iterable


COUNTED_TOKENS = frozenset({token.NAME, token.NUMBER, token.OP, token.STRING})
SOURCE_DIRECTORIES = (
    Path("common"),
    Path("config"),
    Path("examples"),
    Path("miniflight"),
    Path("sim"),
    Path("target"),
)
IGNORED_PARTS = frozenset({".git", "__pycache__", "node_modules", "vendor"})


def is_docstring(item: tokenize.TokenInfo) -> bool:
    """Return true only for a standalone triple-quoted docstring token."""
    return item.type == token.STRING and item.line.lstrip().startswith(('"""', "'''"))


def logical_lines(path: Path) -> tuple[int, float]:
    """Return logical source lines and token density for one Python file."""
    with tokenize.open(path) as source:
        tokens = [
            item
            for item in tokenize.generate_tokens(source.readline)
            if item.type in COUNTED_TOKENS and not is_docstring(item)
        ]
    lines = {line for item in tokens for line in range(item.start[0], item.end[0] + 1)}
    count = len(lines)
    density = 0.0 if count == 0 else len(tokens) / count
    return count, density


def python_files(paths: Iterable[Path]) -> list[Path]:
    """Return tracked source candidates in deterministic path order."""
    files: list[Path] = []
    for path in paths:
        if path.is_file() and path.suffix == ".py":
            files.append(path)
        elif path.is_dir():
            files.extend(candidate for candidate in path.rglob("*.py") if not IGNORED_PARTS.intersection(candidate.parts))
    return sorted(set(files))


def report(paths: Iterable[Path]) -> int:
    """Print per-file logical lines and return the total."""
    rows = [(path, *logical_lines(path)) for path in python_files(paths)]
    for path, count, density in rows:
        print(f"{count:5d}  {density:4.1f}  {path.as_posix()}")
    total = sum(count for _, count, _ in rows)
    print(f"\ntotal lines: {total}")
    return total


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="Python files or directories to count")
    args = parser.parse_args()
    paths = args.paths or (*SOURCE_DIRECTORIES, *sorted(Path(".").glob("*.py")))
    total = report(paths)
    maximum = int(os.environ.get("MAX_LINE_COUNT", "-1"))
    if maximum >= 0 and total > maximum:
        raise SystemExit(f"OVER {maximum} LINES: {total}")


if __name__ == "__main__":
    main()

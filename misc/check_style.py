#!/usr/bin/env python3
import subprocess
import sys
from typing import List

PKGS = ["slamcore_utils", "tests"]


def process(f):  # type: ignore
    def wrapper(*args, **kargs):
        proc_args = f(*args, **kargs)
        p = subprocess.run(proc_args)
        rc = p.returncode
        if rc == 0:
            print(f"[{f.__name__}] OK")
        else:
            print(f"[{f.__name__}] Checks failed with error code {rc}")

        return rc

    return wrapper


@process
def black(pkg: str, check: bool, verbose: bool = False) -> List[str]:
    args = [
        "black",
        pkg,
    ]
    if check:
        args.append("--check")
    if verbose:
        args.append("-v")

    return args


@process
def isort(pkg: str, check: bool, verbose: bool = False) -> List[str]:
    args = [
        "isort",
        pkg,
    ]
    if check:
        args.append("--check")
    if verbose:
        args.append("-v")

    return args


if __name__ == "__main__":
    check = "--check-only" in sys.argv
    verbose = "-v" in sys.argv or "--verbose" in sys.argv
    for pkg in PKGS:
        print(f"\nProcessing {pkg}...\n")
        black(pkg, check=check, verbose=verbose)
        isort(pkg, check=check, verbose=verbose)

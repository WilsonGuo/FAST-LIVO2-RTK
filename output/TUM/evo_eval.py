#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FAST-LIVO2-RTK Evo Evaluation Script

Usage:
    python3 evo_eval.py

or

    python3 evo_eval.py \
        --ref opt_trajectory_before.txt \
        --est opt_trajectory_after.txt
"""

import argparse
import os
import subprocess
import sys


def run_cmd(cmd):
    print("=" * 70)
    print("Running:")
    print(" ".join(cmd))
    print("=" * 70)

    result = subprocess.run(cmd)

    if result.returncode != 0:
        print("\n❌ evo command failed.")
        sys.exit(result.returncode)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--ref",
        default="opt_trajectory_before.txt",
        help="Reference trajectory"
    )

    parser.add_argument(
        "--est",
        default="opt_trajectory_after.txt",
        help="Estimated trajectory"
    )

    parser.add_argument(
        "--save_plot",
        action="store_true",
        help="Save plot as png"
    )

    args = parser.parse_args()

    if not os.path.exists(args.ref):
        print(f"Reference trajectory not found:\n{args.ref}")
        return

    if not os.path.exists(args.est):
        print(f"Estimated trajectory not found:\n{args.est}")
        return

    cmd = [
        "evo_ape",
        "tum",
        args.ref,
        args.est,
        "-a",
        "-p"
    ]

    if args.save_plot:
        cmd += [
            "--save_plot",
            "ape_result.pdf"
        ]

    run_cmd(cmd)


if __name__ == "__main__":
    main()
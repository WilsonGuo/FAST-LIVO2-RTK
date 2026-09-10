#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FAST-LIVO2-RTK Evo Evaluation Script

默认功能：
1. 计算 opt_trajectory_before.txt 与 opt_trajectory_after.txt 的 APE
2. 绘制 Grass01.txt、opt_trajectory_before.txt、opt_trajectory_after.txt 三条轨迹对比图

默认路径会自动按 FAST-LIVO2-RTK 工程目录寻找：
    src/FAST-LIVO2-RTK/Log/result/Grass01.txt
    src/FAST-LIVO2-RTK/output/TUM/opt_trajectory_before.txt
    src/FAST-LIVO2-RTK/output/TUM/opt_trajectory_after.txt

常用运行：
    python3 analyze_trajectory_diff.py

"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


PACKAGE_NAME = "FAST-LIVO2-RTK"


def run_cmd(cmd: list[str]) -> None:
    print("=" * 70)
    print("Running:")
    print(" ".join(cmd))
    print("=" * 70)

    result = subprocess.run(cmd)

    if result.returncode != 0:
        print("\n❌ evo command failed.")
        sys.exit(result.returncode)


def find_project_root(project_root_arg: str | None = None) -> Path:
    """
    自动寻找 FAST-LIVO2-RTK 工程根目录。

    优先级：
    1. 用户通过 --project_root 指定
    2. 从当前脚本路径向上查找 FAST-LIVO2-RTK
    3. 从当前工作目录向上查找 FAST-LIVO2-RTK
    """
    if project_root_arg:
        return Path(project_root_arg).expanduser().resolve()

    search_starts = [Path(__file__).resolve(), Path.cwd().resolve()]

    for start in search_starts:
        candidates = [start] + list(start.parents)
        for p in candidates:
            if p.name == PACKAGE_NAME:
                return p
            child = p / "src" / PACKAGE_NAME
            if child.exists():
                return child.resolve()

    # 兜底：如果脚本就在 output/TUM 里，通常上两级就是工程根目录
    script_path = Path(__file__).resolve()
    try:
        if script_path.parents[1].name == "output":
            return script_path.parents[2]
    except IndexError:
        pass

    # 最后兜底：使用当前目录
    return Path.cwd().resolve()


def resolve_path(path_arg: str | None, default_path: Path) -> Path:
    """如果用户传入路径则使用用户路径，否则使用默认路径。"""
    if path_arg:
        return Path(path_arg).expanduser().resolve()
    return default_path.resolve()


def check_file(path: Path, name: str) -> None:
    if not path.exists():
        print(f"❌ {name} not found:\n{path}")
        sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate FAST-LIVO2-RTK trajectories with evo."
    )

    parser.add_argument(
        "--project_root",
        default=None,
        help="FAST-LIVO2-RTK project root, e.g. /home/wilson/fast_livo_ws/src/FAST-LIVO2-RTK",
    )

    parser.add_argument(
        "--ref",
        default=None,
        help="Reference trajectory. Default: <project_root>/output/TUM/opt_trajectory_before.txt",
    )

    parser.add_argument(
        "--est",
        default=None,
        help="Estimated trajectory. Default: <project_root>/output/TUM/opt_trajectory_after.txt",
    )

    parser.add_argument(
        "--grass",
        default=None,
        help="FAST-LIVO2 original trajectory. Default: <project_root>/Log/result/Grass01.txt",
    )

    parser.add_argument(
        "--plot_mode",
        default="xy",
        choices=["xy", "xz", "yz"],
        help="Trajectory plot mode for evo_traj. Default: xy",
    )

    parser.add_argument(
        "--save_plot",
        action="store_true",
        help="Save APE plot as ape_result.pdf",
    )

    parser.add_argument(
        "--save_results",
        action="store_true",
        help="Save APE result as ape_result.zip",
    )

    parser.add_argument(
        "--no_ape",
        action="store_true",
        help="Do not run evo_ape.",
    )

    parser.add_argument(
        "--no_traj",
        action="store_true",
        help="Do not run evo_traj three-trajectory comparison.",
    )

    args = parser.parse_args()

    project_root = find_project_root(args.project_root)

    ref_path = resolve_path(
        args.ref,
        project_root / "output" / "TUM" / "opt_trajectory_before.txt",
    )
    est_path = resolve_path(
        args.est,
        project_root / "output" / "TUM" / "opt_trajectory_after.txt",
    )
    grass_path = resolve_path(
        args.grass,
        project_root / "Log" / "result" / "Grass01.txt",
    )

    print("Project root:", project_root)
    print("Grass01 trajectory:", grass_path)
    print("Reference trajectory:", ref_path)
    print("Estimated trajectory:", est_path)
    print()

    check_file(ref_path, "Reference trajectory")
    check_file(est_path, "Estimated trajectory")

    if not args.no_traj:
        check_file(grass_path, "Grass01 trajectory")

    if args.no_ape and args.no_traj:
        print("Nothing to run: both --no_ape and --no_traj are set.")
        return

    if not args.no_ape:
        ape_cmd = [
            "evo_ape",
            "tum",
            str(ref_path),
            str(est_path),
            "-a",
            "-p",
        ]

        if args.save_plot:
            ape_cmd += ["--save_plot", "ape_result.pdf"]

        if args.save_results:
            ape_cmd += ["--save_results", "ape_result.zip"]

        run_cmd(ape_cmd)

    if not args.no_traj:
        traj_cmd = [
            "evo_traj",
            "tum",
            str(grass_path),
            str(ref_path),
            str(est_path),
            "--ref",
            str(ref_path),
            "-a",
            "-p",
            "--plot_mode",
            args.plot_mode,
        ]

        run_cmd(traj_cmd)


if __name__ == "__main__":
    main()

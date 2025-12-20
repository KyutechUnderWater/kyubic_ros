import argparse
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

matplotlib.use("qt5agg")
pd.set_option("future.no_silent_downcasting", True)
np.set_printoptions(threshold=1000, precision=3, suppress=True)

QUADRUPLE_SIZE: int = 4


def num_segments(point_chain: np.ndarray) -> int:
    # There is 1 segment per 4 points, so we must subtract 3 from the number of points
    return len(point_chain) - (QUADRUPLE_SIZE - 1)


def flatten(list_of_lists) -> list:
    # E.g. mapping [[1, 2], [3], [4, 5]] to  [1, 2, 3, 4, 5]
    return [elem for lst in list_of_lists for elem in lst]


def catmull_rom_spline(
    P0: tuple,
    P1: tuple,
    P2: tuple,
    P3: tuple,
    num_points: int,
    alpha: float = 0.5,
):
    """
    Compute the points in the spline segment
    :param P0, P1, P2, and P3: The (x,y) point pairs that define the Catmull-Rom spline
    :param num_points: The number of points to include in the resulting curve segment
    :param alpha: 0.5 for the centripetal spline, 0.0 for the uniform spline, 1.0 for the chordal spline.
    :return: The points
    """

    # Calculate t0 to t4. Then only calculate points between P1 and P2.
    # Reshape linspace so that we can multiply by the points P0 to P3
    # and get a point for each value of t.
    def tj(ti: float, pi: tuple, pj: tuple) -> float:
        xi, yi, zi = pi
        xj, yj, zj = pj
        dx, dy, dz = xj - xi, yj - yi, zj - zi
        l = (dx**2 + dy**2 + dz**2) ** 0.5
        return ti + l**alpha

    t0: float = 0.0
    t1: float = tj(t0, P0, P1)
    t2: float = tj(t1, P1, P2)
    t3: float = tj(t2, P2, P3)
    t = np.linspace(t1, t2, num_points).reshape(num_points, 1)

    A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1
    A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2
    A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3
    B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
    B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
    points = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2
    return points


def catmull_rom_chain(points: np.ndarray, num_points: int) -> list:
    """
    Calculate Catmull-Rom for a sequence of initial points and return the combined curve.
    :param points: Base points from which the quadruples for the algorithm are taken
    :param num_points: The number of points to include in each curve segment
    :return: The chain of all points (points of all segments)
    """
    point_quadruples = (  # Prepare function inputs
        (points[idx_segment_start + d] for d in range(QUADRUPLE_SIZE))
        for idx_segment_start in range(num_segments(points))
    )
    all_splines = [catmull_rom_spline(*next(point_quadruples), num_points)]
    all_splines += [
        np.delete(catmull_rom_spline(*pq, num_points), 0, axis=0) for pq in point_quadruples
    ]
    return flatten(all_splines)


@dataclass
class Parameter:
    checkpoint_end_row: int | None = None
    catmull_rom: Literal[0, 1] = 0
    catmull_end_row: int = 0
    catmull_density: int = 0
    catmull_min_distance: float = 0
    catmull_orient_LERP: Literal[0, 1] = 0


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "csv_path",
        type=Path,
        help="csv file path with n x 3(row x col) checkpoint data",
    )
    parser.add_argument(
        "-s",
        "--only_show",
        action="store_true",
        help="Only show checkpoint. Don't completion",
    )
    parser.add_argument(
        "-p",
        "--print_default",
        action="store_true",
        help="out default csv file",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    param_header = [
        "parameter_label",
        "value",
    ]
    ckpts_header = [
        "x",
        "y",
        "z",
        "z_mode",
        "roll",
        "yaw",
    ]
    catmull_header = [
        "catmull_x",
        "catmull_y",
        "catmull_z",
        "catmull_z_mode",
        "catmull_roll",
        "catmull_yaw",
    ]
    header = [
        *param_header,
        *ckpts_header,
        "blank",
        *catmull_header,
    ]

    points: list = []
    if args.print_default:
        param = Parameter()
        df_default = pd.DataFrame(columns=header).round(4)
        df_default["parameter_label"] = asdict(param).keys()
        df_default["value"] = asdict(param).values()
        df_default.to_csv(args.csv_path, index=False)
        exit()

    data = pd.read_csv(args.csv_path).replace({np.nan: None})  # Red points

    # Extract parameter
    _param = data[data["parameter_label"].notnull()][["parameter_label", "value"]]
    param = Parameter(**dict(zip(_param["parameter_label"].to_list(), _param["value"].to_list())))

    # Extract checkpoints
    checkpoints = data[["x", "y", "z", "z_mode", "roll", "yaw"]]
    checkpoints_list = []
    if param.checkpoint_end_row is not None:
        checkpoints = checkpoints.iloc[0 : int(param.checkpoint_end_row)]
        assert checkpoints.notnull().all().all(), "Error: checkpoints include nan."
    else:
        idx = 0
        for point in checkpoints.values:
            if all(point == None):
                break
            assert not any(point == None), f"Error: {point} include nan. {idx + 1}"
            idx += 1
        checkpoints = checkpoints.iloc[0:idx]
        param.checkpoint_end_row = idx

    # Check for duplicates
    for i in range(1, len(checkpoints)):
        assert not checkpoints.iloc[i - 1].equals(checkpoints.iloc[i]), (
            f"Error: Duplicate found in line {i + 1}"
        )

    # Divide by z mode and only rotation behavior
    pose = checkpoints[["x", "y", "z"]]
    mode = checkpoints["z_mode"]
    start = 0
    for end in range(1, len(mode)):
        if mode.iloc[end - 1] != mode[end] or ((pose.iloc[end - 1] - pose.iloc[end]) == 0).all():
            checkpoints_list.append(checkpoints.iloc[start:end])
            start = end
    checkpoints_list.append(checkpoints.iloc[start : len(mode)])

    # 各区間の分割数
    if param.catmull_density is not None:
        NUM_POINTS: int = int(param.catmull_density)
    else:
        NUM_POINTS: int = 100  # Density of blue chain points between two red points

    points_list = []
    for ckpts in checkpoints_list:
        # 座標が１つの場合は，曲線生成せずに追加
        if len(ckpts) == 1:
            points_list.append(ckpts.values)
            continue

        nd_ckpts = ckpts[["x", "y", "z"]].values.astype(float)

        # 始点と終点の追加(スタートとゴール時の方向ベクトル)
        nd_ckpts = np.insert(nd_ckpts, 0, 2 * nd_ckpts[0] - nd_ckpts[1], axis=0)
        nd_ckpts = np.insert(
            nd_ckpts,
            len(nd_ckpts),
            2 * nd_ckpts[-1] - nd_ckpts[-2],
            axis=0,
        )

        chain_points = catmull_rom_chain(nd_ckpts, NUM_POINTS)
        assert (
            len(chain_points) == num_segments(nd_ckpts) * (NUM_POINTS - 1) + 1
        )  # 400 blue points for this example

        # z_mode, rollとyawを追加
        nd_orients = ckpts[["z_mode", "roll", "yaw"]].values.astype(float)
        chain_orients = np.vstack(
            [
                nd_orients[i - 1]
                + (nd_orients[i] - nd_orients[i - 1])
                * (j / (NUM_POINTS - 2) if param.catmull_orient_LERP == 1 else 1)
                for i in range(1, len(nd_orients))
                for j in range(NUM_POINTS - 1)
            ]
        )
        chain_orients = np.vstack([nd_orients[0], chain_orients])
        chain_pose = np.hstack([chain_points, chain_orients])

        # point間距離の最小値をもとに，ダウンサンプリング
        def _calc_distance(points: np.ndarray) -> np.ndarray:
            return np.linalg.norm(
                np.array(points)[1:] - np.array(points)[:-1],
                ord=2,
                axis=1,
            )

        dist = 0
        points = [chain_pose[0]]
        for i, d in enumerate(_calc_distance(chain_pose[:, :3]), 1):
            dist += d
            if (
                dist > param.catmull_min_distance
                or i % (NUM_POINTS - 1) == 0
                or i == len(chain_pose) - 1
            ):
                print(i)
                print(chain_pose[i])
                dist = 0
                points.append(chain_pose[i])
        points_list.append(points)

        ax = plt.figure().add_subplot(projection="3d")
        ax.plot(*zip(*np.array(points)[:, :3]), c="blue")
        ax.plot(*zip(*nd_ckpts.tolist()), c="red", linestyle="none", marker="o")
        ax.set_aspect("equal")
        plt.show()

    if not args.only_show:
        points_list = flatten(points_list)
        param.catmull_rom = 1
        param.catmull_end_row = len(points_list)
        param_dict = asdict(param)

        df_param = pd.DataFrame(param_dict.items(), columns=param_header)
        df_checkpoints = checkpoints
        df_blank = pd.DataFrame(columns=["blank"])
        df_points = pd.DataFrame(points_list, columns=catmull_header).round(4)

        df = pd.concat([df_param, df_checkpoints, df_blank, df_points], axis=1)
        df.to_csv(args.csv_path.with_name(f"{args.csv_path.stem}_generated.csv"), index=False)
        print(df)


if __name__ == "__main__":
    main()

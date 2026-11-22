"""Estimate friction from saved Wheelleg bench traces."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import linear_regression, mean, median


COLUMNS = ("elapsed_ms", "position_rad", "speed_rad_s", "torque_nm")
THIGH_COLUMN = "thigh_position_rad"
NOMINAL_WHEEL_INERTIA = 6.95e-4 + 15.8**2 * 1.2e-5


def read_trace(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        if path.suffix.lower() == ".json":
            rows = json.load(stream)
            if isinstance(rows, dict):
                rows = rows["data"]
        else:
            rows = list(csv.DictReader(stream))

    samples = []
    for row in rows:
        fields = COLUMNS + (THIGH_COLUMN,) if row.get(THIGH_COLUMN) not in (None, "") else COLUMNS
        sample = {name: float(row[name]) for name in fields}
        if not all(map(math.isfinite, sample.values())):
            raise ValueError(f"{path}: nonfinite sample")
        if samples and sample["elapsed_ms"] <= samples[-1]["elapsed_ms"]:
            raise ValueError(f"{path}: elapsed_ms must strictly increase")
        samples.append(sample)
    if not samples:
        raise ValueError(f"{path}: empty trace")
    return samples


def time_fit(rows, column, sign=1):
    time = [row["elapsed_ms"] / 1000 for row in rows]
    values = [sign * row[column] for row in rows]
    slope, intercept = linear_regression(time, values)
    center = mean(values)
    variance = sum((value - center) ** 2 for value in values)
    residual = sum((value - intercept - slope * tick) ** 2 for tick, value in zip(time, values))
    return slope, (1 - residual / variance if variance else None)


def joint_window(path, window, direction):
    start, end = window
    rows = [row for row in read_trace(path) if start <= row["elapsed_ms"] <= end]
    if len(rows) < 3:
        raise ValueError(f"{path}: joint window requires at least three samples")
    if any(direction * row["speed_rad_s"] <= 0 for row in rows):
        raise ValueError(f"{path}: velocity must keep the requested sign throughout the window")

    duration_s = (rows[-1]["elapsed_ms"] - rows[0]["elapsed_ms"]) / 1000
    speed = (rows[-1]["position_rad"] - rows[0]["position_rad"]) / duration_s
    acceleration, _ = time_fit(rows, "speed_rad_s")
    result = {
        "time_ms": [rows[0]["elapsed_ms"], rows[-1]["elapsed_ms"]],
        "points": len(rows),
        "mean_position_rad": mean(row["position_rad"] for row in rows),
        "average_speed_rad_s": speed,
        "acceleration_rad_s2": acceleration,
        "median_torque_nm": median(row["torque_nm"] for row in rows),
    }
    if all(THIGH_COLUMN in row for row in rows):
        thigh = [row[THIGH_COLUMN] for row in rows]
        result.update(mean_thigh_position_rad=mean(thigh), thigh_travel_rad=max(thigh) - min(thigh))
    return result


def joint_pair(args):
    positive = joint_window(args.positive, args.pos_ms, 1)
    negative = joint_window(args.negative, args.neg_ms, -1)
    torque_pos = positive["median_torque_nm"]
    torque_neg = negative["median_torque_nm"]
    return {
        "positive": positive,
        "negative": negative,
        "motion_friction_nm": (torque_pos - torque_neg) / 2,
        "load_bias_nm": (torque_pos + torque_neg) / 2,
    }


def wheel_coast(args):
    start, end = args.time_ms
    min_speed, max_speed = args.speed_rad_s
    if not (0 <= start < end and 0 < min_speed < max_speed and args.inertia > 0):
        raise ValueError("invalid time window, speed band or inertia")
    rows = read_trace(args.trace)
    direction = 1 if args.direction == "positive" else -1
    segments = []
    current = []
    for row in rows:
        speed = direction * row["speed_rad_s"]
        time = row["elapsed_ms"]
        if not (start <= time <= end and min_speed <= speed <= max_speed):
            current = []
            continue
        if not current:
            segments.append(current)
        current.append(row)

    window = max(segments, key=len, default=[])
    if len(window) < 3:
        raise ValueError("no contiguous coast window with at least three samples")
    slope, r_squared = time_fit(window, "speed_rad_s", direction)
    if slope >= 0:
        raise ValueError("selected coast window is not decelerating")
    inertia_torque = -args.inertia * slope
    residual_torque = mean(direction * row["torque_nm"] for row in window)
    return {
        "time_ms": [window[0]["elapsed_ms"], window[-1]["elapsed_ms"]],
        "points": len(window),
        "deceleration_rad_s2": -slope,
        "r_squared": r_squared,
        "inertia_kg_m2": args.inertia,
        "inertia_torque_nm": inertia_torque,
        "residual_torque_nm": residual_torque,
        "corrected_friction_nm": inertia_torque + residual_torque,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True)
    joint = commands.add_parser("joint", help="compare two matched constant-speed windows")
    joint.add_argument("positive", type=Path)
    joint.add_argument("negative", type=Path)
    joint.add_argument("--pos-ms", type=float, nargs=2, required=True)
    joint.add_argument("--neg-ms", type=float, nargs=2, required=True)
    joint.set_defaults(calculate=joint_pair)

    wheel = commands.add_parser("wheel", help="fit a wheel coast-down window")
    wheel.add_argument("trace", type=Path)
    wheel.add_argument("--direction", choices=("positive", "negative"), required=True)
    wheel.add_argument("--time-ms", type=float, nargs=2, required=True)
    wheel.add_argument("--speed-rad-s", type=float, nargs=2, default=(1.0, 6.0))
    wheel.add_argument("--inertia", type=float, default=NOMINAL_WHEEL_INERTIA, help="wheel-axis inertia in kg m²")
    wheel.set_defaults(calculate=wheel_coast)

    args = parser.parse_args()
    try:
        result = args.calculate(args)
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

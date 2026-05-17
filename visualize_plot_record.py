#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path


# Edit these defaults when you want to run the script without CLI arguments.
RECORD_FILE = "records/plot_latest.jsonl"
TIME_KEY = "t"

# Leave empty to plot all numeric fields found in the record.
# Nested fields can be selected with dot paths, for example: "target.x".
CURVES = [
    # "gimbal_yaw",
    # "cmd_yaw",
]


def flatten_numeric(value, prefix=""):
    out = {}

    if isinstance(value, bool):
        out[prefix] = float(value)
    elif isinstance(value, (int, float)) and math.isfinite(value):
        out[prefix] = float(value)
    elif isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten_numeric(child, child_prefix))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            child_prefix = f"{prefix}.{index}" if prefix else str(index)
            out.update(flatten_numeric(child, child_prefix))

    return out


def load_jsonl(path):
    rows = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"Skip malformed line {line_no}: {exc}")
    return rows


def choose_curves(flat_rows, time_key, configured_curves):
    if configured_curves:
        return configured_curves

    keys = set()
    for row in flat_rows:
        keys.update(row.keys())
    return sorted(key for key in keys if key != time_key)


def resolve_record_path(record_file):
    path = Path(record_file)
    if path.exists():
        return path

    if record_file == RECORD_FILE:
        candidates = sorted(
            Path("records").glob("*.jsonl"),
            key=lambda item: item.stat().st_mtime,
            reverse=True,
        )
        if candidates:
            return candidates[0]

    return path


def main():
    parser = argparse.ArgumentParser(description="Visualize PlotRecord JSONL time-series data.")
    parser.add_argument("record_file", nargs="?", default=RECORD_FILE)
    parser.add_argument("--time", default=TIME_KEY, help="Time key in JSON records.")
    parser.add_argument(
        "--curves",
        nargs="+",
        default=CURVES,
        help="Curve keys to plot. Dot paths are supported.",
    )
    parser.add_argument("--save", default="", help="Save figure to this path instead of only showing it.")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "matplotlib is required for plotting. Install it with: python3 -m pip install matplotlib"
        ) from exc

    path = resolve_record_path(args.record_file)
    if not path.exists():
        raise FileNotFoundError(f"Record file not found: {path}")

    rows = load_jsonl(path)
    if not rows:
        raise RuntimeError(f"No valid JSON records found in {path}")

    flat_rows = [flatten_numeric(row) for row in rows]
    curves = choose_curves(flat_rows, args.time, args.curves)
    if not curves:
        raise RuntimeError("No numeric curves found to plot.")

    x = []
    for index, row in enumerate(flat_rows):
        x.append(row.get(args.time, float(index)))

    plotted = 0
    for curve in curves:
        y = [row.get(curve, float("nan")) for row in flat_rows]
        if all(math.isnan(value) for value in y):
            print(f"Skip missing/non-numeric curve: {curve}")
            continue
        plt.plot(x, y, label=curve)
        plotted += 1

    if plotted == 0:
        raise RuntimeError("None of the selected curves contain numeric data.")

    plt.xlabel(args.time if args.time in flat_rows[0] else "sample")
    plt.ylabel("value")
    plt.title(str(path))
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=180)
    plt.show()


if __name__ == "__main__":
    main()

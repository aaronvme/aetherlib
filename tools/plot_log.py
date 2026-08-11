#!/usr/bin/env python3
"""Plot a tuning CSV pulled off the SD card.

    python tools/plot_log.py lift00.csv
    python tools/plot_log.py chas00.csv --cols x y targetX targetY
    python tools/plot_log.py chas00.csv --xy          # path in field coords

Needs matplotlib: pip install matplotlib
"""

import argparse
import csv
import os
import sys


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        sys.exit(f"{path}: empty")
    header = [h.strip() for h in rows[0]]
    cols = {name: [] for name in header}
    for row in rows[1:]:
        if len(row) != len(header):
            continue  # partial last line if the brain lost power mid-write
        for name, value in zip(header, row):
            try:
                cols[name].append(float(value))
            except ValueError:
                cols[name].append(float("nan"))
    return header, cols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--cols", nargs="*", help="columns to plot (default: all but t_ms)")
    ap.add_argument("--xy", action="store_true", help="plot y against x instead of vs time")
    args = ap.parse_args()

    header, cols = read_csv(args.csv)

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib not installed: pip install matplotlib")

    title = os.path.basename(args.csv)

    if args.xy:
        if "x" not in cols or "y" not in cols:
            sys.exit("--xy needs x and y columns")
        plt.figure()
        plt.plot(cols["x"], cols["y"], label="actual")
        if "targetX" in cols and "targetY" in cols:
            plt.plot(cols["targetX"], cols["targetY"], "--", label="target")
        plt.gca().set_aspect("equal")
        plt.xlabel("x (in)")
        plt.ylabel("y (in)")
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.show()
        return

    t = cols.get("t_ms", list(range(len(next(iter(cols.values()))))))
    names = args.cols or [h for h in header if h != "t_ms"]
    missing = [n for n in names if n not in cols]
    if missing:
        sys.exit(f"no such column(s): {', '.join(missing)}\navailable: {', '.join(header)}")

    plt.figure()
    for name in names:
        plt.plot(t, cols[name], label=name)
    plt.xlabel("t (ms)")
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()

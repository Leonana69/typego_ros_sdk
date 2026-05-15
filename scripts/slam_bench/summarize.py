#!/usr/bin/env python3
"""Build summary.md from a run directory.

Handles both layouts:
  single run :  <run_dir>/<backend>/{latency.csv,compute.csv,drift.json}
  --repeats N:  <run_dir>/<backend>/run_<i>/{...}

With multiple runs per backend, each metric is reported as `mean ± std` and a
per-run drift breakdown is appended (drift is the noisiest metric, so the
spread across runs is what matters).
"""

import argparse
import csv
import glob
import json
import math
import os
import sys
from statistics import mean, pstdev


# ---------------------------------------------------------------- loaders

def percentile(sorted_vals, q):
    if not sorted_vals:
        return None
    n = len(sorted_vals)
    idx = min(n - 1, max(0, int(round(q * (n - 1)))))
    return sorted_vals[idx]


def load_latency(path):
    if not os.path.exists(path):
        return {}
    lags, arrivals = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                lags.append(float(row["lag_ms"]))
                arrivals.append(float(row["arrival_t_mono"]))
            except (KeyError, ValueError):
                continue
    if not lags:
        return {}
    s = sorted(lags)
    if len(arrivals) >= 2 and arrivals[-1] > arrivals[0]:
        rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
    else:
        rate = None
    return {
        "n_msgs": len(lags),
        "rate_hz": rate,
        "lag_p50_ms": percentile(s, 0.50),
        "lag_p95_ms": percentile(s, 0.95),
        "lag_p99_ms": percentile(s, 0.99),
        "lag_max_ms": s[-1],
    }


def load_compute(path):
    if not os.path.exists(path):
        return {}
    cpu, rss, th = [], [], []
    n_procs = 0
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                cpu.append(float(row["total_cpu_pct"]))
                rss.append(float(row["total_rss_mb"]))
                th.append(int(row["total_threads"]))
                n_procs = max(n_procs, int(row["n_procs"]))
            except (KeyError, ValueError):
                continue
    if not cpu:
        return {}
    return {
        "cpu_mean_pct": mean(cpu),
        "cpu_peak_pct": max(cpu),
        "rss_mean_mb": mean(rss),
        "rss_peak_mb": max(rss),
        "threads_peak": max(th),
        "n_procs": n_procs,
    }


def load_drift(path):
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            d = json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}
    return {
        "drift_m": d.get("translation_m"),
        "yaw_deg": d.get("yaw_deg"),
        "path_length_m": d.get("path_length_m"),
        "n_poses": d.get("n_poses"),
        "duration_s": d.get("duration_s"),
    }


def collect_run(run_dir):
    """All scalar metrics for one run directory."""
    m = {}
    m.update(load_latency(os.path.join(run_dir, "latency.csv")))
    m.update(load_compute(os.path.join(run_dir, "compute.csv")))
    m.update(load_drift(os.path.join(run_dir, "drift.json")))
    return m


def find_run_dirs(run_dir, backend):
    """Return the list of per-run dirs for a backend (handles --repeats)."""
    be_dir = os.path.join(run_dir, backend)
    if not os.path.isdir(be_dir):
        return []
    runs = sorted(glob.glob(os.path.join(be_dir, "run_*")))
    runs = [r for r in runs if os.path.isdir(r)]
    if runs:
        return runs
    return [be_dir]  # single-run layout


# ---------------------------------------------------------------- format

# (label, metric key, format spec, decimals for std)
METRICS = [
    ("Output rate (Hz, mean)", "rate_hz", "{:.2f}"),
    ("Messages received", "n_msgs", "{:.0f}"),
    ("Lag p50 (ms)", "lag_p50_ms", "{:.1f}"),
    ("Lag p95 (ms)", "lag_p95_ms", "{:.1f}"),
    ("Lag p99 (ms)", "lag_p99_ms", "{:.1f}"),
    ("Lag max (ms)", "lag_max_ms", "{:.1f}"),
    ("CPU % mean (sum across procs)", "cpu_mean_pct", "{:.1f}"),
    ("CPU % peak (sum)", "cpu_peak_pct", "{:.1f}"),
    ("RSS MB mean (sum)", "rss_mean_mb", "{:.0f}"),
    ("RSS MB peak (sum)", "rss_peak_mb", "{:.0f}"),
    ("Threads peak (sum)", "threads_peak", "{:.0f}"),
    ("Processes detected", "n_procs", "{:.0f}"),
    ("Final-pose drift (m)", "drift_m", "{:.3f}"),
    ("Yaw drift (deg)", "yaw_deg", "{:.2f}"),
    ("Path length (m)", "path_length_m", "{:.2f}"),
    ("Trajectory poses", "n_poses", "{:.0f}"),
    ("Trajectory duration (s)", "duration_s", "{:.1f}"),
]


def cell(values, fmt):
    """Render a metric across runs as 'mean' or 'mean ± std'."""
    vals = [v for v in values if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return "n/a"
    mu = mean(vals)
    if len(vals) == 1:
        return fmt.format(mu)
    sd = pstdev(vals)
    return f"{fmt.format(mu)} ± {fmt.format(sd)}"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run_dir")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    backends = [be for be in ("arise", "lightning")
                if os.path.isdir(os.path.join(args.run_dir, be))]
    if not backends:
        print(f"error: no arise/ or lightning/ dir under {args.run_dir}", file=sys.stderr)
        sys.exit(1)

    # backend -> list of per-run metric dicts
    data = {}
    n_runs = {}
    for be in backends:
        run_dirs = find_run_dirs(args.run_dir, be)
        data[be] = [collect_run(rd) for rd in run_dirs]
        n_runs[be] = len(run_dirs)

    a = backends[0]
    b = backends[1] if len(backends) > 1 else None

    lines = []
    name = os.path.basename(os.path.abspath(args.run_dir))
    lines.append(f"# SLAM Bench Summary — `{name}`\n")
    lines.append(f"Run directory: `{os.path.abspath(args.run_dir)}`\n")
    runs_desc = ", ".join(f"{be}: {n_runs[be]} run(s)" for be in backends)
    lines.append(f"Repeats — {runs_desc}.\n")
    if os.path.exists(os.path.join(args.run_dir, "bag_info.txt")):
        lines.append("Input bag: see `bag_info.txt`.\n")
    lines.append("")

    multi = any(n > 1 for n in n_runs.values())
    hdr_note = " (mean ± std across runs)" if multi else ""
    lines.append(f"| Metric{hdr_note} | {a} | {b or 'B'} |")
    lines.append("|---|---|---|")
    for label, key, fmt in METRICS:
        av = [run.get(key) for run in data.get(a, [])]
        bv = [run.get(key) for run in data.get(b, [])] if b else []
        lines.append(f"| {label} | {cell(av, fmt)} | {cell(bv, fmt)} |")

    # Per-run drift breakdown — the metric most worth seeing run-by-run.
    if multi:
        lines.append("")
        lines.append("## Per-run drift")
        lines.append("")
        lines.append(f"| Run | {a} drift (m) | {a} yaw (°) | "
                     f"{b or 'B'} drift (m) | {b or 'B'} yaw (°) |")
        lines.append("|---|---|---|---|---|")
        max_runs = max(n_runs.values())
        for i in range(max_runs):
            def g(be, key):
                runs = data.get(be, [])
                if i < len(runs) and runs[i].get(key) is not None:
                    return runs[i][key]
                return None
            ad, ay = g(a, "drift_m"), g(a, "yaw_deg")
            bd, by = (g(b, "drift_m"), g(b, "yaw_deg")) if b else (None, None)
            def f(v, spec):
                return spec.format(v) if v is not None else "n/a"
            lines.append(
                f"| {i+1} | {f(ad,'{:.3f}')} | {f(ay,'{:.2f}')} | "
                f"{f(bd,'{:.3f}')} | {f(by,'{:.2f}')} |"
            )

    lines += [
        "",
        "Notes:",
        "- CPU% is summed across the backend's processes (ARISE: 3, Lightning: 1) "
        "and may exceed 100% — that's total CPU time across cores.",
        "- Drift is **closed-loop drift**: it assumes the bag starts and ends at the "
        "same physical pose. If it doesn't, drift is not interpretable as accuracy.",
        "- Latency uses sim time (bag `/clock`); a slightly-negative lag is `/clock` "
        "quantization, not a real negative latency.",
        "- Loop closure is enabled in both backends by default.",
        "- Trajectories are saved per run as `trajectory.tum` "
        "(`evo_ape tum a.tum b.tum -as` for cross-SLAM consistency).",
    ]

    out = args.out or os.path.join(args.run_dir, "summary.md")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()

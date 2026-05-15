#!/usr/bin/env python3
"""Sample CPU%, RSS, thread count of one SLAM backend's processes.

ARISE runs as three executables (feature_extraction_node, laser_mapping_node,
imu_preintegration_node); Lightning runs as one (run_slam_online). We discover
PIDs by matching against `psutil.Process.name()` (the executable basename),
falling back to a cmdline scan for any process where the basename was elided
by ROS / launch.

CSV columns:
  t_mono, total_cpu_pct, total_rss_mb, total_threads, n_procs, per_proc_json

`cpu_percent` reports total CPU time across all cores per process (can exceed
100% on multi-threaded procs). We sum across the backend's procs to get a
single "this is how much compute the SLAM is using" number.
"""

import argparse
import json
import signal
import sys
import time

import psutil


BACKEND_EXES = {
    "arise": (
        "feature_extraction_node",
        "laser_mapping_node",
        "imu_preintegration_node",
    ),
    "lightning": ("run_slam_online",),
}


def find_procs(targets):
    """Return list of psutil.Process whose name OR cmdline[0] matches a target."""
    found = []
    for p in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            info = p.info
            name = info.get("name") or ""
            cmd = info.get("cmdline") or []
            if name in targets:
                found.append(p)
                continue
            # Fall back to argv[0] basename — ROS sometimes wraps the exe.
            if cmd:
                argv0 = cmd[0].rsplit("/", 1)[-1]
                if argv0 in targets:
                    found.append(p)
                    continue
                # Some launchers exec via "python3 .../foo"; also check argv tail.
                for tok in cmd[1:]:
                    base = tok.rsplit("/", 1)[-1]
                    if base in targets:
                        found.append(p)
                        break
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return found


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--backend", required=True, choices=sorted(BACKEND_EXES))
    p.add_argument("--out", required=True)
    p.add_argument("--interval", type=float, default=0.5,
                   help="seconds between samples")
    p.add_argument("--rescan-every", type=float, default=5.0,
                   help="seconds between PID rescans")
    args = p.parse_args()

    targets = BACKEND_EXES[args.backend]
    procs = find_procs(targets)
    # Prime cpu_percent (first call after construct returns 0.0).
    for pr in procs:
        try:
            pr.cpu_percent(None)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    fh = open(args.out, "w", buffering=1)
    fh.write("t_mono,total_cpu_pct,total_rss_mb,total_threads,n_procs,per_proc_json\n")

    stop = {"v": False}
    def _stop(_s, _f):
        stop["v"] = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    last_rescan = time.monotonic()
    print(f"compute_probe: backend={args.backend} targets={targets} "
          f"initial_pids={[pr.pid for pr in procs]}", flush=True)

    while not stop["v"]:
        if time.monotonic() - last_rescan > args.rescan_every:
            current_pids = {pr.pid for pr in procs if pr.is_running()}
            fresh = find_procs(targets)
            for pr in fresh:
                if pr.pid not in current_pids:
                    try:
                        pr.cpu_percent(None)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                    procs.append(pr)
            procs = [pr for pr in procs if pr.is_running()]
            last_rescan = time.monotonic()

        total_cpu = 0.0
        total_rss = 0.0
        total_threads = 0
        per_proc = []
        for pr in list(procs):
            try:
                c = pr.cpu_percent(None)
                m = pr.memory_info().rss / 1e6
                th = pr.num_threads()
                per_proc.append({
                    "pid": pr.pid,
                    "name": pr.name(),
                    "cpu_pct": c,
                    "rss_mb": m,
                    "threads": th,
                })
                total_cpu += c
                total_rss += m
                total_threads += th
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                procs = [x for x in procs if x.pid != pr.pid]

        fh.write(
            f"{time.monotonic():.3f},{total_cpu:.2f},{total_rss:.2f},"
            f"{total_threads},{len(per_proc)},"
            f"{json.dumps(per_proc, separators=(',', ':'))}\n"
        )
        time.sleep(args.interval)

    fh.flush()
    fh.close()
    sys.exit(0)


if __name__ == "__main__":
    main()

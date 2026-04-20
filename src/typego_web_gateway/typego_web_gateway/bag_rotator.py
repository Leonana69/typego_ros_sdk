"""Always-on rosbag2 recorder with size-bounded retention.

Runs ``ros2 bag record`` as a subprocess with ``--max-bag-duration`` so
rosbag2 itself rolls over to a new file every N seconds. A housekeeping
thread prunes the oldest complete bag directories to keep retention within
``retain`` files.
"""
from __future__ import annotations

import logging
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import List, Optional

LOG = logging.getLogger('typego_web_gateway.bag_rotator')

DEFAULT_TOPICS = [
    '/scan',
    '/state_estimation',
    '/tf',
    '/tf_static',
    '/cmd_vel',
    '/map',
    '/rosout',
    '/diagnostics',
    '/navigate_to_pose/_action/status',
]


class BagRotator:
    def __init__(
        self,
        bag_dir: str,
        topics: Optional[List[str]] = None,
        chunk_seconds: int = 600,
        retain: int = 6,
    ):
        self._bag_dir = Path(bag_dir)
        self._topics = topics or list(DEFAULT_TOPICS)
        self._chunk_seconds = chunk_seconds
        self._retain = retain
        self._proc: Optional[subprocess.Popen] = None
        self._stop = threading.Event()
        self._janitor: Optional[threading.Thread] = None

    @property
    def bag_dir(self) -> Path:
        return self._bag_dir

    def start(self) -> None:
        if self._proc is not None:
            return
        try:
            self._bag_dir.mkdir(parents=True, exist_ok=True)
        except (PermissionError, OSError) as exc:
            LOG.warning(
                'bag_rotator disabled: cannot create %s (%s). '
                'Pass --bag-dir <writable path> or --no-bag to suppress.',
                self._bag_dir, exc,
            )
            return
        prefix = self._bag_dir / time.strftime('typego_%Y%m%d_%H%M%S')
        cmd = [
            'ros2', 'bag', 'record',
            '--output', str(prefix),
            '--max-bag-duration', str(self._chunk_seconds),
            '--storage', 'mcap',
        ] + list(self._topics)
        LOG.info('bag_rotator: %s', shlex.join(cmd))
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError as exc:
            LOG.warning('bag_rotator disabled: %s', exc)
            self._proc = None
            return
        self._janitor = threading.Thread(
            target=self._run_janitor, daemon=True, name='bag_rotator_janitor',
        )
        self._janitor.start()

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self._proc = None
        if self._janitor is not None:
            self._janitor.join(timeout=2.0)
            self._janitor = None

    def list_bags(self) -> List[Path]:
        """Return completed bag directories ordered newest-first."""
        if not self._bag_dir.is_dir():
            return []
        bags: List[Path] = []
        for entry in self._bag_dir.iterdir():
            if not entry.is_dir():
                continue
            if not (entry / 'metadata.yaml').is_file():
                continue  # still being written
            bags.append(entry)
        bags.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return bags

    def bags_covering(self, minutes: int) -> List[Path]:
        """Bags whose mtime is within the last ``minutes`` minutes."""
        cutoff = time.time() - minutes * 60
        return [p for p in self.list_bags() if p.stat().st_mtime >= cutoff]

    def _run_janitor(self) -> None:
        while not self._stop.wait(30.0):
            try:
                self._prune()
            except Exception as exc:  # pragma: no cover
                LOG.warning('bag_rotator prune failed: %s', exc)

    def _prune(self) -> None:
        bags = self.list_bags()
        for old in bags[self._retain:]:
            LOG.info('bag_rotator: removing %s', old)
            shutil.rmtree(old, ignore_errors=True)

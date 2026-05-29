#!/usr/bin/env python3
"""Interactive rviz launcher with a log pane + quit input.

Asks for autonomy mode (base/full), and for full mode asks which layer's
config to load (mirrors MODE=0/1/2 in src/autonomy/real_robot.sh):

    0  base_autonomy         vehicle_simulator.rviz
    1  FAR route planner     far_planner/default.rviz
    2  PCD grid planner      pcd_grid_planner/default.rviz
    3  exploration (TARE)    tare_planner_ground.rviz

Base mode runs plain rviz (no -d). Once launched, rviz's stdout/stderr
streams into the log pane; type `q` + Enter to quit, or `m` + Enter to go
back to the menu and pick a different config.

Install:   pip install textual
Run:       make rviz   (or: python3 scripts/rviz_launch.py)
"""
from __future__ import annotations

import asyncio
import os
import signal
import sys
from pathlib import Path
from typing import Optional

try:
    from rich.text import Text
    from textual.app import App, ComposeResult
    from textual.containers import Vertical
    from textual.widgets import Input, RichLog, Static
except ImportError:
    sys.stderr.write(
        "textual is not installed.\n"
        "  pip install textual\n"
    )
    sys.exit(1)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
AUTONOMY = PROJECT_ROOT / "src/autonomy"

FULL_CONFIGS = [
    ("0", "vehicle simulator (base layer)",
     AUTONOMY / "base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz"),
    ("1", "FAR route planner",
     AUTONOMY / "route_planner/far_planner/rviz/default.rviz"),
    ("2", "PCD grid planner",
     AUTONOMY / "route_planner/pcd_grid_planner/rviz/default.rviz"),
    ("3", "TARE exploration planner",
     AUTONOMY / "exploration_planner/tare_planner/rviz/tare_planner_ground.rviz"),
]


def _ns_remaps() -> list[str]:
    robot_id = os.environ.get("ROBOT_ID", "").strip()
    if not robot_id:
        return []
    ns = f"/robot{robot_id}"
    return [
        "--ros-args",
        "-r", f"/tf:={ns}/tf",
        "-r", f"/tf_static:={ns}/tf_static",
        "-r", f"/goal_pose:={ns}/goal_pose",
    ]


class RvizConsole(App):
    CSS = """
    Screen { layout: vertical; }
    RichLog {
        height: 1fr;
        border: solid $accent;
        background: $panel;
        padding: 0 1;
    }
    #status {
        height: 1;
        padding: 0 1;
        background: $boost;
        color: $text;
    }
    Input { height: 3; }
    """

    def __init__(self) -> None:
        super().__init__()
        self.state: str = "mode"
        self.mode: Optional[str] = None
        self.rviz_proc: Optional[asyncio.subprocess.Process] = None

    # ---------- Layout ----------

    def compose(self) -> ComposeResult:
        self.log_pane = RichLog(highlight=False, markup=True, wrap=True)
        self.status = Static("", id="status")
        self.prompt = Input(placeholder="")
        yield Vertical(self.log_pane, self.status, self.prompt)

    def on_mount(self) -> None:
        self.title = "Typego rviz"
        self.prompt.focus()
        self._show_mode_menu()

    # ---------- Menu rendering ----------

    def _banner(self, text: str) -> None:
        self.log_pane.clear()
        self.log_pane.write(f"[bold cyan]{text}[/bold cyan]")
        self.log_pane.write("")

    def _show_mode_menu(self) -> None:
        self.state = "mode"
        self.mode = None
        self._banner("Typego rviz launcher")
        self.log_pane.write("Choose autonomy mode:")
        self.log_pane.write("  [bold]1[/bold]  base  (slam_toolbox)")
        self.log_pane.write("  [bold]2[/bold]  full  (ARISE SLAM)")
        self.log_pane.write("")
        self._set_status("Select mode")
        self.prompt.placeholder = "Type 1 or 2, then Enter  (q to quit)"
        self.prompt.value = ""

    def _show_sub_menu(self) -> None:
        self.state = "sub"
        self._banner("Mode: full")
        self.log_pane.write("Rviz config:")
        for key, label, _ in FULL_CONFIGS:
            self.log_pane.write(f"  [bold]{key}[/bold]  {label}")
        self.log_pane.write("")
        self.log_pane.write("  [bold]b[/bold]  back       [bold]q[/bold]  quit")
        self._set_status("Select rviz config")
        self.prompt.placeholder = "Type 0, 1, 2, or 3 (or b / q), then Enter"
        self.prompt.value = ""

    def _set_status(self, text: str) -> None:
        self.status.update(text)

    # ---------- Input dispatch ----------

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        raw = event.value.strip().lower()
        self.prompt.value = ""
        handlers = {
            "mode": self._handle_mode,
            "sub": self._handle_sub,
            "running": self._handle_running,
        }
        handler = handlers.get(self.state)
        if handler is not None:
            await handler(raw)

    async def _handle_mode(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
            return
        if raw == "1":
            self.mode = "base"
            await self._start_rviz(rviz_args=[])
        elif raw == "2":
            self.mode = "full"
            self._show_sub_menu()
        else:
            self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")

    async def _handle_sub(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
            return
        if raw in ("b", "back"):
            self._show_mode_menu()
            return
        for key, label, path in FULL_CONFIGS:
            if key == raw:
                if not path.is_file():
                    self.log_pane.write(f"[red]Config not found: {path}[/red]")
                    return
                self.log_pane.write(f"[green]=> {label}: {path}[/green]")
                await self._start_rviz(rviz_args=["-d", str(path)])
                return
        self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")

    async def _handle_running(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
        elif raw in ("m", "menu"):
            await self._kill_rviz()
            self._show_mode_menu()
        elif raw:
            self.log_pane.write(f"[red]Unknown command: {raw!r}[/red]")

    # ---------- rviz subprocess ----------

    async def _start_rviz(self, rviz_args: list[str]) -> None:
        self.state = "running"
        cmd = ["ros2", "run", "rviz2", "rviz2", *rviz_args, *_ns_remaps()]
        self._banner(f"Launching: {' '.join(cmd)}")
        self._set_status(f"Running rviz [{self.mode}]")
        self.prompt.placeholder = "[q]uit  ·  [m]enu (pick a different config)"
        try:
            self.rviz_proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,  # own process group so SIGINT covers children
            )
        except FileNotFoundError:
            self.log_pane.write("[red]ros2 not on PATH — source the ROS env first.[/red]")
            self._show_mode_menu()
            return
        self.run_worker(self._pump(self.rviz_proc), exclusive=False, name="rviz_pump")

    async def _pump(self, proc: asyncio.subprocess.Process) -> None:
        assert proc.stdout is not None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            try:
                self.log_pane.write(Text.from_ansi(line.decode(errors="replace").rstrip()))
            except Exception:
                self.log_pane.write(repr(line))
        rc = await proc.wait()
        if proc is self.rviz_proc:
            self.log_pane.write(f"[dim]-- rviz exited ({rc}) --[/dim]")
            self.rviz_proc = None
            if self.state == "running":
                self._set_status("rviz exited — [m]enu or [q]uit")
                self.prompt.placeholder = "Type q or m, then Enter"

    async def _kill_rviz(self) -> None:
        proc = self.rviz_proc
        if proc is None or proc.returncode is not None:
            self.rviz_proc = None
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            await asyncio.wait_for(proc.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            await proc.wait()
        self.rviz_proc = None

    async def _quit(self) -> None:
        await self._kill_rviz()
        self.exit()


def main() -> int:
    RvizConsole().run()
    return 0


if __name__ == "__main__":
    sys.exit(main())

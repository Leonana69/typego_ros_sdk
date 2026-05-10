#!/usr/bin/env python3
"""Interactive launcher for the Typego SDK.

A Claude-Code-style TUI: logs scroll in the top pane, a context-aware
input sits at the bottom. Walks you through the mode + map choice, spawns
ros2 launch for you, and handles save-map-and-return-to-menu in one place.

Install:   pip install textual
Run:       make console    (or: python3 scripts/sdk_console.py)
"""
from __future__ import annotations

import asyncio
import os
import signal
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO

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
MAP_RESOURCE_DIR = PROJECT_ROOT / "src/typego_sdk/resource"
SAVE_MAP_SCRIPT = PROJECT_ROOT / "scripts/save_map.py"
LOG_DIR = PROJECT_ROOT / "logs"
LAUNCH_CMD = ["ros2", "launch", "typego_sdk", "typego_bringup.launch.py"]
EMPTY_MAP = "empty_map"

# Full-autonomy sub-modes: (key, short-label, launch-arg-value).
# Mirrors typego_bringup.launch.py's full_mode mapping.
FULL_SUB_OPTIONS = [
    ("0", "plain (vehicle simulator)"),
    ("1", "FAR route planner"),
    ("2", "TARE exploration planner"),
]


def _maps_for_mode(mode: str) -> list[str]:
    """List maps valid for the given mode by looking at on-disk artefacts.

    base (slam_toolbox):  Map-<name>/<name>.posegraph
    full (ARISE SLAM):    Map-<name>/<name>.pcd
    """
    if not MAP_RESOURCE_DIR.is_dir():
        return []
    names = []
    for entry in sorted(MAP_RESOURCE_DIR.iterdir()):
        if not entry.is_dir() or not entry.name.startswith("Map-"):
            continue
        name = entry.name[len("Map-"):]
        if name == EMPTY_MAP:
            continue
        if mode == "base" and (entry / f"{name}.posegraph").is_file():
            names.append(name)
        elif mode == "full" and (entry / f"{name}.pcd").is_file():
            names.append(name)
    return names


def _safe_map_name(raw: str) -> str:
    """Strip whitespace, replace path separators + spaces with underscores."""
    cleaned = raw.strip().replace("/", "_").replace("\\", "_")
    return "_".join(cleaned.split())


class TypegoConsole(App):
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
        self.full_mode_key: str = "0"         # full-autonomy sub-mode: "0" / "1" / "2"
        self.full_mode_label: str = ""        # short label for status line
        self.map_options: list[str] = []
        self.selected_map: Optional[str] = None  # None means "build new"
        self.launch_proc: Optional[asyncio.subprocess.Process] = None
        self.log_file: Optional[TextIO] = None
        self.log_path: Optional[Path] = None

    # ---------- Layout ----------

    def compose(self) -> ComposeResult:
        self.log_pane = RichLog(highlight=False, markup=True, wrap=True)
        self.status = Static("", id="status")
        self.prompt = Input(placeholder="")
        yield Vertical(self.log_pane, self.status, self.prompt)

    def on_mount(self) -> None:
        self.title = "Typego SDK Console"
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
        self.full_mode_label = ""
        self.selected_map = None
        self._banner("Typego SDK Console")
        self.log_pane.write("Choose autonomy mode:")
        self.log_pane.write("  [bold]1[/bold]  base   (slam_toolbox)")
        self.log_pane.write("  [bold]2[/bold]  full   (ARISE SLAM)")
        self.log_pane.write("")
        self._set_status("Select mode")
        self.prompt.placeholder = "Type 1 or 2, then Enter  (q to quit)"
        self.prompt.value = ""

    def _show_full_sub_menu(self) -> None:
        self.state = "full_sub"
        self._banner("Mode: full")
        self.log_pane.write("Full-autonomy layer:")
        for key, label in FULL_SUB_OPTIONS:
            self.log_pane.write(f"  [bold]{key}[/bold]  {label}")
        self.log_pane.write("")
        self.log_pane.write("  [bold]b[/bold]  back       [bold]q[/bold]  quit")
        self._set_status("Select full-autonomy layer")
        self.prompt.placeholder = "Type 0, 1, or 2 (or b / q), then Enter"
        self.prompt.value = ""

    def _show_map_menu(self) -> None:
        self.state = "map"
        self.selected_map = None
        self.map_options = _maps_for_mode(self.mode or "")
        self._banner(f"Mode: {self.mode}")
        self.log_pane.write("Choose a map:")
        for i, name in enumerate(self.map_options, start=1):
            self.log_pane.write(f"  [bold]{i}[/bold]  {name}")
        new_idx = len(self.map_options) + 1
        self.log_pane.write(f"  [bold]{new_idx}[/bold]  [green]new map[/green]  (start mapping from empty)")
        self.log_pane.write("")
        self.log_pane.write("  [bold]b[/bold]  back      [bold]q[/bold]  quit")
        self._set_status(f"Mode: {self.mode}  —  select map")
        upper = max(new_idx, 1)
        self.prompt.placeholder = f"Type 1-{upper} (or b / q), then Enter"
        self.prompt.value = ""

    def _mode_label(self) -> str:
        if self.mode == "full" and self.full_mode_label:
            return f"full · {self.full_mode_label}"
        return self.mode or ""

    def _update_running_status(self) -> None:
        label = self._mode_label()
        if self.selected_map is None:
            self._set_status(f"Running [{label}]  ·  building new map")
            self.prompt.placeholder = "[s]ave and return  ·  [m]enu  ·  [q]uit"
        else:
            self._set_status(f"Running [{label}]  ·  map: {self.selected_map}")
            self.prompt.placeholder = "[m]enu  ·  [q]uit"

    def _set_status(self, text: str) -> None:
        self.status.update(text)

    # ---------- Input dispatch ----------

    async def on_input_submitted(self, event: Input.Submitted) -> None:
        raw = event.value.strip()
        self.prompt.value = ""
        handlers = {
            "mode": self._handle_mode,
            "full_sub": self._handle_full_sub,
            "map": self._handle_map,
            "running": self._handle_running,
            "naming": self._handle_naming,
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
            self._show_map_menu()
        elif raw == "2":
            self.mode = "full"
            self._show_full_sub_menu()
        else:
            self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")

    async def _handle_full_sub(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
            return
        if raw in ("b", "back"):
            self._show_mode_menu()
            return
        for key, label in FULL_SUB_OPTIONS:
            if key == raw:
                self.full_mode_key = key
                self.full_mode_label = label
                self._show_map_menu()
                return
        self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")

    async def _handle_map(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
            return
        if raw in ("b", "back"):
            if self.mode == "full":
                self._show_full_sub_menu()
            else:
                self._show_mode_menu()
            return
        try:
            idx = int(raw)
        except ValueError:
            self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")
            return
        new_idx = len(self.map_options) + 1
        if 1 <= idx <= len(self.map_options):
            self.selected_map = self.map_options[idx - 1]
            await self._start_launch()
        elif idx == new_idx:
            self.selected_map = None
            await self._start_launch()
        else:
            self.log_pane.write(f"[red]Out of range: {raw!r}[/red]")

    async def _handle_running(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
        elif raw in ("m", "menu"):
            await self._kill_launch()
            self._show_mode_menu()
        elif raw == "s" and self.selected_map is None:
            self.state = "naming"
            self._set_status("Save map — enter a name (or 'cancel')")
            self.prompt.placeholder = "Enter map name — it's saved once you press Enter"
        elif raw:
            self.log_pane.write(f"[red]Unknown command: {raw!r}[/red]")

    async def _handle_naming(self, raw: str) -> None:
        if raw in ("", "cancel"):
            self.state = "running"
            self._update_running_status()
            return
        name = _safe_map_name(raw)
        if not name:
            self.log_pane.write("[red]Map name can't be empty.[/red]")
            return
        self.log_pane.write(f"[bold yellow]=> Saving map as '{name}'…[/bold yellow]")
        rc = await self._run_save_map(name)
        if rc == 0:
            self.log_pane.write(
                f"[bold green]=> Saved. Shutting down SLAM and returning to menu.[/bold green]"
            )
            await self._kill_launch()
            self._show_mode_menu()
        else:
            self.log_pane.write(f"[red]Save failed with exit code {rc}. SLAM is still running.[/red]")
            self.state = "running"
            self._update_running_status()

    # ---------- Launch subprocess ----------

    def _open_log_file(self, cmd: list[str]) -> None:
        try:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            self.log_path = LOG_DIR / f"console-{datetime.now():%Y%m%d-%H%M%S}.log"
            self.log_file = self.log_path.open("w", buffering=1, encoding="utf-8", errors="replace")
            self.log_file.write(f"# typego console log — {datetime.now().isoformat(timespec='seconds')}\n")
            self.log_file.write(f"# cmd: {' '.join(cmd)}\n")
        except OSError as exc:
            self.log_file = None
            self.log_path = None
            self.log_pane.write(f"[yellow]Could not open log file: {exc}[/yellow]")

    def _close_log_file(self) -> None:
        if self.log_file is not None:
            try:
                self.log_file.close()
            except OSError:
                pass
        self.log_file = None
        self.log_path = None

    def _tee_line(self, raw: bytes) -> None:
        text = raw.decode(errors="replace").rstrip("\r\n")
        if self.log_file is not None:
            try:
                self.log_file.write(text + "\n")
            except OSError:
                pass
        try:
            self.log_pane.write(Text.from_ansi(text))
        except Exception:
            self.log_pane.write(repr(raw))

    async def _start_launch(self) -> None:
        self.state = "running"
        map_arg = EMPTY_MAP if self.selected_map is None else self.selected_map
        cmd = list(LAUNCH_CMD) + [
            f"autonomy_type:={self.mode}",
            f"slam_map_name:={map_arg}",
        ]
        if self.mode == "full":
            cmd.append(f"full_mode:={self.full_mode_key}")
        self._banner(f"Launching: {' '.join(cmd)}")
        self._open_log_file(cmd)
        if self.log_path is not None:
            rel = self.log_path.relative_to(PROJECT_ROOT) if self.log_path.is_relative_to(PROJECT_ROOT) else self.log_path
            self.log_pane.write(f"[dim]Logs tee'd to {rel}  ·  Hold Shift while dragging to copy text.[/dim]")
        self._update_running_status()
        try:
            self.launch_proc = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,  # own process group so we can SIGINT the whole tree
            )
        except FileNotFoundError:
            self.log_pane.write("[red]ros2 not on PATH — source the ROS env before running.[/red]")
            self._close_log_file()
            self._show_mode_menu()
            return
        self.run_worker(self._pump(self.launch_proc), exclusive=False, name="launch_pump")

    async def _pump(self, proc: asyncio.subprocess.Process) -> None:
        assert proc.stdout is not None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            self._tee_line(line)
        rc = await proc.wait()
        if proc is self.launch_proc:
            self.log_pane.write(f"[dim]-- ros2 launch exited ({rc}) --[/dim]")
            self.launch_proc = None
            self._close_log_file()
            # If it died on its own, drop back to the menu rather than leaving
            # the user staring at a frozen-looking screen.
            if self.state == "running":
                self._show_mode_menu()

    async def _kill_launch(self) -> None:
        proc = self.launch_proc
        if proc is None or proc.returncode is not None:
            self.launch_proc = None
            self._close_log_file()
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            await asyncio.wait_for(proc.wait(), timeout=8.0)
        except asyncio.TimeoutError:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            await proc.wait()
        self.launch_proc = None
        self._close_log_file()

    async def _run_save_map(self, name: str) -> int:
        cmd = ["python3", str(SAVE_MAP_SCRIPT), name, "--mode", self.mode or ""]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        assert proc.stdout is not None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            self._tee_line(line)
        return await proc.wait()

    async def _quit(self) -> None:
        await self._kill_launch()
        self.exit()


def main() -> int:
    TypegoConsole().run()
    return 0


if __name__ == "__main__":
    sys.exit(main())

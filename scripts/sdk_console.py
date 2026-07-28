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
from typing import NamedTuple, Optional, TextIO

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

# 3D-autonomy planner options. PCD grid shares full_mode=1 with FAR and is
# selected by passing route_planner_backend:=pcd_grid.
class FullSubOption(NamedTuple):
    key: str
    label: str
    full_mode: str
    extra_args: tuple[str, ...] = ()
    requires_existing_map: bool = False


FULL_SUB_OPTIONS = [
    # "local planner only" rather than "base": `base` was the old name of
    # the 2D stack, so reusing it here for "no route planner" would give
    # one word two meanings.
    FullSubOption("0", "local planner only (no route planner)", "0"),
    FullSubOption("1", "FAR route planner", "1", ("route_planner_backend:=far",)),
    FullSubOption(
        "2",
        "PCD grid planner",
        "1",
        ("route_planner_backend:=pcd_grid",),
        # Builds its 2D map live from /registered_scan — no saved map required,
        # so the "new map" option must stay available.
        False,
    ),
    FullSubOption("3", "TARE exploration planner", "2"),
]

# Operator-facing names for the two autonomy stacks. These keys are the
# actual autonomy_type launch-arg values, matching autonomy.type in
# robot.yaml. `base`/`full` named nothing; 2d/3d names the axis that
# really differs -- the sensing and state-estimation layer.
MODE_MENU = {
    "2d": "2D-autonomy   (SLAM Toolbox + Nav2)",
    "3d": "3D-autonomy   (LIO + terrain analysis + local planner)",
}
MODE_SHORT = {"2d": "2D-autonomy", "3d": "3D-autonomy"}


def _mode_name(mode: str) -> str:
    """Short display name; falls back to the raw value if unrecognised."""
    return MODE_SHORT.get(mode, mode or "")


def _maps_for_mode(mode: str) -> list[str]:
    """List maps valid for the given mode by looking at on-disk artefacts.

    2d (slam_toolbox):  Map-<name>/<name>.posegraph
    3d (ARISE SLAM):    Map-<name>/<name>.pcd
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
        if mode == "2d" and (entry / f"{name}.posegraph").is_file():
            names.append(name)
        elif mode == "3d" and (entry / f"{name}.pcd").is_file():
            names.append(name)
    return names


def _safe_map_name(raw: str) -> str:
    """Strip whitespace, replace path separators + spaces with underscores."""
    cleaned = raw.strip().replace("/", "_").replace("\\", "_")
    return "_".join(cleaned.split())


class FollowLog(RichLog):
    """A RichLog that tails new output only while you're parked at the bottom.

    RichLog's built-in ``auto_scroll`` yanks the viewport to the end on every
    write, so you can't read back through history while logs keep streaming in.
    Here a ``follow`` flag drives the scroll instead: it stays armed while you're
    at the bottom and disarms the moment you scroll up, re-arming once you scroll
    back down.

    The flag is updated from ``watch_scroll_y`` — i.e. from a *settled* scroll
    position — rather than sampled per-write. That matters because RichLog scrolls
    via ``scroll_end(immediate=False)``, which defers the actual scroll to the next
    refresh while ``max_scroll_y`` grows synchronously; a burst of writes between
    two refreshes would otherwise read a half-updated position and get stuck part
    way up.
    """

    def __init__(self, **kwargs) -> None:
        # We manage following ourselves, so disable RichLog's own auto-scroll.
        super().__init__(auto_scroll=False, **kwargs)
        self._follow = True

    def write(self, content, *args, **kwargs):  # type: ignore[override]
        # RichLog.write is (content, width, expand, shrink, scroll_end, animate),
        # so scroll_end is the 4th positional after content. RichLog's own
        # deferred-render replay (on first resize) passes it positionally — honour
        # that and only inject our follow default when the caller left it unset.
        if len(args) < 4 and kwargs.get("scroll_end") is None:
            kwargs["scroll_end"] = self._follow
        return super().write(content, *args, **kwargs)

    def watch_scroll_y(self, old_value: float, new_value: float) -> None:
        super().watch_scroll_y(old_value, new_value)
        # Re-arm following when parked at the bottom; disarm when scrolled up.
        self._follow = self.is_vertical_scroll_end


class TypegoConsole(App):
    # The Input keeps focus, so plain arrows/Home/End belong to line editing.
    # PageUp/PageDown aren't used by a single-line Input, so they bubble up here
    # and let you page through log history without a mouse (e.g. over SSH).
    BINDINGS = [
        ("pageup", "log_scroll('up')", "Scroll log up"),
        ("pagedown", "log_scroll('down')", "Scroll log down"),
        ("ctrl+end", "log_bottom", "Jump to latest"),
    ]

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
        self.full_mode_key: str = "0"         # typego_bringup full_mode launch value
        self.full_mode_label: str = ""        # short label for status line
        self.full_extra_launch_args: list[str] = []
        self.full_requires_existing_map: bool = False
        self.map_options: list[str] = []
        self.selected_map: Optional[str] = None  # None means "build new"
        self.launch_proc: Optional[asyncio.subprocess.Process] = None
        self.log_file: Optional[TextIO] = None
        self.log_path: Optional[Path] = None

    # ---------- Layout ----------

    def compose(self) -> ComposeResult:
        self.log_pane = FollowLog(highlight=False, markup=True, wrap=True)
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
        self.full_extra_launch_args = []
        self.full_requires_existing_map = False
        self.selected_map = None
        self._banner("Typego SDK Console")
        self.log_pane.write("Choose autonomy stack:")
        self.log_pane.write(f"  [bold]1[/bold]  {MODE_MENU['2d']}")
        self.log_pane.write(f"  [bold]2[/bold]  {MODE_MENU['3d']}")
        self.log_pane.write("")
        self._set_status("Select autonomy stack")
        self.prompt.placeholder = "Type 1 or 2, then Enter  (q to quit)"
        self.prompt.value = ""

    def _show_full_sub_menu(self) -> None:
        self.state = "full_sub"
        self._banner(f"Stack: {MODE_SHORT['3d']}")
        self.log_pane.write("3D-autonomy planner:")
        for option in FULL_SUB_OPTIONS:
            self.log_pane.write(f"  [bold]{option.key}[/bold]  {option.label}")
        self.log_pane.write("")
        self.log_pane.write("  [bold]b[/bold]  back       [bold]q[/bold]  quit")
        self._set_status("Select 3D-autonomy planner")
        self.prompt.placeholder = "Type 0, 1, 2, or 3 (or b / q), then Enter"
        self.prompt.value = ""

    def _show_map_menu(self) -> None:
        self.state = "map"
        self.selected_map = None
        self.map_options = _maps_for_mode(self.mode or "")
        self._banner(f"Stack: {_mode_name(self.mode or '')}")
        self.log_pane.write("Choose a map:")
        for i, name in enumerate(self.map_options, start=1):
            self.log_pane.write(f"  [bold]{i}[/bold]  {name}")
        show_new_map = not (self.mode == "3d" and self.full_requires_existing_map)
        new_idx = len(self.map_options) + 1
        if show_new_map:
            self.log_pane.write(f"  [bold]{new_idx}[/bold]  [green]new map[/green]  (start mapping from empty)")
        elif not self.map_options:
            self.log_pane.write("[yellow]No saved PCD maps found. This planner needs an existing 3D-autonomy map.[/yellow]")
        self.log_pane.write("")
        self.log_pane.write("  [bold]b[/bold]  back      [bold]q[/bold]  quit")
        self._set_status(f"{_mode_name(self.mode or '')}  —  select map")
        if show_new_map:
            upper = max(new_idx, 1)
            self.prompt.placeholder = f"Type 1-{upper} (or b / q), then Enter"
        elif self.map_options:
            self.prompt.placeholder = f"Type 1-{len(self.map_options)} (or b / q), then Enter"
        else:
            self.prompt.placeholder = "Type b or q, then Enter"
        self.prompt.value = ""

    def _mode_label(self) -> str:
        name = _mode_name(self.mode or "")
        if self.mode == "3d" and self.full_mode_label:
            return f"{name} · {self.full_mode_label}"
        return name

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

    # ---------- Log scrolling ----------

    def action_log_scroll(self, direction: str) -> None:
        if direction == "up":
            self.log_pane.scroll_page_up()
        else:
            self.log_pane.scroll_page_down()

    def action_log_bottom(self) -> None:
        # Re-follow: scroll_end pins us at the bottom; watch_scroll_y re-arms.
        self.log_pane.scroll_end(animate=False)

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
            self.mode = "2d"
            self._show_map_menu()
        elif raw == "2":
            self.mode = "3d"
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
        for option in FULL_SUB_OPTIONS:
            if option.key == raw:
                self.full_mode_key = option.full_mode
                self.full_mode_label = option.label
                self.full_extra_launch_args = list(option.extra_args)
                self.full_requires_existing_map = option.requires_existing_map
                self._show_map_menu()
                return
        self.log_pane.write(f"[red]Unknown option: {raw!r}[/red]")

    async def _handle_map(self, raw: str) -> None:
        if raw in ("q", "quit"):
            await self._quit()
            return
        if raw in ("b", "back"):
            if self.mode == "3d":
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
        elif idx == new_idx and not (self.mode == "3d" and self.full_requires_existing_map):
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
        if self.mode == "3d":
            cmd.append(f"full_mode:={self.full_mode_key}")
            cmd.extend(self.full_extra_launch_args)
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

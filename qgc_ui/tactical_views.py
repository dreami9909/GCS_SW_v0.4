from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk
from typing import Callable

from .components import Theme, flat_button
from .domain import MissionStore, VehicleState
from .map_widget import MapWidget
from .tactical import TacticalState, ThreatTrack


HMI = {
    "background": "#07100d",
    "panel": "#101a14",
    "panel_alt": "#16231a",
    "border": "#425746",
    "grid": "#283d30",
    "text": "#c6d6c8",
    "muted": "#718677",
    "green": "#39e36c",
    "amber": "#f3b52d",
    "red": "#ff4a42",
    "cyan": "#54b9d4",
    "blue": "#4da7e8",
}


def _section(master: tk.Misc, title: str) -> tuple[tk.Frame, tk.Frame]:
    shell = tk.Frame(
        master,
        background=HMI["panel"],
        highlightbackground=HMI["border"],
        highlightthickness=1,
    )
    header = tk.Label(
        shell,
        text=title,
        background="#1b291f",
        foreground=HMI["amber"],
        font=("Malgun Gothic", 9, "bold"),
        anchor="w",
        padx=8,
        pady=4,
    )
    header.pack(fill=tk.X)
    body = tk.Frame(shell, background=HMI["panel"])
    body.pack(fill=tk.BOTH, expand=True, padx=7, pady=6)
    return shell, body


class TacticalFlyView(tk.Frame):
    """Situational-awareness mockup with no command transport."""

    def __init__(
        self,
        master: tk.Misc,
        mission: MissionStore,
        vehicle: VehicleState,
        tactical: TacticalState,
        status_callback: Callable[[str], None],
    ) -> None:
        super().__init__(master, background=HMI["background"])
        self.vehicle = vehicle
        self.tactical = tactical
        self.status_callback = status_callback
        self._tree_refreshing = False

        self._build_left_console()
        self.map = MapWidget(
            self,
            mission,
            vehicle,
            editable=False,
            status_callback=status_callback,
            tactical_state=tactical,
        )
        self.map.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._build_seeker_video()
        self.update_display()

    def _build_left_console(self) -> None:
        outer = tk.Frame(self, background=HMI["background"], width=390)
        outer.pack(side=tk.LEFT, fill=tk.Y)
        outer.pack_propagate(False)

        canvas = tk.Canvas(
            outer,
            background=HMI["background"],
            highlightthickness=0,
            width=372,
        )
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.console = tk.Frame(canvas, background=HMI["background"])
        window = canvas.create_window((0, 0), window=self.console, anchor="nw")
        self.console.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(window, width=event.width),
        )
        canvas.bind_all(
            "<Shift-MouseWheel>",
            lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"),
        )

        banner = tk.Frame(self.console, background="#18251c", height=36)
        banner.pack(fill=tk.X, padx=6, pady=(6, 4))
        banner.pack_propagate(False)
        tk.Label(
            banner,
            text="TACTICAL SITUATION // TRAINING",
            background="#18251c",
            foreground=HMI["green"],
            font=("Consolas", 10, "bold"),
        ).pack(side=tk.LEFT, padx=9, pady=8)
        self.zulu_label = tk.Label(
            banner,
            text="--:--:--",
            background="#18251c",
            foreground=HMI["muted"],
            font=("Consolas", 9),
        )
        self.zulu_label.pack(side=tk.RIGHT, padx=8)

        self._build_threat_list()
        self._build_information()
        self._build_readiness()
        self._build_mission_status()
        self._build_controls()

    def _build_threat_list(self) -> None:
        shell, body = _section(self.console, "위협 표적 / THREAT TRACKS")
        shell.pack(fill=tk.X, padx=6, pady=4)
        style = ttk.Style(self)
        style.configure(
            "Tactical.Treeview",
            background="#09110d",
            fieldbackground="#09110d",
            foreground=HMI["text"],
            rowheight=23,
            borderwidth=0,
            font=("Consolas", 8),
        )
        style.configure(
            "Tactical.Treeview.Heading",
            background="#243127",
            foreground=HMI["amber"],
            font=("Malgun Gothic", 8, "bold"),
            relief=tk.FLAT,
        )
        style.map(
            "Tactical.Treeview",
            background=[("selected", "#71322b")],
            foreground=[("selected", "#ffffff")],
        )
        self.threat_tree = ttk.Treeview(
            body,
            style="Tactical.Treeview",
            columns=("id", "speed", "heading", "tracked"),
            show="headings",
            height=5,
            selectmode="browse",
        )
        headings = (
            ("id", "NO", 52),
            ("speed", "속도", 72),
            ("heading", "방향", 66),
            ("tracked", "최초추적", 86),
        )
        for name, title, width in headings:
            self.threat_tree.heading(name, text=title)
            self.threat_tree.column(name, width=width, anchor="center", stretch=True)
        self.threat_tree.pack(fill=tk.X)
        self.threat_tree.bind("<Double-Button-1>", self._select_threat_from_tree)
        tk.Label(
            body,
            text="표적 행 또는 지도 심볼을 더블 클릭하여 추적 표적 선택",
            background=HMI["panel"],
            foreground=HMI["muted"],
            font=("Malgun Gothic", 7),
            anchor="w",
        ).pack(fill=tk.X, pady=(4, 0))

    def _build_information(self) -> None:
        shell, body = _section(self.console, "비행체 / 선택 표적 정보")
        shell.pack(fill=tk.X, padx=6, pady=4)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        self.vehicle_values = self._information_column(
            body,
            0,
            "VEHICLE",
            HMI["blue"],
        )
        self.target_values = self._information_column(
            body,
            1,
            "TARGET",
            HMI["red"],
        )

    def _information_column(
        self,
        master: tk.Misc,
        column: int,
        title: str,
        color: str,
    ) -> dict[str, tk.Label]:
        frame = tk.Frame(master, background="#0b130f")
        frame.grid(row=0, column=column, sticky="nsew", padx=(0, 3) if column == 0 else (3, 0))
        tk.Label(
            frame,
            text=title,
            background="#1b291f",
            foreground=color,
            font=("Consolas", 8, "bold"),
            pady=3,
        ).pack(fill=tk.X)
        values: dict[str, tk.Label] = {}
        for label, key in (
            ("고도", "alt"),
            ("위도", "lat"),
            ("경도", "lon"),
            ("속도", "speed"),
            ("방위각", "azimuth"),
        ):
            row = tk.Frame(frame, background="#0b130f")
            row.pack(fill=tk.X, padx=5, pady=1)
            tk.Label(
                row,
                text=label,
                background="#0b130f",
                foreground=HMI["muted"],
                font=("Malgun Gothic", 7),
                width=6,
                anchor="w",
            ).pack(side=tk.LEFT)
            value = tk.Label(
                row,
                text="--",
                background="#0b130f",
                foreground=HMI["text"],
                font=("Consolas", 8, "bold"),
                anchor="e",
            )
            value.pack(side=tk.RIGHT)
            values[key] = value
        return values

    def _build_readiness(self) -> None:
        shell, body = _section(self.console, "발사 준비 상태 / READINESS")
        shell.pack(fill=tk.X, padx=6, pady=4)
        self.readiness_lamps: dict[str, tk.Label] = {}
        row = tk.Frame(body, background=HMI["panel"])
        row.pack(fill=tk.X)
        for name in ("AVS", "LC", "RDR", "DL", "GCS"):
            cell = tk.Frame(row, background="#0a120e")
            cell.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            tk.Label(
                cell,
                text=name,
                background="#0a120e",
                foreground=HMI["muted"],
                font=("Consolas", 8, "bold"),
            ).pack(pady=(3, 0))
            lamp = tk.Label(
                cell,
                text="●",
                background="#0a120e",
                foreground=HMI["red"],
                font=("Segoe UI Symbol", 15),
            )
            lamp.pack(pady=(0, 3))
            self.readiness_lamps[name] = lamp
        self.launch_ready_label = tk.Label(
            body,
            text="● 발사 불가",
            background="#1b211c",
            foreground=HMI["red"],
            font=("Malgun Gothic", 9, "bold"),
            pady=5,
        )
        self.launch_ready_label.pack(fill=tk.X, pady=(5, 0))

    def _build_mission_status(self) -> None:
        shell, body = _section(self.console, "임무 상태 / MISSION STATUS")
        shell.pack(fill=tk.X, padx=6, pady=4)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        self.mission_lamps: dict[str, tk.Label] = {}
        for index, name in enumerate(self.tactical.mission_status):
            cell = tk.Frame(body, background="#0a120e")
            cell.grid(
                row=index // 2,
                column=index % 2,
                sticky="ew",
                padx=2,
                pady=2,
            )
            lamp = tk.Label(
                cell,
                text="●",
                background="#0a120e",
                foreground=HMI["red"],
                font=("Segoe UI Symbol", 11),
            )
            lamp.pack(side=tk.LEFT, padx=(5, 3))
            tk.Label(
                cell,
                text=name,
                background="#0a120e",
                foreground=HMI["text"],
                font=("Malgun Gothic", 8),
            ).pack(side=tk.LEFT, pady=3)
            self.mission_lamps[name] = lamp
        self.intercept_label = tk.Label(
            body,
            text="● 격추 대기",
            background="#1b211c",
            foreground=HMI["red"],
            font=("Malgun Gothic", 9, "bold"),
            pady=5,
        )
        self.intercept_label.grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(4, 0),
        )

    def _build_controls(self) -> None:
        shell, body = _section(self.console, "운용 모드 / LOCAL SIMULATION ONLY")
        shell.pack(fill=tk.X, padx=6, pady=(4, 8))
        mode_row = tk.Frame(body, background=HMI["panel"])
        mode_row.pack(fill=tk.X)
        tk.Label(
            mode_row,
            text="AUTO MODE",
            background=HMI["panel"],
            foreground=HMI["muted"],
            font=("Consolas", 8),
        ).pack(side=tk.LEFT)
        self.mode_label = tk.Label(
            mode_row,
            text="SAFE",
            background="#251c0d",
            foreground=HMI["amber"],
            font=("Consolas", 12, "bold"),
            width=8,
            pady=3,
        )
        self.mode_label.pack(side=tk.RIGHT)

        button_row = tk.Frame(body, background=HMI["panel"])
        button_row.pack(fill=tk.X, pady=(6, 0))
        self.launch_button = flat_button(
            button_row,
            "발사 (SIM)",
            self._request_launch,
            background="#2c2111",
            foreground=HMI["amber"],
            active_background="#5a451e",
            font=("Malgun Gothic", 10, "bold"),
            pady=8,
        )
        self.launch_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3))
        self.emergency_button = flat_button(
            button_row,
            "비상모드",
            self._toggle_emergency,
            background="#451713",
            foreground="#ffaaa4",
            active_background="#70221c",
            font=("Malgun Gothic", 10, "bold"),
            pady=8,
        )
        self.emergency_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(3, 0))

    def _build_seeker_video(self) -> None:
        shell = tk.Frame(
            self.map,
            background="#050806",
            highlightbackground=HMI["border"],
            highlightthickness=2,
            width=430,
            height=270,
        )
        shell.place(relx=1.0, rely=1.0, x=-18, y=-18, anchor="se")
        shell.pack_propagate(False)
        header = tk.Frame(shell, background="#18251c", height=27)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        tk.Label(
            header,
            text="SEEKER VIDEO // SIMULATED",
            background="#18251c",
            foreground=HMI["green"],
            font=("Consolas", 9, "bold"),
        ).pack(side=tk.LEFT, padx=8, pady=5)
        self.seeker_lock_label = tk.Label(
            header,
            text="NO LOCK",
            background="#18251c",
            foreground=HMI["red"],
            font=("Consolas", 8, "bold"),
        )
        self.seeker_lock_label.pack(side=tk.RIGHT, padx=8)
        self.seeker_canvas = tk.Canvas(
            shell,
            background="#07100b",
            highlightthickness=0,
        )
        self.seeker_canvas.pack(fill=tk.BOTH, expand=True)

    def _select_threat_from_tree(self, _event: tk.Event) -> None:
        if self._tree_refreshing:
            return
        selected = self.threat_tree.selection()
        if not selected:
            return
        track_id = int(selected[0])
        self.tactical.select_threat(track_id)
        self.status_callback(f"Threat THR-{track_id} selected.")
        self.update_display()

    def _request_launch(self) -> None:
        if self.tactical.request_simulated_launch():
            self.status_callback("SIM launch sequence accepted; no command was transmitted.")
        else:
            self.status_callback("SIM launch inhibited by readiness or automatic SAFE state.")
        self.update_display()

    def _toggle_emergency(self) -> None:
        enabled = self.tactical.toggle_emergency()
        self.status_callback(
            "Emergency simulation mode enabled."
            if enabled
            else "Emergency simulation mode cleared."
        )
        self.update_display()

    def update_display(self) -> None:
        self.zulu_label.configure(text=time.strftime("%H:%M:%S"))
        self._refresh_threat_tree()
        self._refresh_information()
        self._refresh_lamps()
        self._render_seeker()
        self.map.render()

    def _refresh_threat_tree(self) -> None:
        self._tree_refreshing = True
        try:
            self.threat_tree.delete(*self.threat_tree.get_children())
            for track in self.tactical.threats:
                self.threat_tree.insert(
                    "",
                    "end",
                    iid=str(track.track_id),
                    values=(
                        track.track_id,
                        f"{track.speed_mps:.0f}m/s",
                        f"{track.heading_deg:03.0f}°",
                        track.first_tracked_text,
                    ),
                )
            if self.tactical.selected_track_id is not None:
                item = str(self.tactical.selected_track_id)
                if self.threat_tree.exists(item):
                    self.threat_tree.selection_set(item)
        finally:
            self._tree_refreshing = False

    def _refresh_information(self) -> None:
        vehicle_data = {
            "alt": f"{self.vehicle.relative_altitude_m:,.1f} m",
            "lat": f"{self.vehicle.latitude:.5f}",
            "lon": f"{self.vehicle.longitude:.5f}",
            "speed": f"{self.vehicle.ground_speed_mps:.1f} m/s",
            "azimuth": f"{self.vehicle.heading_deg:03.0f}°",
        }
        for key, value in vehicle_data.items():
            self.vehicle_values[key].configure(text=value)

        threat = self.tactical.selected_threat
        target_data = {
            "alt": f"{threat.altitude_m:,.0f} m" if threat else "--",
            "lat": f"{threat.latitude:.5f}" if threat else "--",
            "lon": f"{threat.longitude:.5f}" if threat else "--",
            "speed": f"{threat.speed_mps:.0f} m/s" if threat else "--",
            "azimuth": f"{threat.heading_deg:03.0f}°" if threat else "--",
        }
        for key, value in target_data.items():
            self.target_values[key].configure(text=value)

    def _refresh_lamps(self) -> None:
        for name, ready in self.tactical.readiness.items():
            self.readiness_lamps[name].configure(
                foreground=HMI["green"] if ready else HMI["red"]
            )
        ready = self.tactical.launch_ready
        self.launch_ready_label.configure(
            text="● 발사 가능" if ready else "● 발사 불가",
            foreground=HMI["green"] if ready else HMI["red"],
        )
        for name, completed in self.tactical.mission_status.items():
            self.mission_lamps[name].configure(
                foreground=HMI["green"] if completed else HMI["red"]
            )
        success = self.tactical.engagement_success
        self.intercept_label.configure(
            text="● 격추 성공" if success else "● 격추 대기 / 실패",
            foreground=HMI["green"] if success else HMI["red"],
        )
        mode = self.tactical.automatic_mode
        self.mode_label.configure(
            text=mode,
            background="#11341b" if mode == "ARM" else "#251c0d",
            foreground=HMI["green"] if mode == "ARM" else HMI["amber"],
        )
        self.emergency_button.configure(
            text="비상 해제" if self.tactical.emergency_mode else "비상모드"
        )

    def _render_seeker(self) -> None:
        canvas = self.seeker_canvas
        width = max(canvas.winfo_width(), 420)
        height = max(canvas.winfo_height(), 235)
        canvas.delete("all")
        phase = time.monotonic()
        for index in range(28):
            y = int((index * 37 + phase * 18) % height)
            tone = 24 + (index * 7) % 35
            color = f"#{tone:02x}{tone + 12:02x}{tone:02x}"
            canvas.create_line(0, y, width, y, fill=color)
        for ring in (38, 72, 106):
            canvas.create_oval(
                width / 2 - ring,
                height / 2 - ring,
                width / 2 + ring,
                height / 2 + ring,
                outline="#31513b",
                dash=(3, 6),
            )
        canvas.create_line(width / 2, 15, width / 2, height - 15, fill="#77b889")
        canvas.create_line(15, height / 2, width - 15, height / 2, fill="#77b889")
        threat = self.tactical.selected_threat
        lock = self.tactical.mission_status["LOCK ON"] and threat is not None
        offset_x = math.sin(phase * 0.65) * 55
        offset_y = math.cos(phase * 0.52) * 35
        box_color = HMI["green"] if lock else HMI["amber"]
        canvas.create_rectangle(
            width / 2 + offset_x - 34,
            height / 2 + offset_y - 26,
            width / 2 + offset_x + 34,
            height / 2 + offset_y + 26,
            outline=box_color,
            width=2,
        )
        canvas.create_text(
            10,
            height - 10,
            anchor="sw",
            text=(
                f"THR-{threat.track_id}  AZ {threat.heading_deg:03.0f}  "
                f"ALT {threat.altitude_m:.0f}M"
                if threat
                else "NO DESIGNATED TRACK"
            ),
            fill="#9cc5a6",
            font=("Consolas", 8, "bold"),
        )
        self.seeker_lock_label.configure(
            text="LOCK" if lock else "NO LOCK",
            foreground=HMI["green"] if lock else HMI["red"],
        )


class DataView(tk.Frame):
    """MAVLink-oriented receive/log presentation using synthetic data."""

    SECTIONS = (
        "General",
        "Maps",
        "Video",
        "Data Report",
        "Launch Controller",
        "Data Link",
        "Seeker",
        "Motor",
        "Battery",
        "AVS",
        "Fuze",
    )
    LINK_CHANNELS = ("Telemetry", "ADT", "GDT", "FANET", "GPS", "Radar")

    def __init__(
        self,
        master: tk.Misc,
        vehicle: VehicleState,
        tactical: TacticalState,
        status_callback: Callable[[str], None],
    ) -> None:
        super().__init__(master, background=HMI["background"])
        self.vehicle = vehicle
        self.tactical = tactical
        self.status_callback = status_callback
        self.active_section = "Data Report"
        self._last_log_second = -1
        self._value_labels: dict[str, tk.Label] = {}
        self._nav_buttons: dict[str, tk.Button] = {}

        self._build_navigation()
        self._build_content()
        self.show_section("Data Report", announce=False)

    def _build_navigation(self) -> None:
        navigation = tk.Frame(self, background="#0d1711", width=255)
        navigation.pack(side=tk.LEFT, fill=tk.Y)
        navigation.pack_propagate(False)
        tk.Label(
            navigation,
            text="▤  DATA",
            background="#18251c",
            foreground=HMI["amber"],
            font=("Consolas", 18, "bold"),
            padx=16,
            pady=17,
            anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            navigation,
            text="MAVLINK DATA CONSOLE\nRECEIVE / SIMULATION",
            background="#0d1711",
            foreground=HMI["muted"],
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
            padx=17,
            pady=10,
        ).pack(fill=tk.X)
        for name in self.SECTIONS:
            button = flat_button(
                navigation,
                name,
                lambda value=name: self.show_section(value),
                background="#0d1711",
                foreground=HMI["text"],
                active_background="#26372b",
                font=("Consolas", 9, "bold"),
                padx=18,
                pady=8,
            )
            button.configure(anchor="w")
            button.pack(fill=tk.X, padx=6, pady=1)
            self._nav_buttons[name] = button

    def _build_content(self) -> None:
        content = tk.Frame(self, background=HMI["background"])
        content.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        header = tk.Frame(
            content,
            background="#18251c",
            highlightbackground=HMI["border"],
            highlightthickness=1,
        )
        header.pack(fill=tk.X)
        self.section_title = tk.Label(
            header,
            text="DATA REPORT",
            background="#18251c",
            foreground=HMI["green"],
            font=("Consolas", 18, "bold"),
            anchor="w",
            padx=15,
            pady=10,
        )
        self.section_title.pack(side=tk.LEFT)
        self.link_badge = tk.Label(
            header,
            text="● MAVLINK SIM",
            background="#18251c",
            foreground=HMI["green"],
            font=("Consolas", 9, "bold"),
            padx=14,
        )
        self.link_badge.pack(side=tk.RIGHT)

        self.channel_frame = tk.Frame(content, background=HMI["background"])
        self.channel_frame.pack(fill=tk.X, pady=(8, 0))
        self.channel_labels: dict[str, tk.Label] = {}
        for channel in self.LINK_CHANNELS:
            label = tk.Label(
                self.channel_frame,
                text=f"● {channel}",
                background="#111c15",
                foreground=HMI["green"],
                font=("Consolas", 8, "bold"),
                padx=9,
                pady=5,
                highlightbackground=HMI["border"],
                highlightthickness=1,
            )
            label.pack(side=tk.LEFT, padx=(0, 5))
            self.channel_labels[channel] = label

        data_shell, data_body = _section(content, "LIVE DATA FIELDS")
        data_shell.pack(fill=tk.X, pady=8)
        self.data_body = data_body

        log_shell, log_body = _section(content, "MAVLINK MESSAGE LOG // SYNTHETIC")
        log_shell.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
        self.log_text = tk.Text(
            log_body,
            background="#050906",
            foreground="#86cf97",
            insertbackground=HMI["green"],
            selectbackground="#28432f",
            relief=tk.FLAT,
            wrap=tk.NONE,
            font=("Consolas", 9),
            padx=8,
            pady=7,
        )
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll = ttk.Scrollbar(log_body, orient="vertical", command=self.log_text.yview)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=scroll.set, state=tk.DISABLED)

    def show_section(self, name: str, *, announce: bool = True) -> None:
        if name not in self.SECTIONS:
            return
        self.active_section = name
        self.section_title.configure(text=name.upper())
        for section, button in self._nav_buttons.items():
            selected = section == name
            button.configure(
                background="#2a3b2e" if selected else "#0d1711",
                foreground=HMI["amber"] if selected else HMI["text"],
            )
        if name == "Data Link":
            self.channel_frame.pack(fill=tk.X, pady=(8, 0))
        else:
            self.channel_frame.pack_forget()
        self._rebuild_data_fields()
        if announce:
            self.status_callback(f"DATA: {name}")

    def _section_rows(self) -> list[tuple[str, str]]:
        threat = self.tactical.selected_threat
        common = {
            "General": [
                ("Application", "Python Ground Control 0.2"),
                ("Mode", "Training / Simulation"),
                ("MAVLink transport", "Not connected (UI model)"),
                ("System time", time.strftime("%Y-%m-%d %H:%M:%S")),
            ],
            "Maps": [
                ("Provider", "Google Static Maps / Offline fallback"),
                ("Map type", "Hybrid"),
                ("Center", f"{self.vehicle.latitude:.6f}, {self.vehicle.longitude:.6f}"),
                ("API key", "Environment variable"),
            ],
            "Video": [
                ("Source", "Seeker simulation"),
                ("Resolution", "430 x 243 preview"),
                ("Lock state", "LOCK" if self.tactical.mission_status["LOCK ON"] else "NO LOCK"),
                ("Target", f"THR-{threat.track_id}" if threat else "NONE"),
            ],
            "Data Report": [
                ("HEARTBEAT", "1.0 Hz / SIM"),
                ("Vehicle position", f"{self.vehicle.latitude:.6f}, {self.vehicle.longitude:.6f}"),
                ("Flight mode", self.vehicle.flight_mode),
                ("GPS satellites", str(self.vehicle.gps_satellites)),
                ("Threat tracks", str(len(self.tactical.threats))),
                ("Selected track", f"THR-{threat.track_id}" if threat else "NONE"),
                ("Link quality", f"{self.vehicle.link_percent}%"),
                ("Battery", f"{self.vehicle.battery_percent}%"),
            ],
            "Launch Controller": [
                ("Controller", "LC-SIM"),
                ("Readiness", "READY" if self.tactical.readiness["LC"] else "NOT READY"),
                ("Automatic mode", self.tactical.automatic_mode),
                ("Command TX", "DISABLED"),
            ],
            "Data Link": [
                ("Telemetry", f"{self.vehicle.link_percent}% / 10 Hz"),
                ("ADT", "SIM READY / 5 Hz"),
                ("GDT", "SIM READY / 5 Hz"),
                ("FANET", "SIM READY / 2 Hz"),
                ("GPS", f"{self.vehicle.gps_fix} / {self.vehicle.gps_satellites} SAT"),
                ("Radar", f"{len(self.tactical.threats)} TRACKS / 4 Hz"),
            ],
            "Seeker": [
                ("Track", f"THR-{threat.track_id}" if threat else "NONE"),
                ("Shutter", "ON" if self.tactical.mission_status["셔터 ON"] else "OFF"),
                ("Lock", "ON" if self.tactical.mission_status["LOCK ON"] else "OFF"),
                ("Video", "SIMULATED"),
            ],
            "Motor": [
                ("State", "STANDBY"),
                ("RPM", "0"),
                ("Temperature", "24.6 °C"),
                ("Command TX", "DISABLED"),
            ],
            "Battery": [
                ("Remaining", f"{self.vehicle.battery_percent}%"),
                ("Voltage", "22.8 V / SIM"),
                ("Current", "4.2 A / SIM"),
                ("Health", "NOMINAL"),
            ],
            "AVS": [
                ("State", "READY" if self.tactical.readiness["AVS"] else "NOT READY"),
                ("Roll", f"{self.vehicle.roll_deg:+.2f}°"),
                ("Pitch", f"{self.vehicle.pitch_deg:+.2f}°"),
                ("Heading", f"{self.vehicle.heading_deg:03.1f}°"),
            ],
            "Fuze": [
                ("State", "ACTIVE" if self.tactical.mission_status["신관 작동"] else "SAFE"),
                ("TDD detection", "YES" if self.tactical.mission_status["TDD 탐지"] else "NO"),
                ("Command TX", "DISABLED"),
                ("Data source", "SYNTHETIC"),
            ],
        }
        return common[self.active_section]

    def _rebuild_data_fields(self) -> None:
        for child in self.data_body.winfo_children():
            child.destroy()
        self._value_labels.clear()
        rows = self._section_rows()
        columns = 2 if len(rows) > 4 else 1
        for index, (name, value) in enumerate(rows):
            column = index % columns
            row = index // columns
            cell = tk.Frame(
                self.data_body,
                background="#0a120e",
                highlightbackground=HMI["grid"],
                highlightthickness=1,
            )
            cell.grid(row=row, column=column, sticky="ew", padx=3, pady=3)
            tk.Label(
                cell,
                text=name.upper(),
                background="#0a120e",
                foreground=HMI["muted"],
                font=("Consolas", 8),
                anchor="w",
                padx=8,
                pady=3,
            ).pack(fill=tk.X)
            value_label = tk.Label(
                cell,
                text=value,
                background="#0a120e",
                foreground=HMI["green"],
                font=("Consolas", 10, "bold"),
                anchor="w",
                padx=8,
                pady=4,
            )
            value_label.pack(fill=tk.X)
            self._value_labels[name] = value_label
        for column in range(columns):
            self.data_body.columnconfigure(column, weight=1)

    def update_data(self) -> None:
        self.link_badge.configure(
            text="● MAVLINK SIM" if self.vehicle.connected else "● LINK DOWN",
            foreground=HMI["green"] if self.vehicle.connected else HMI["red"],
        )
        second = int(time.monotonic())
        if second == self._last_log_second:
            return
        self._last_log_second = second
        self._rebuild_data_fields()
        messages = (
            (
                "HEARTBEAT",
                f"type=SIM autopilot=GENERIC mode={self.vehicle.flight_mode}",
            ),
            (
                "GLOBAL_POSITION_INT",
                f"lat={self.vehicle.latitude:.7f} lon={self.vehicle.longitude:.7f} "
                f"relative_alt={self.vehicle.relative_altitude_m:.1f}",
            ),
            (
                "ATTITUDE",
                f"roll={self.vehicle.roll_deg:+.3f} pitch={self.vehicle.pitch_deg:+.3f} "
                f"yaw={self.vehicle.heading_deg:.2f}",
            ),
            (
                "SYS_STATUS",
                f"battery_remaining={self.vehicle.battery_percent} "
                f"link={self.vehicle.link_percent}",
            ),
            (
                "RADAR_TRACK",
                f"tracks={len(self.tactical.threats)} "
                f"selected={self.tactical.selected_track_id}",
            ),
        )
        message_name, fields = messages[second % len(messages)]
        line = f"{time.strftime('%H:%M:%S')}  RX  {message_name:<22} {fields}\n"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line)
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 250:
            self.log_text.delete("1.0", "40.0")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

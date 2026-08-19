from __future__ import annotations

import math
import os
import time
import json
from concurrent.futures import Future, ThreadPoolExecutor

from PySide6.QtCore import QEvent, QSignalBlocker, QRectF, QUrl, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QCursor, QFont, QPainter, QPen
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtWebEngineCore import QWebEngineSettings
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QMenu,
    QPushButton,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .fly_bridge import FlyMapBridge
from .fly_map_html import ASSET_DIR, load_fly_map_html, load_seeker_map_html
from .fly_state import (
    FlyState,
    LM_CRUISE_SPEED_MPS,
    destination_position,
    horizontal_distance_m,
)
from .map_bridge import MapBridge
from .planning import MODEL_NAME, RuleBasedPlanningEngine, SeekerSpec
from .site_store import MissionPoint, SiteStore


GREEN = "#55e77a"
AMBER = "#f3b52d"
RED = "#ff554c"
TARGET_YELLOW = "#ffd34f"
BLUE = "#4db3ff"
MUTED = "#869187"
WAYPOINT_CONTEXT_DELETE_RADIUS_M = 2500.0
MANUAL_ROUTE_HOLD_S = 50.0


def json_safe_payload(value):
    """Return a strict-JSON-safe copy of a map telemetry payload.

    Python's JSON encoder writes NaN/Infinity by default, while the browser's
    ``JSON.parse`` rejects them.  A single non-finite planning metric would
    therefore suspend every vehicle repaint until a later valid frame, which
    appears to the operator as a freeze followed by a position jump.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {
            key: json_safe_payload(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [json_safe_payload(item) for item in value]
    return value


def simplify_flight_path(
    points: list[dict],
    *,
    tolerance_m: float = 24.0,
    maximum_points: int = 96,
) -> list[dict]:
    """Keep the drawn track accurate without feeding hundreds of collinear
    vertices into Google Maps 3D on every telemetry frame."""
    if len(points) <= 2:
        return list(points)

    reference_latitude = sum(
        float(point["latitude"]) for point in points
    ) / len(points)
    longitude_scale = 111_320.0 * math.cos(math.radians(reference_latitude))
    projected = [
        (
            float(point["longitude"]) * longitude_scale,
            float(point["latitude"]) * 111_320.0,
        )
        for point in points
    ]
    retained = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        start_index, end_index = stack.pop()
        start_x, start_y = projected[start_index]
        end_x, end_y = projected[end_index]
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        length_squared = delta_x * delta_x + delta_y * delta_y
        best_distance = -1.0
        best_index = -1
        for index in range(start_index + 1, end_index):
            point_x, point_y = projected[index]
            if length_squared <= 1e-9:
                distance = math.hypot(point_x - start_x, point_y - start_y)
            else:
                fraction = max(
                    0.0,
                    min(
                        1.0,
                        (
                            (point_x - start_x) * delta_x
                            + (point_y - start_y) * delta_y
                        )
                        / length_squared,
                    ),
                )
                distance = math.hypot(
                    point_x - (start_x + delta_x * fraction),
                    point_y - (start_y + delta_y * fraction),
                )
            if distance > best_distance:
                best_distance = distance
                best_index = index
        if best_index >= 0 and best_distance > tolerance_m:
            retained.add(best_index)
            stack.append((start_index, best_index))
            stack.append((best_index, end_index))

    ordered = sorted(retained)
    if len(ordered) > maximum_points:
        ordered = sorted(
            {
                ordered[
                    round(index * (len(ordered) - 1) / (maximum_points - 1))
                ]
                for index in range(maximum_points)
            }
        )
    return [points[index] for index in ordered]


def build_mission_map_plan_payload(
    live_store: SiteStore,
    pending_store: SiteStore,
    pending_dirty: bool,
) -> dict:
    """Build a visual-only plan with newly added waypoints marked as pending."""
    display_store = pending_store if pending_dirty else live_store
    plan = display_store.render_dict()
    plan["pending_edit"] = pending_dirty
    if not pending_dirty:
        return plan

    original_coordinates = {
        vehicle_id: {
            (
                round(point.latitude, 7),
                round(point.longitude, 7),
                round(point.altitude_m, 1),
            )
            for point in live_store.waypoints_for(vehicle_id)
        }
        for vehicle_id in SiteStore.VEHICLE_IDS
    }

    def mark_waypoints(waypoints: list[dict], vehicle_id: int) -> None:
        known_coordinates = original_coordinates.get(vehicle_id, set())
        for waypoint in waypoints:
            coordinate = (
                round(float(waypoint["latitude"]), 7),
                round(float(waypoint["longitude"]), 7),
                round(float(waypoint.get("altitude_m", 0.0)), 1),
            )
            waypoint["pending_preview"] = coordinate not in known_coordinates

    for route in plan.get("vehicle_routes", []):
        mark_waypoints(
            route.get("waypoints", []),
            int(route.get("vehicle_id", 0)),
        )
    mark_waypoints(
        plan.get("waypoints", []),
        int(plan.get("active_vehicle_id", 1)),
    )
    return plan


def waypoint_for_context_key(
    route,
    vehicle_id: int,
    context_key: str | None,
):
    """Resolve the exact waypoint named by a map marker context key."""
    if not context_key or ":" not in context_key:
        return None
    owner, code = context_key.split(":", 1)
    if owner != "FLEET":
        try:
            if int(owner) != int(vehicle_id):
                return None
        except ValueError:
            return None
    if not code.startswith("WP"):
        return None
    return next((point for point in route if point.code == code), None)


def nearest_waypoint_within(
    route,
    latitude: float,
    longitude: float,
    radius_m: float = WAYPOINT_CONTEXT_DELETE_RADIUS_M,
):
    """Return a nearby waypoint without ever selecting a distant route point."""
    if not route:
        return None
    waypoint = min(
        route,
        key=lambda point: horizontal_distance_m(
            latitude,
            longitude,
            point.latitude,
            point.longitude,
        ),
    )
    distance_m = horizontal_distance_m(
        latitude,
        longitude,
        waypoint.latitude,
        waypoint.longitude,
    )
    return waypoint if distance_m <= radius_m else None


def runtime_route_index_for_context_key(
    route: list[dict],
    vehicle_id: int,
    context_key: str | None,
) -> int | None:
    """Resolve a PLAN or live AUTO marker to one exact editable route index."""
    if not context_key:
        return None
    parts = context_key.split(":")
    if len(parts) == 3 and parts[0] == "AUTO":
        try:
            owner = int(parts[1])
            index = int(parts[2])
        except ValueError:
            return None
        return index if owner == int(vehicle_id) and 0 <= index < len(route) else None
    if len(parts) != 2:
        return None
    owner, code = parts
    if owner != "FLEET":
        try:
            if int(owner) != int(vehicle_id):
                return None
        except ValueError:
            return None
    return next(
        (
            index
            for index, point in enumerate(route)
            if str(point.get("code", "")) == code
        ),
        None,
    )


def nearest_runtime_route_index_within(
    route: list[dict],
    latitude: float,
    longitude: float,
    radius_m: float = WAYPOINT_CONTEXT_DELETE_RADIUS_M,
) -> int | None:
    if not route:
        return None
    index = min(
        range(len(route)),
        key=lambda candidate: horizontal_distance_m(
            latitude,
            longitude,
            float(route[candidate]["latitude"]),
            float(route[candidate]["longitude"]),
        ),
    )
    distance_m = horizontal_distance_m(
        latitude,
        longitude,
        float(route[index]["latitude"]),
        float(route[index]["longitude"]),
    )
    return index if distance_m <= radius_m else None


class SeekerVideoWidget(QWidget):
    activated = Signal()

    def __init__(self, state: FlyState, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.seeker_spec = SeekerSpec()
        self._map_loaded = False
        self._last_payload = ""
        self.setMinimumSize(390, 220)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.web_view = QWebEngineView(self)
        self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        settings = self.web_view.settings()
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,
            True,
        )
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled,
            True,
        )
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.WebGLEnabled,
            True,
        )
        self.web_view.installEventFilter(self)
        layout.addWidget(self.web_view)
        self.web_view.loadFinished.connect(self._on_map_loaded)
        self.web_view.setHtml(
            load_seeker_map_html(
                os.getenv("GOOGLE_MAPS_API_KEY", "").strip()
            ),
            QUrl.fromLocalFile(str(ASSET_DIR.resolve()) + os.sep),
        )

    def _on_map_loaded(self, ok: bool) -> None:
        self._map_loaded = bool(ok)
        self._last_payload = ""
        self.refresh()

    def _state_payload(self) -> dict:
        vehicle = self.state.vehicle
        target = self.state.selected_threat
        target_latitude = target.latitude if target is not None else vehicle.latitude
        target_longitude = (
            target.longitude if target is not None else vehicle.longitude
        )
        target_distance_m = (
            horizontal_distance_m(
                vehicle.latitude,
                vehicle.longitude,
                target_latitude,
                target_longitude,
            )
            if target is not None
            else float("inf")
        )
        altitude_separation_m = abs(
            float(vehicle.altitude_m)
            - float(target.altitude_m if target is not None else 0.0)
        )
        slant_distance_m = math.hypot(
            target_distance_m,
            altitude_separation_m,
        )
        off_nadir_deg = math.degrees(
            math.atan2(
                target_distance_m,
                max(1.0, altitude_separation_m),
            )
        )
        coverage_m = self.seeker_spec.gimbal_centerline_envelope_m
        target_visible = bool(
            target is not None
            and target_distance_m <= coverage_m / 2.0
            and off_nadir_deg <= self.seeker_spec.max_gimbal_angle_deg
        )
        return {
            "vehicle": {
                "latitude": vehicle.latitude,
                "longitude": vehicle.longitude,
                "heading_deg": vehicle.heading_deg,
                "altitude_m": vehicle.altitude_m,
            },
            "target": {
                "latitude": target_latitude,
                "longitude": target_longitude,
                "code": target.code if target is not None else "NO TARGET",
                "altitude_m": (
                    target.altitude_m if target is not None else 0.0
                ),
            },
            "target_visible": target_visible,
            "target_distance_m": target_distance_m,
            "horizontal_distance_m": target_distance_m,
            "altitude_separation_m": altitude_separation_m,
            "slant_distance_m": slant_distance_m,
            "off_nadir_deg": off_nadir_deg,
            "coverage_m": coverage_m,
            "phase": self.state.flight_phase,
            "seeker_mode": self.state.seeker_mode,
            "locked": bool(self.state.mission_status["LOCK ON"]),
            "destroyed": bool(self.state.target_destroyed),
        }

    def refresh(self) -> None:
        if not self._map_loaded:
            return
        payload = json.dumps(
            self._state_payload(),
            ensure_ascii=False,
            separators=(",", ":"),
        )
        if payload == self._last_payload:
            return
        self._last_payload = payload
        self.web_view.page().runJavaScript(
            f"window.updateSeeker && window.updateSeeker({payload});"
        )

    def eventFilter(self, watched, event) -> bool:  # type: ignore[override]
        if (
            watched is self.web_view
            and event.type() == QEvent.Type.MouseButtonDblClick
        ):
            self.activated.emit()
            return True
        return super().eventFilter(watched, event)

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        self.refresh()
        return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#050b07"))
        width = self.width()
        height = self.height()
        phase = time.monotonic()

        for index in range(34):
            y = int((index * 31 + phase * 19) % max(1, height))
            tone = 20 + (index * 5) % 28
            painter.setPen(QPen(QColor(tone, tone + 12, tone + 2, 175), 1))
            painter.drawLine(0, y, width, y)

        painter.setPen(QPen(QColor("#31513b"), 1, Qt.PenStyle.DashLine))
        center_x = width / 2
        center_y = height / 2
        for radius in (38, 72, 106):
            painter.drawEllipse(QRectF(
                center_x - radius,
                center_y - radius,
                radius * 2,
                radius * 2,
            ))

        painter.setPen(QPen(QColor("#77b889"), 1))
        painter.drawLine(int(center_x), 14, int(center_x), height - 14)
        painter.drawLine(14, int(center_y), width - 14, int(center_y))
        painter.drawLine(int(center_x - 14), int(center_y), int(center_x + 14), int(center_y))
        painter.drawLine(int(center_x), int(center_y - 14), int(center_x), int(center_y + 14))

        target = self.state.selected_threat
        seeker_mode = self.state.seeker_mode
        locked = self.state.mission_status["LOCK ON"] and target is not None
        if seeker_mode == "SEARCH":
            sweep_angle = (phase * 0.9) % (math.pi * 2.0)
            sweep_radius = min(width, height) * 0.38
            painter.setPen(QPen(QColor("#55e77a"), 2))
            painter.drawLine(
                int(center_x),
                int(center_y),
                int(center_x + math.sin(sweep_angle) * sweep_radius),
                int(center_y - math.cos(sweep_angle) * sweep_radius),
            )
        offset_x = 0.0 if locked else math.sin(phase * 0.65) * 56
        offset_y = 0.0 if locked else math.cos(phase * 0.52) * 34
        if (
            target is not None
            and self.state.target_detected
            and self.state.flight_phase
            in {
                "DETECTION_TRANSIT",
                "INITIAL_GUIDANCE",
                "MIDCOURSE_GUIDANCE",
                "TERMINAL_GUIDANCE",
                "DESTROYED",
            }
        ):
            terminal = self.state.flight_phase in {
                "TERMINAL_GUIDANCE",
                "DESTROYED",
            }
            acquiring = self.state.flight_phase in {
                "DETECTION_TRANSIT",
                "INITIAL_GUIDANCE",
            }
            box_color = QColor(
                RED if terminal else (AMBER if acquiring else TARGET_YELLOW)
            )
            target_box = QRectF(
                center_x + offset_x - 34,
                center_y + offset_y - 25,
                68,
                50,
            )
            painter.setPen(QPen(box_color, 2))
            painter.drawRect(target_box)
            painter.setFont(QFont("IBM Plex Mono", 8, QFont.Weight.Bold))
            painter.drawText(
                QRectF(
                    target_box.left(),
                    target_box.top() - 18,
                    150,
                    16,
                ),
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                (
                    "TARGET / TERMINAL"
                    if terminal
                    else (
                        "TARGET / ATR ACQUIRE"
                        if acquiring
                        else "TARGET / MIDCOURSE"
                    )
                ),
            )

        painter.setFont(QFont("IBM Plex Mono", 8, QFont.Weight.DemiBold))
        painter.setPen(QColor(GREEN if locked else RED))
        painter.drawText(10, 18, "LOCK" if locked else "NO LOCK")
        mode_color = (
            RED
            if seeker_mode in {"ATR LOCK", "SHUT DOWN"}
            else (TARGET_YELLOW if seeker_mode.startswith("ATR") else GREEN)
        )
        painter.setPen(QColor(mode_color))
        painter.drawText(
            QRectF(0, 7, width - 10, 18),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            seeker_mode,
        )
        painter.setPen(QColor("#9cc5a6"))
        target_text = (
            f"{target.code}  AZ {target.heading_deg:03.0f}°  "
            f"ALT {target.altitude_m:.0f}M"
            if target is not None and self.state.target_detected
            else "ATR SEARCH / NO DETECTION"
        )
        painter.drawText(10, height - 10, target_text)
        if self.state.target_destroyed:
            painter.setPen(QPen(QColor(RED), 5))
            cross_size = min(width, height) * 0.18
            painter.drawLine(
                int(center_x - cross_size),
                int(center_y - cross_size),
                int(center_x + cross_size),
                int(center_y + cross_size),
            )
            painter.drawLine(
                int(center_x - cross_size),
                int(center_y + cross_size),
                int(center_x + cross_size),
                int(center_y - cross_size),
            )
            painter.setFont(
                QFont("IBM Plex Mono", 15, QFont.Weight.Bold)
            )
            painter.drawText(
                QRectF(0, center_y + cross_size + 8, width, 30),
                Qt.AlignmentFlag.AlignCenter,
                "SHUT DOWN",
            )
        painter.end()

    def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
        self.activated.emit()
        event.accept()


class FlyMapStage(QWidget):
    def __init__(
        self,
        web_view: QWebEngineView | None,
        state: FlyState,
        seeker_spec: SeekerSpec | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.web_view = web_view
        self.map_layout = QVBoxLayout(self)
        self.map_layout.setContentsMargins(0, 0, 0, 0)
        if web_view is not None:
            self.map_layout.addWidget(web_view)

        self.seeker_panel = QFrame(self)
        self.seeker_panel.setObjectName("seekerPanel")
        self.seeker_panel.setStyleSheet(
            """
            QFrame#seekerPanel {
                background: #050906;
                border-top: 1px solid #69766a;
                border-left: 1px solid #59665a;
                border-right: 3px solid #020403;
                border-bottom: 3px solid #020403;
            }
            """
        )
        self.seeker_panel.setFixedSize(440, 280)
        seeker_layout = QVBoxLayout(self.seeker_panel)
        seeker_layout.setContentsMargins(6, 5, 6, 6)
        seeker_layout.setSpacing(4)
        header = QHBoxLayout()
        title = QLabel("SEEKER VIDEO // EO")
        title.setObjectName("fieldCaption")
        self.lock_label = QLabel("● NO LOCK")
        self.lock_label.setObjectName("dataValue")
        header.addWidget(title)
        header.addStretch(1)
        self.split_button = QPushButton("탐색기 화면 (6분할)")
        self.split_button.setMaximumHeight(26)
        self.split_button.setStyleSheet("padding:2px 6px; font-size:8pt;")
        header.addWidget(self.split_button)
        header.addWidget(self.lock_label)
        seeker_layout.addLayout(header)
        self.seeker_video = SeekerVideoWidget(state)
        seeker_layout.addWidget(self.seeker_video, 1)
        seeker_spec = seeker_spec or SeekerSpec()
        specification = QLabel(
            "ATR // "
            f"ALT {seeker_spec.altitude_m:.0f}m · "
            f"Af {seeker_spec.horizontal_fov_deg:.0f}° · "
            f"Ag {seeker_spec.max_gimbal_angle_deg:.0f}° · "
            f"COVER {seeker_spec.gimbal_centerline_envelope_m:.0f}m · "
            f"CELL {seeker_spec.nominal_cell_m:.0f}m/"
            f"{seeker_spec.nominal_cell_search_time_s:.1f}s · "
            f"AREA {seeker_spec.nominal_search_area_km2:.1f}km²/"
            f"{seeker_spec.nominal_search_time_min:.0f}min"
        )
        specification.setObjectName("mutedText")
        specification.setStyleSheet("font-size:7pt;")
        seeker_layout.addWidget(specification)
        self.seeker_panel.raise_()
        self.mission_modify_button = QPushButton("임무지도 수정", self)
        self.mission_modify_button.setObjectName("primaryButton")
        self.mission_modify_button.setFixedSize(110, 28)
        self.mission_modify_button.raise_()

    def attach_map_view(self, web_view: QWebEngineView) -> None:
        self.web_view = web_view
        self.map_layout.addWidget(web_view)
        web_view.show()
        self.seeker_panel.raise_()

    def refresh_seeker(self) -> None:
        locked = self.seeker_video.state.mission_status["LOCK ON"]
        destroyed = self.seeker_video.state.target_destroyed
        seeker_mode = self.seeker_video.state.seeker_mode
        self.lock_label.setText(
            "SHUT DOWN"
            if destroyed
            else ("● LOCK" if locked else f"● {seeker_mode}")
        )
        self.lock_label.setStyleSheet(
            f"color: {RED if destroyed else (GREEN if locked else RED)};"
        )
        self.seeker_video.refresh()

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        margin = 16
        self.seeker_panel.move(
            max(margin, self.width() - self.seeker_panel.width() - margin),
            max(margin, self.height() - self.seeker_panel.height() - margin),
        )
        self.mission_modify_button.move(
            max(margin, self.width() - self.mission_modify_button.width() - margin),
            64,
        )
        self.seeker_panel.raise_()
        self.mission_modify_button.raise_()


class Fly3DView(QWidget):
    statusMessage = Signal(str)
    vehicleSelectionRequested = Signal(int)

    def __init__(
        self,
        store: SiteStore,
        parent: QWidget | None = None,
        *,
        shared_web_view: QWebEngineView | None = None,
        shared_bridge: MapBridge | None = None,
    ) -> None:
        super().__init__(parent)
        self.store = store
        self._shared_web_view = shared_web_view
        self._shared_bridge = shared_bridge
        self._uses_shared_map = (
            shared_web_view is not None and shared_bridge is not None
        )
        center = store.sites.get("GCS")
        center_lat = center.latitude if center else 37.3422
        center_lon = center.longitude if center else 127.9202
        self.states = {
            vehicle_id: FlyState.demo(center_lat, center_lon)
            for vehicle_id in SiteStore.VEHICLE_IDS
        }
        self.selected_vehicle_id = 0
        self.state = self.states[1]
        self.seeker_spec = SeekerSpec()
        self.planning_engine = self._new_planning_engine(center_lat, center_lon)
        self._planning_payload: dict = {
            "model": MODEL_NAME,
            "status": "INITIALIZING",
            "sensor": self.seeker_spec.display_dict(),
        }
        self._planning_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="gcs-rhp-planner",
        )
        self._planning_future: Future | None = None
        self._planning_future_generation = 0
        self._planning_future_elapsed_s = 0.0
        self._planning_future_initial_precompute = False
        self._planning_generation = 0
        self._initial_rhp_precompute_requested = False
        self._last_planning_cycle_s = -1e9
        self._last_planning_search_elapsed_s = -1e9
        self.api_key = os.getenv("GOOGLE_MAPS_API_KEY", "").strip()
        self.map_html, self.provider_name = load_fly_map_html(self.api_key)
        self._map_loaded = False
        self._has_activated = False
        self._active = False
        self._refreshing_table = False
        self._last_map_track_click: tuple[int, float] | None = None
        self._reported_shutdown = False
        self._detection_dialog: QMessageBox | None = None
        self._pending_mission = SiteStore()
        self._pending_dirty = False
        self._pending_runtime_routes: dict[int, list[dict]] = {}
        self._pending_runtime_originals: dict[int, list[dict]] = {}
        self._manual_route_applied_vehicle_ids: set[int] = set()
        self._plan_render_revision = 0
        self._cached_plan_key: tuple[int, bool] | None = None
        self._cached_plan_payload: dict | None = None
        self._last_sent_plan_key: tuple[int, bool] | None = None
        self._flight_path_payload_cache: dict[
            int,
            tuple[tuple, list[dict]],
        ] = {}
        self._preserve_execution_on_next_plan_change = False
        self._pending_context_position: tuple[float, float, float, float] | None = None
        self._pending_context_feature: tuple[str, float] | None = None
        self._context_menu_block_until = 0.0
        self._split_active_vehicle_id = 1
        self._last_console_refresh_at = 0.0
        self._last_seeker_refresh_at = 0.0
        self._last_map_emit_at = 0.0

        self._build_ui()
        self._connect_map()
        self.store.subscribe(self._on_plan_changed)
        for vehicle_id, state in self.states.items():
            state.load_mission(self.store, vehicle_id)
        self._refresh_display()

        self.update_timer = QTimer(self)
        self.update_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.update_timer.timeout.connect(self._tick)
        self.update_timer.setInterval(50)

    def _build_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)
        layout.addWidget(self._build_console())

        if self._shared_web_view is None:
            self.web_view = QWebEngineView()
            self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
            settings = self.web_view.settings()
            settings.setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)
            settings.setAttribute(
                QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled,
                True,
            )
        else:
            self.web_view = self._shared_web_view
        self.map_stage = FlyMapStage(
            None if self._uses_shared_map else self.web_view,
            self.state,
            self.seeker_spec,
        )
        self.map_stage.split_button.clicked.connect(self._show_split_seekers)
        self.map_stage.mission_modify_button.clicked.connect(
            self._apply_pending_mission
        )
        self.map_stage.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
        layout.addWidget(self.map_stage, 1)

    def _build_console(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setObjectName("flyConsole")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFixedWidth(410)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        content = QWidget()
        content.setObjectName("toolPanel")
        panel = QVBoxLayout(content)
        panel.setContentsMargins(9, 9, 9, 12)
        panel.setSpacing(6)

        banner = QFrame()
        banner.setStyleSheet(
            "background:#17231a; border:1px solid #435346;"
        )
        banner_layout = QHBoxLayout(banner)
        banner_layout.setContentsMargins(9, 6, 9, 6)
        title = QLabel("임무 지도 // MISSION MAP")
        title.setObjectName("panelTitle")
        title.setStyleSheet("border:0; padding:0;")
        self.source_label = QLabel("LOCAL DATA")
        self.source_label.setObjectName("dataValue")
        banner_layout.addWidget(title)
        banner_layout.addStretch(1)
        banner_layout.addWidget(self.source_label)
        panel.addWidget(banner)

        panel.addWidget(self._build_threat_group())
        panel.addWidget(self._build_information_group())
        panel.addWidget(self._build_readiness_group())
        panel.addWidget(self._build_mission_group())
        panel.addWidget(self._build_control_group())
        panel.addStretch(1)
        scroll.setWidget(content)
        return scroll

    def _build_threat_group(self) -> QWidget:
        group = QGroupBox("위협 표적 / THREAT TRACKS")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(7, 12, 7, 7)
        layout.setSpacing(4)
        self.threat_table = QTableWidget(0, 4)
        self.threat_table.setHorizontalHeaderLabels(
            ("NO/TYPE", "속도", "방향", "최초 추적")
        )
        self.threat_table.setAlternatingRowColors(True)
        self.threat_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.threat_table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self.threat_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.threat_table.verticalHeader().setVisible(False)
        header = self.threat_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.threat_table.setFixedHeight(120)
        self.threat_table.cellDoubleClicked.connect(self._select_threat_from_table)
        layout.addWidget(self.threat_table)
        return group

    def _build_information_group(self) -> QWidget:
        group = QGroupBox("비행체 / 선택 표적 정보")
        layout = QHBoxLayout(group)
        layout.setContentsMargins(7, 12, 7, 7)
        layout.setSpacing(6)
        (
            vehicle_frame,
            self.vehicle_title_label,
            self.vehicle_values,
        ) = self._information_column(
            "LM-01",
            BLUE,
        )
        target_frame, _target_title, self.target_values = self._information_column(
            "TARGET",
            RED,
        )
        layout.addWidget(vehicle_frame, 1)
        layout.addWidget(target_frame, 1)
        return group

    @staticmethod
    def _information_column(
        title: str,
        color: str,
    ) -> tuple[QWidget, QLabel, dict[str, QLabel]]:
        frame = QFrame()
        frame.setStyleSheet("background:#09100c; border:1px solid #2e3930;")
        layout = QGridLayout(frame)
        layout.setContentsMargins(6, 5, 6, 5)
        layout.setHorizontalSpacing(5)
        layout.setVerticalSpacing(2)
        title_label = QLabel(title)
        title_label.setStyleSheet(
            f"color:{color}; font-family:'IBM Plex Mono'; font-weight:700; border:0;"
        )
        layout.addWidget(title_label, 0, 0, 1, 2)
        values: dict[str, QLabel] = {}
        for row, (caption, key) in enumerate(
            (
                ("고도", "altitude"),
                ("위도", "latitude"),
                ("경도", "longitude"),
                ("속도", "speed"),
                ("방위각", "heading"),
                ("거리", "distance"),
                ("도달시간", "eta"),
            ),
            start=1,
        ):
            caption_label = QLabel(caption)
            caption_label.setObjectName("mutedText")
            caption_label.setStyleSheet("border:0;")
            value_label = QLabel("--")
            value_label.setObjectName("dataValue")
            value_label.setAlignment(Qt.AlignmentFlag.AlignRight)
            value_label.setStyleSheet("border:0;")
            layout.addWidget(caption_label, row, 0)
            layout.addWidget(value_label, row, 1)
            values[key] = value_label
        return frame, title_label, values

    def _build_readiness_group(self) -> QWidget:
        group = QGroupBox("발사 준비 상태 / READINESS")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(7, 12, 7, 7)
        lamp_row = QHBoxLayout()
        lamp_row.setSpacing(3)
        self.readiness_lamps: dict[str, QLabel] = {}
        for name in ("AVS", "LC", "RDR", "DL", "GCS"):
            lamp, label = self._lamp_cell(name)
            self.readiness_lamps[name] = label
            lamp_row.addWidget(lamp, 1)
        layout.addLayout(lamp_row)
        self.launch_ready_label = QLabel("● 발사 불가")
        self.launch_ready_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.launch_ready_label.setStyleSheet(
            f"color:{RED}; background:#120a08; border:1px solid #5f2824; padding:5px;"
        )
        layout.addWidget(self.launch_ready_label)
        return group

    def _build_mission_group(self) -> QWidget:
        group = QGroupBox("임무 진행 상태 / MISSION")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(7, 12, 7, 7)
        layout.setSpacing(3)
        self.mission_lamps: dict[str, QLabel] = {}

        status_names = list(self.state.mission_status)
        guidance_row = QHBoxLayout()
        guidance_row.setSpacing(3)
        for name in status_names[:3]:
            lamp, label = self._lamp_cell(name)
            self.mission_lamps[name] = label
            guidance_row.addWidget(lamp, 1)
        layout.addLayout(guidance_row)

        terminal_row = QHBoxLayout()
        terminal_row.setSpacing(3)
        for name in status_names[3:]:
            display_name = "근접센서 탐지" if name == "TDD 탐지" else name
            lamp, label = self._lamp_cell(display_name)
            self.mission_lamps[name] = label
            terminal_row.addWidget(lamp, 1)
        layout.addLayout(terminal_row)

        self.intercept_label = QLabel("● 격추 대기")
        self.intercept_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.intercept_label.setStyleSheet(
            f"color:{RED}; background:#120a08; border:1px solid #5f2824; padding:5px;"
        )
        layout.addWidget(self.intercept_label)
        return group

    @staticmethod
    def _lamp_cell(name: str) -> tuple[QWidget, QLabel]:
        cell = QFrame()
        cell.setStyleSheet("background:#090f0b; border:1px solid #2d382f;")
        layout = QVBoxLayout(cell)
        layout.setContentsMargins(3, 2, 3, 3)
        layout.setSpacing(0)
        name_label = QLabel(name)
        name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        name_label.setStyleSheet(
            "color:#8b968c; font-family:'IBM Plex Mono'; font-size:8pt; border:0;"
        )
        lamp = QLabel("●")
        lamp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lamp.setStyleSheet(f"color:{RED}; font-size:14pt; border:0;")
        layout.addWidget(name_label)
        layout.addWidget(lamp)
        return cell, lamp

    def _build_control_group(self) -> QWidget:
        group = QGroupBox("운용 모드 / CONTROLS")
        layout = QGridLayout(group)
        layout.setContentsMargins(7, 12, 7, 7)
        layout.setSpacing(5)
        caption = QLabel("◈ ATR MODE")
        caption.setObjectName("fieldCaption")
        self.mode_label = QLabel("SAFE")
        self.mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mode_label.setStyleSheet(
            f"color:{AMBER}; background:#251c0d; border:1px solid #715822; padding:7px;"
        )
        self.launch_button = QPushButton("발사")
        self.launch_button.setStyleSheet(
            f"background:#12351c; color:{GREEN}; border:1px solid #2e7640;"
            "font-weight:700; padding:7px;"
        )
        self.launch_button.clicked.connect(self._request_launch)
        self.emergency_button = QPushButton("비상모드")
        self.emergency_button.clicked.connect(self._toggle_emergency)
        layout.addWidget(caption, 0, 0)
        layout.addWidget(self.mode_label, 0, 1)
        layout.addWidget(self.launch_button, 1, 0)
        layout.addWidget(self.emergency_button, 1, 1)
        return group

    def _connect_map(self) -> None:
        if self._uses_shared_map:
            assert self._shared_bridge is not None
            self.map_bridge = self._shared_bridge
            self.map_bridge.featureSelected.connect(
                self._on_shared_feature_selected
            )
            self.map_bridge.mapRightClicked.connect(
                self._on_map_right_clicked
            )
            self.map_bridge.featureRightClicked.connect(
                self._on_waypoint_right_clicked
            )
            self._map_loaded = True
            return
        self.map_bridge = FlyMapBridge(self.store, self.state)
        self.map_bridge.threatSelected.connect(self._select_threat)
        self.map_bridge.mapRightClicked.connect(self._on_map_right_clicked)
        self.map_bridge.featureRightClicked.connect(
            self._on_waypoint_right_clicked
        )
        self.map_bridge.mapStatusChanged.connect(self.statusMessage)
        self.web_channel = QWebChannel(self.web_view.page())
        self.web_channel.registerObject("bridge", self.map_bridge)
        self.web_view.page().setWebChannel(self.web_channel)
        self.web_view.loadFinished.connect(self._on_map_loaded)

    def _new_planning_engine(
        self,
        center_latitude: float,
        center_longitude: float,
    ) -> RuleBasedPlanningEngine:
        metadata = self.store.mission_metadata
        raw_arc = metadata.get("arc_search_pattern", {})
        arc = raw_arc if isinstance(raw_arc, dict) else {}
        route = self.store.waypoints_for(1)
        return RuleBasedPlanningEngine(
            center_latitude,
            center_longitude,
            seeker=self.seeker_spec,
            particle_count=max(120, int(arc.get("particle_count", 3_000))),
            rhp_decision_interval_s=max(
                1.0,
                float(arc.get("decision_interval_s", 25.0)),
            ),
            rhp_encounter_sample_count=max(
                16,
                int(arc.get("encounter_sample_count", 64)),
            ),
            rhp_radial_shortlist_count=max(
                1,
                int(arc.get("radial_shortlist_count", 5)),
            ),
            search_radius_m=max(
                self.seeker_spec.track_spacing_m,
                float(metadata.get("search_radius_m", 3_333.3333333333)),
            ),
            minimum_route_updates_before_detection=max(
                0,
                int(
                    metadata.get(
                        "visualization_min_rhp_updates_before_atr",
                        0,
                    )
                ),
            ),
            search_center_latitude=(
                route[0].latitude if route else center_latitude
            ),
            search_center_longitude=(
                route[0].longitude if route else center_longitude
            ),
        )

    def activate(self) -> None:
        if not self._has_activated:
            for vehicle_id, state in self.states.items():
                state.load_mission(self.store, vehicle_id)
            self._has_activated = True
        # ``_refresh_display`` can emit a state while the shared WebView is
        # still displaying PLAN.  That payload is intentionally ignored by
        # the JavaScript FLY renderer, so it must not count as the one plan
        # snapshot sent for this revision.  Force a full Mission Map plan on
        # every activation; otherwise TP and the per-LM waypoint layer can be
        # absent until the operator edits the mission again.
        self._last_sent_plan_key = None
        if self._uses_shared_map:
            self.map_stage.attach_map_view(self.web_view)
            self.web_view.page().runJavaScript(
                "window.setMapMode && window.setMapMode('FLY');"
            )
        elif not self._map_loaded:
            base_url = QUrl.fromLocalFile(str(ASSET_DIR.resolve()) + os.sep)
            self.web_view.setHtml(self.map_html, base_url)
            self._map_loaded = True
        self._active = True
        if not self.update_timer.isActive():
            self.update_timer.start()
        self._tick()
        self.statusMessage.emit(
            f"MISSION MAP READY // {self.provider_name} // UI 내부 모의 데이터"
        )

    def deactivate(self) -> None:
        self._active = False
        self.update_timer.stop()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.update_timer.stop()
        self._planning_executor.shutdown(
            wait=False,
            cancel_futures=True,
        )
        super().closeEvent(event)

    def _on_map_loaded(self, ok: bool) -> None:
        if ok:
            self._last_sent_plan_key = None
            self.map_bridge.emit_state()
        else:
            self.statusMessage.emit("MISSION MAP 지도를 불러오지 못했습니다.")

    def _on_plan_changed(self) -> None:
        self._plan_render_revision += 1
        self._cached_plan_key = None
        self._cached_plan_payload = None
        self._last_sent_plan_key = None
        preserve_execution = self._preserve_execution_on_next_plan_change
        self._preserve_execution_on_next_plan_change = False
        manually_applied = set(self._manual_route_applied_vehicle_ids)
        self._manual_route_applied_vehicle_ids.clear()
        for vehicle_id, state in self.states.items():
            if preserve_execution and manually_applied:
                # Edited routes have already been applied through
                # ``apply_manual_runtime_route`` and every other LM must keep
                # its own current RHP path.  Refresh readiness only; replacing
                # any state from the store here would collapse live routes
                # back onto their original PLAN definitions.
                state.sync_plan_readiness(self.store, vehicle_id)
            elif preserve_execution and state.mission_launched:
                state.update_route_from_store(self.store, vehicle_id)
            else:
                state.load_mission(self.store, vehicle_id)
        if not preserve_execution:
            self._planning_generation += 1
            self._planning_future = None
            center = self.store.sites.get("GCS")
            center_latitude = (
                center.latitude if center is not None else self.state.center_latitude
            )
            center_longitude = (
                center.longitude if center is not None else self.state.center_longitude
            )
            self.planning_engine = self._new_planning_engine(
                center_latitude,
                center_longitude,
            )
            self._initial_rhp_precompute_requested = False
            self._planning_future_initial_precompute = False
            self._last_planning_cycle_s = -1e9
            self._last_planning_search_elapsed_s = -1e9
            self._planning_payload = {
                "model": MODEL_NAME,
                "status": "MISSION RESET",
                "sensor": self.seeker_spec.display_dict(),
            }
        self._refresh_display()
        if self._map_loaded:
            self._emit_map_state()

    def _run_planning_cycle(self, *, force: bool = False) -> None:
        if force:
            self._collect_planning_result(wait=True)
        elif self._planning_future is not None:
            return

        wall_elapsed_s = self.states[1].elapsed_s
        canonical_state = self.states[1]
        engagement_track_id = next(
            (
                state.selected_track_id
                for state in self.states.values()
                if state.engagement_requested and not state.target_destroyed
            ),
            None,
        )
        fleet_launched = all(
            state.mission_launched for state in self.states.values()
        )
        search_active = any(
            state.search_started for state in self.states.values()
        )
        initial_prefix_precompute = bool(
            engagement_track_id is None
            and fleet_launched
            and not search_active
            and not self._initial_rhp_precompute_requested
        )
        if (
            engagement_track_id is None
            and not search_active
            and not initial_prefix_precompute
        ):
            # Before TP, the sole permitted planner operation is one t=0
            # initial-prefix cache.  Ingress must not advance the PF, create
            # seeker observations, or consume the 25-second RHP clock.
            return

        if not force:
            if search_active:
                if (
                    canonical_state.search_elapsed_s
                    - self._last_planning_search_elapsed_s
                    < 1.0
                ):
                    return
            elif wall_elapsed_s - self._last_planning_cycle_s < 1.0:
                return

        self._last_planning_cycle_s = wall_elapsed_s
        planning_elapsed_s = (
            0.0
            if initial_prefix_precompute
            else canonical_state.search_elapsed_s
        )
        if initial_prefix_precompute:
            self._initial_rhp_precompute_requested = True
        elif search_active:
            self._last_planning_search_elapsed_s = planning_elapsed_s
        vehicles = []
        for vehicle_id, state in self.states.items():
            route = self.store.waypoints_for(vehicle_id)
            rally = route[0] if route else None
            if initial_prefix_precompute and rally is None:
                self._initial_rhp_precompute_requested = False
                return
            vehicles.append(
                {
                "vehicle_id": vehicle_id,
                # CPP defines search t=0 with all six LM physically at TP.
                # This synthetic position is used only for a side-effect-free
                # first-prefix cache; the live vehicle remains on ingress.
                "latitude": (
                    rally.latitude
                    if initial_prefix_precompute and rally is not None
                    else state.vehicle.latitude
                ),
                "longitude": (
                    rally.longitude
                    if initial_prefix_precompute and rally is not None
                    else state.vehicle.longitude
                ),
                "altitude_m": (
                    rally.altitude_m
                    if initial_prefix_precompute and rally is not None
                    else state.vehicle.altitude_m
                ),
                "speed_mps": state.vehicle.speed_mps,
                "heading_deg": state.vehicle.heading_deg,
                "mission_launched": state.mission_launched,
                "emergency_mode": state.emergency_mode,
                "flight_phase": state.flight_phase,
                "completed_route_segment_count": (
                    state.completed_route_segment_count
                ),
                "search_started": (
                    state.search_started or initial_prefix_precompute
                ),
                "rhp_preview_only": initial_prefix_precompute,
                "runtime_route_revision": state.runtime_route_revision,
                "runtime_route_update_count": (
                    state.runtime_route_update_count
                ),
                }
            )
        c4i_center = self.store.mission_metadata.get(
            "rally_predicted_target",
            {},
        )
        targets = [
            {
                "track_id": track.track_id,
                "latitude": track.latitude,
                "longitude": track.longitude,
                "altitude_m": track.altitude_m,
                "speed_mps": track.speed_mps,
                "heading_deg": track.heading_deg,
                "position_uncertainty_m": track.position_uncertainty_m,
                "destroyed": track.destroyed,
                "measurement_latitude": (
                    track.latitude
                    if engagement_track_id == track.track_id
                    else float(c4i_center.get("latitude", track.latitude))
                ),
                "measurement_longitude": (
                    track.longitude
                    if engagement_track_id == track.track_id
                    else float(c4i_center.get("longitude", track.longitude))
                ),
                "estimator_speed_mps": (
                    track.speed_mps
                    if engagement_track_id == track.track_id
                    else 0.0
                ),
                "static_surveillance_measurement": (
                    engagement_track_id != track.track_id
                    and bool(c4i_center)
                ),
            }
            for track in canonical_state.threats
        ]
        routes = {
            vehicle_id: state.route_points_payload()
            for vehicle_id, state in self.states.items()
        }
        arguments = {
            "elapsed_s": planning_elapsed_s,
            "vehicles": vehicles,
            "targets": targets,
            "routes": routes,
            "selected_track_id": canonical_state.selected_track_id,
            "engagement_track_id": engagement_track_id,
        }
        if not force:
            self._planning_future_generation = self._planning_generation
            self._planning_future_elapsed_s = planning_elapsed_s
            self._planning_future_initial_precompute = (
                initial_prefix_precompute
            )
            self._planning_future = self._planning_executor.submit(
                self.planning_engine.update,
                **arguments,
            )
            return
        try:
            result = self.planning_engine.update(**arguments)
        except Exception as error:  # Keep the last valid route on planner failure.
            if initial_prefix_precompute:
                self._initial_rhp_precompute_requested = False
            self._planning_payload = {
                **self._planning_payload,
                "status": "PLANNER ERROR",
                "error": str(error),
            }
            return

        self._apply_planning_result(
            result,
            planning_elapsed_s,
            initial_prefix_precompute=initial_prefix_precompute,
        )

    def _collect_planning_result(self, *, wait: bool = False) -> None:
        future = self._planning_future
        if future is None or (not wait and not future.done()):
            return
        generation = self._planning_future_generation
        simulation_elapsed_s = self._planning_future_elapsed_s
        initial_prefix_precompute = (
            self._planning_future_initial_precompute
        )
        self._planning_future = None
        self._planning_future_initial_precompute = False
        try:
            result = future.result()
        except Exception as error:  # Keep the last valid route on planner failure.
            if initial_prefix_precompute:
                self._initial_rhp_precompute_requested = False
            if generation == self._planning_generation:
                self._planning_payload = {
                    **self._planning_payload,
                    "status": "PLANNER ERROR",
                    "error": str(error),
                }
            return
        if generation != self._planning_generation:
            return
        self._apply_planning_result(
            result,
            simulation_elapsed_s,
            initial_prefix_precompute=initial_prefix_precompute,
        )

    def _apply_planning_result(
        self,
        result,
        simulation_elapsed_s: float,
        *,
        initial_prefix_precompute: bool = False,
    ) -> None:
        canonical_state = self.states[1]

        for vehicle_id, solution in result.intercepts.items():
            state = self.states.get(vehicle_id)
            if state is not None:
                state.set_external_intercept(solution)
        queued_vehicle_ids = []
        precomputed_vehicle_ids = []
        manual_hold_vehicle_ids = []
        for vehicle_id, points in result.route_updates.items():
            state = self.states.get(vehicle_id)
            if state is None:
                continue
            if state.manual_route_hold_active:
                manual_hold_vehicle_ids.append(vehicle_id)
                continue
            if state.queue_runtime_route(result.revision, points):
                queued_vehicle_ids.append(vehicle_id)
                if (
                    state.pending_runtime_route_revision == result.revision
                    and state.runtime_route_revision != result.revision
                ):
                    precomputed_vehicle_ids.append(vehicle_id)
        self._planning_payload = {
            **result.render_dict(),
            "status": "ACTIVE",
            "update_interval_s": 1.0,
            "simulation_elapsed_s": simulation_elapsed_s,
            "queued_vehicle_ids": queued_vehicle_ids,
            "precomputed_vehicle_ids": precomputed_vehicle_ids,
            "initial_prefix_precomputed": initial_prefix_precompute,
            "manual_hold_vehicle_ids": manual_hold_vehicle_ids,
        }
        if queued_vehicle_ids:
            vehicles_text = ", ".join(
                f"LM-{vehicle_id:02d}" for vehicle_id in queued_vehicle_ids
            )
            decision_interval_s = result.planner["decision_interval_s"]
            if set(precomputed_vehicle_ids) == set(queued_vehicle_ids):
                self.statusMessage.emit(
                    "CPP TP t=0 RHP 초기 PREFIX 캐시 완료 // "
                    f"{vehicles_text} // PF·센서·25초 시계 미개시"
                )
            else:
                self.statusMessage.emit(
                    "RHP 확률경로 적용 // "
                    f"{vehicles_text} // {decision_interval_s:g}초 ACTION PREFIX"
                )
        elif manual_hold_vehicle_ids:
            vehicles_text = ", ".join(
                f"LM-{vehicle_id:02d}" for vehicle_id in manual_hold_vehicle_ids
            )
            self.statusMessage.emit(
                f"수동 경로 보호 중 // {vehicles_text} // RHP 후보는 평가만 수행"
            )
        if result.detections and not any(
            state.target_detected for state in self.states.values()
        ):
            preferred = next(
                (
                    detection
                    for detection in result.detections
                    if int(detection["track_id"])
                    == canonical_state.selected_track_id
                ),
                result.detections[0],
            )
            self._engage_detected_target(
                int(preferred["track_id"]),
                int(preferred["vehicle_id"]),
                automatic=True,
                run_planning_cycle=False,
            )

    def _tick(self) -> None:
        if not self._active:
            return
        self._collect_planning_result()
        for state in self.states.values():
            state.tick()
        self._run_planning_cycle()
        now = time.monotonic()
        if now - self._last_console_refresh_at >= 0.20:
            for vehicle_id, state in self.states.items():
                state.sync_plan_readiness(self.store, vehicle_id)
            self._refresh_display(refresh_seeker=False)
            self._last_console_refresh_at = now
        if now - self._last_seeker_refresh_at >= 0.10:
            self.map_stage.refresh_seeker()
            self._last_seeker_refresh_at = now
        if now - self._last_map_emit_at >= 0.10:
            self._emit_map_state()
            self._last_map_emit_at = now
        if self.state.target_destroyed and not self._reported_shutdown:
            self._reported_shutdown = True
            self.statusMessage.emit(
                f"격추 성공 // {self.state.vehicle.code} 및 지정 표적 "
                "중첩 지점 SHUT DOWN"
            )
        elif not self.state.target_destroyed:
            self._reported_shutdown = False

    def _emit_map_state(self) -> None:
        if self._uses_shared_map:
            has_runtime_preview = bool(self._pending_runtime_routes)
            display_store = (
                self._pending_mission
                if self._pending_dirty and not has_runtime_preview
                else self.store
            )
            render_state = self.state.render_dict(
                display_store,
                include_plan=False,
                include_flight_path=False,
            )
            plan_key = (self._plan_render_revision, self._pending_dirty)
            if self._cached_plan_key != plan_key:
                self._cached_plan_payload = build_mission_map_plan_payload(
                    self.store,
                    display_store,
                    self._pending_dirty and not has_runtime_preview,
                )
                if has_runtime_preview:
                    self._cached_plan_payload["pending_edit"] = True
                self._cached_plan_key = plan_key
            if self._last_sent_plan_key != plan_key:
                # Send the low-rate mission geometry through its own durable
                # browser channel.  Embedding PLAN in one telemetry frame was
                # unsafe: requestAnimationFrame coalescing could replace that
                # frame with the next position-only update before rendering,
                # making TP and every planned RHP route disappear.
                plan_payload = json.dumps(
                    json_safe_payload(self._cached_plan_payload or {}),
                    ensure_ascii=False,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                plan_argument = json.dumps(
                    plan_payload,
                    ensure_ascii=False,
                )
                self.web_view.page().runJavaScript(
                    "window.setFlyPlan && "
                    f"window.setFlyPlan({plan_argument});"
                )
                self._last_sent_plan_key = plan_key
            render_state["plan_revision"] = self._plan_render_revision
            vehicle_payloads = []
            for vehicle_id, state in self.states.items():
                raw_path = state.flight_path[-1200:]
                path_key = (
                    len(raw_path),
                    (
                        round(float(raw_path[-1]["latitude"]), 7),
                        round(float(raw_path[-1]["longitude"]), 7),
                    )
                    if raw_path
                    else None,
                )
                cached_path = self._flight_path_payload_cache.get(vehicle_id)
                if cached_path is None or cached_path[0] != path_key:
                    cached_path = (
                        path_key,
                        simplify_flight_path(raw_path),
                    )
                    self._flight_path_payload_cache[vehicle_id] = cached_path
                vehicle_payloads.append(
                    {
                        **vars(state.vehicle),
                        "flight_path": cached_path[1],
                        "flight_phase": state.flight_phase,
                        "mission_launched": state.mission_launched,
                        "current_waypoint_index": state.current_waypoint_index,
                        "completed_route_segment_count": (
                            state.completed_route_segment_count
                        ),
                        "runtime_route": state.runtime_route_payload(),
                        "runtime_route_revision": state.runtime_route_revision,
                        "runtime_route_update_count": (
                            state.runtime_route_update_count
                        ),
                        "manual_route_preview": self._pending_runtime_routes.get(
                            vehicle_id,
                            [],
                        ),
                        "manual_route_preview_pending": (
                            vehicle_id in self._pending_runtime_routes
                        ),
                        "manual_route_active": state.manual_route_active,
                        "manual_route_hold_remaining_s": (
                            state.manual_route_hold_remaining_s
                        ),
                        "pending_runtime_route": (
                            state.pending_runtime_route_payload()
                        ),
                        "pending_runtime_route_revision": (
                            state.pending_runtime_route_revision
                        ),
                        "engagement_route": state.engagement_route_payload(),
                        "seeker_mode": state.seeker_mode,
                        "detection_source_vehicle_id": (
                            state.detection_source_vehicle_id
                        ),
                        "selected": (
                            self.selected_vehicle_id == vehicle_id
                        ),
                    }
                )
            render_state["vehicles"] = vehicle_payloads
            render_state["selected_vehicle_id"] = self.selected_vehicle_id
            render_state["planning"] = dict(self._planning_payload)
            payload = json.dumps(
                json_safe_payload(render_state),
                ensure_ascii=False,
                separators=(",", ":"),
                allow_nan=False,
            )
            argument = json.dumps(payload, ensure_ascii=False)
            self.web_view.page().runJavaScript(
                f"window.setFlyState && window.setFlyState({argument});"
            )
        elif isinstance(self.map_bridge, FlyMapBridge):
            self.map_bridge.emit_state()

    def _on_shared_feature_selected(self, code: str) -> None:
        if not self._active or not code.startswith("THR-"):
            return
        try:
            track_id = int(code.removeprefix("THR-"))
        except ValueError:
            return
        now = time.monotonic()
        if (
            self._last_map_track_click is not None
            and self._last_map_track_click[0] == track_id
            and now - self._last_map_track_click[1] <= 0.8
        ):
            self._last_map_track_click = None
            self._select_threat(track_id)
            return
        self._last_map_track_click = (track_id, now)
        self.statusMessage.emit(
            f"THR-{track_id} 지정: 같은 표적을 한 번 더 클릭하십시오."
        )

    def _select_threat_from_table(self, row: int, _column: int) -> None:
        item = self.threat_table.item(row, 0)
        if item is None:
            return
        track_id = int(item.data(Qt.ItemDataRole.UserRole))
        self._select_threat(track_id)

    def _select_threat(self, track_id: int) -> None:
        self._engage_detected_target(
            track_id,
            self._nearest_detector_id(track_id),
            automatic=False,
            run_planning_cycle=True,
        )

    def _engage_detected_target(
        self,
        track_id: int,
        detector_id: int | None,
        *,
        automatic: bool,
        run_planning_cycle: bool,
    ) -> None:
        canonical_target = next(
            (
                track
                for track in self.states[1].threats
                if track.track_id == track_id and not track.destroyed
            ),
            None,
        )
        if canonical_target is None:
            self.statusMessage.emit(f"THR-{track_id}는 교전 가능한 표적이 아닙니다.")
            return
        detected_track = (
            canonical_target.latitude,
            canonical_target.longitude,
            canonical_target.altitude_m,
            canonical_target.speed_mps,
            canonical_target.heading_deg,
        )
        # One LM detection becomes the shared ATR track immediately. This
        # avoids six independent stale copies steering to different positions.
        for state in self.states.values():
            local_target = next(
                (
                    track
                    for track in state.threats
                    if track.track_id == track_id and not track.destroyed
                ),
                None,
            )
            if local_target is not None:
                (
                    local_target.latitude,
                    local_target.longitude,
                    local_target.altitude_m,
                    local_target.speed_mps,
                    local_target.heading_deg,
                ) = detected_track
        results = [
            state.designate_threat(
                track_id,
                detector_vehicle_id=detector_id,
            )
            for state in self.states.values()
        ]
        if not any(results):
            return
        if detector_id is not None:
            # Display before the planning cycle so a heavy IMM/PF evaluation
            # cannot delay or hide the operator warning.
            self._show_detection_notice(detector_id, automatic=automatic)
        if run_planning_cycle:
            self._run_planning_cycle(force=True)
        self._refresh_display()
        self._emit_map_state()
        if detector_id is not None:
            self.statusMessage.emit(
                f"THR-{track_id} {'자동' if automatic else '수동'} 탐지 // "
                f"LM-{detector_id:02d} // ATR 협동 교전"
            )
        else:
            self.statusMessage.emit(
                f"THR-{track_id} 선택 // 발사 후 탐지 가능"
            )

    def _nearest_detector_id(self, track_id: int) -> int | None:
        candidates: list[tuple[float, int]] = []
        for vehicle_id, state in self.states.items():
            if not state.mission_launched or state.emergency_mode:
                continue
            target = next(
                (
                    track
                    for track in state.threats
                    if track.track_id == track_id and not track.destroyed
                ),
                None,
            )
            if target is None:
                continue
            candidates.append(
                (
                    horizontal_distance_m(
                        state.vehicle.latitude,
                        state.vehicle.longitude,
                        target.latitude,
                        target.longitude,
                    ),
                    vehicle_id,
                )
            )
        return min(candidates)[1] if candidates else None

    def _show_detection_notice(
        self,
        vehicle_id: int,
        *,
        automatic: bool = True,
    ) -> None:
        if self._detection_dialog is not None:
            self._detection_dialog.close()
        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Icon.Warning)
        dialog.setWindowTitle("탐지 확인")
        dialog.setText(
            "⚠ 표적을 탐지했습니다.\n"
            f"최초 탐지: LM-{vehicle_id:02d}\n"
            f"탐지 방식: {'ATR 자동 탐지' if automatic else 'MITL 수동 지정'}\n"
            "6대 ATR 협동 타격을 시작합니다."
        )
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.setWindowModality(Qt.WindowModality.ApplicationModal)
        dialog.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, True)
        dialog.finished.connect(
            lambda _result, current=dialog: (
                setattr(self, "_detection_dialog", None)
                if self._detection_dialog is current
                else None
            )
        )
        self._detection_dialog = dialog
        dialog.show()

        def bring_to_front() -> None:
            if self._detection_dialog is dialog:
                dialog.raise_()
                dialog.activateWindow()

        QTimer.singleShot(0, bring_to_front)

    def _request_launch(self) -> None:
        launched = [
            state.request_simulated_launch()
            for state in self.states.values()
        ]
        if any(launched):
            self._initial_rhp_precompute_requested = False
            self._last_planning_cycle_s = -1e9
            self._last_planning_search_elapsed_s = -1e9
            self.statusMessage.emit(
                "LM-01~06 일괄 발사 // CPP TP t=0 초기 PREFIX 백그라운드 계산"
            )
        else:
            self.statusMessage.emit(
                "발사할 수 없습니다. 임무 장입 상태 또는 기존 발사 여부를 확인하십시오."
            )
        self._refresh_display()
        self._emit_map_state()
        if any(launched):
            self._run_planning_cycle()

    def _toggle_emergency(self) -> None:
        target_enabled = not self.state.emergency_mode
        for state in self.states.values():
            if state.emergency_mode != target_enabled:
                state.toggle_emergency()
        enabled = target_enabled
        self.statusMessage.emit(
            "비상모드 // 6기 모두 안전지대 중심으로 직선 복귀합니다."
            if enabled
            else "비상모드 해제 // 중단했던 임무 단계를 재개합니다."
        )
        self._refresh_display()
        self._emit_map_state()

    def _stop_engagement(self) -> None:
        for state in self.states.values():
            state.stop_engagement()
        self.statusMessage.emit("MITL 중지 // 교전 중단 후 탐색 경로 복귀")
        self._refresh_display()
        self._emit_map_state()

    def _show_split_seekers(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle("탐색기 화면 // LM-01~06")
        dialog.resize(1180, 720)
        root = QVBoxLayout(dialog)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        command_row = QHBoxLayout()
        active_label = QLabel(
            f"활성 타격기 // LM-{self._split_active_vehicle_id:02d}"
        )
        active_label.setObjectName("dataValue")
        engage_button = QPushButton("격추")
        stop_button = QPushButton("중지")
        engage_button.setStyleSheet(
            """
            QPushButton {
                background:qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #713a34, stop:0.52 #4b2622, stop:1 #281512
                );
                color:#ffd2cb;
                border-top:1px solid #a65b52;
                border-left:1px solid #86453f;
                border-right:2px solid #1b0d0c;
                border-bottom:2px solid #1b0d0c;
                font-weight:700;
                padding:5px 12px;
            }
            QPushButton:hover {
                background:#7b4039;
                color:#fff0ec;
            }
            QPushButton:pressed {
                background:#321714;
            }
            """
        )
        stop_button.setStyleSheet(
            """
            QPushButton {
                background:qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 #454334, stop:0.52 #2d2d24, stop:1 #181a16
                );
                color:#ddd4b2;
                border-top:1px solid #77735b;
                border-left:1px solid #625f4c;
                border-right:2px solid #10120f;
                border-bottom:2px solid #10120f;
                font-weight:700;
                padding:5px 12px;
            }
            QPushButton:hover {
                background:#55513d;
                color:#f1e7bf;
            }
            QPushButton:pressed {
                background:#202018;
            }
            """
        )
        engage_button.setMaximumWidth(90)
        stop_button.setMaximumWidth(90)
        engage_button.clicked.connect(self._split_engage_active)
        stop_button.clicked.connect(self._split_stop_active)
        command_row.addWidget(active_label)
        command_row.addStretch(1)
        command_row.addWidget(engage_button)
        command_row.addWidget(stop_button)
        root.addLayout(command_row)

        grid = QGridLayout()
        grid.setSpacing(6)
        root.addLayout(grid, 1)
        widgets: list[SeekerVideoWidget] = []
        panels: dict[int, QGroupBox] = {}
        for index, vehicle_id in enumerate(SiteStore.VEHICLE_IDS):
            panel = QGroupBox(f"LM-{vehicle_id:02d} // ATR SEEKER")
            panel_layout = QVBoxLayout(panel)
            seeker = SeekerVideoWidget(self.states[vehicle_id])
            seeker.setMinimumSize(300, 190)
            seeker.activated.connect(
                lambda selected_id=vehicle_id: self._activate_split_vehicle(
                    selected_id
                )
            )
            panel_layout.addWidget(seeker)
            grid.addWidget(panel, index // 3, index % 3)
            widgets.append(seeker)
            panels[vehicle_id] = panel
        dialog._seeker_widgets = widgets
        dialog._seeker_panels = panels
        dialog._active_label = active_label
        refresh_timer = QTimer(dialog)
        refresh_timer.setTimerType(Qt.TimerType.PreciseTimer)
        refresh_timer.setInterval(125)
        refresh_timer.timeout.connect(
            lambda: [widget.refresh() for widget in widgets]
        )
        refresh_timer.start()
        dialog._refresh_timer = refresh_timer
        dialog.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        dialog.show()
        self._split_seeker_dialog = dialog
        self._refresh_split_selection()

    def _activate_split_vehicle(self, vehicle_id: int) -> None:
        self._split_active_vehicle_id = int(vehicle_id)
        self.vehicleSelectionRequested.emit(self._split_active_vehicle_id)
        self._refresh_split_selection()
        self.statusMessage.emit(
            f"LM-{vehicle_id:02d} 탐색기 활성화 // MITL 명령 대상"
        )

    def _refresh_split_selection(self) -> None:
        dialog = getattr(self, "_split_seeker_dialog", None)
        if dialog is None:
            return
        dialog._active_label.setText(
            f"활성 타격기 // LM-{self._split_active_vehicle_id:02d}"
        )
        for vehicle_id, panel in dialog._seeker_panels.items():
            panel.setStyleSheet(
                "QGroupBox { border:2px solid #9cff00; color:#9cff00; }"
                if vehicle_id == self._split_active_vehicle_id
                else "QGroupBox { border:1px solid #3d4a40; color:#b7c1b8; }"
            )

    def _split_engage_active(self) -> None:
        vehicle_id = self._split_active_vehicle_id
        state = self.states[vehicle_id]
        if not state.mission_launched:
            self.statusMessage.emit(
                f"LM-{vehicle_id:02d} 미발사 // 격추 명령 거부"
            )
            return
        target_id = state.selected_track_id
        state.designate_threat(target_id)
        self.statusMessage.emit(
            f"MITL 격추 명령 // LM-{vehicle_id:02d} → THR-{target_id}"
        )

    def _split_stop_active(self) -> None:
        vehicle_id = self._split_active_vehicle_id
        started = self.states[vehicle_id].request_safe_return_via_waypoint()
        self.statusMessage.emit(
            (
                f"MITL 중지 명령 // LM-{vehicle_id:02d} 최근접 전용 "
                "웨이포인트 경유 후 안전구역 복귀"
            )
            if started
            else f"LM-{vehicle_id:02d} 복귀 명령 거부 // 미발사 또는 비상모드"
        )
        self._refresh_display()
        self._emit_map_state()

    @staticmethod
    def _copy_route(route: list[dict]) -> list[dict]:
        return [
            {
                "latitude": float(point["latitude"]),
                "longitude": float(point["longitude"]),
                "altitude_m": max(0.0, float(point.get("altitude_m", 600.0))),
                "code": str(point.get("code") or f"MWP{index:03d}"),
            }
            for index, point in enumerate(route, start=1)
        ]

    def _effective_route_for_editing(self, vehicle_id: int) -> list[dict]:
        pending = self._pending_runtime_routes.get(vehicle_id)
        if pending is not None:
            return self._copy_route(pending)
        state = self.states[vehicle_id]
        runtime = state.runtime_route_payload()
        if runtime:
            return self._copy_route(runtime)
        return [
            {
                "latitude": point.latitude,
                "longitude": point.longitude,
                "altitude_m": point.altitude_m,
                "code": point.code,
            }
            for point in self.store.waypoints_for(vehicle_id)
        ]

    @staticmethod
    def _renumber_manual_route(route: list[dict]) -> None:
        for sequence, point in enumerate(route, start=1):
            point["code"] = f"MWP{sequence:03d}"

    def _show_waypoint_context_menu(self, position=None) -> None:
        if not self._active:
            return
        if time.monotonic() < self._context_menu_block_until:
            return

        remembered = self._pending_context_position
        if remembered is not None and time.monotonic() - remembered[3] <= 2.5:
            latitude, longitude, altitude = remembered[:3]
        else:
            display_vehicle_id = (
                1 if self.selected_vehicle_id == 0 else self.selected_vehicle_id
            )
            vehicle = self.states[display_vehicle_id].vehicle
            latitude, longitude = destination_position(
                vehicle.latitude,
                vehicle.longitude,
                850.0,
                vehicle.heading_deg,
            )
            altitude = max(600.0, vehicle.altitude_m)

        menu = QMenu(self)
        action_commands: dict[object, tuple[str, int]] = {}
        nearest_by_vehicle = {}
        remembered_feature = self._pending_context_feature
        context_key = (
            remembered_feature[0]
            if remembered_feature is not None
            and time.monotonic() - remembered_feature[1] <= 2.5
            else None
        )
        vehicle_ids = (
            SiteStore.VEHICLE_IDS
            if self.selected_vehicle_id == 0
            else (self.selected_vehicle_id,)
        )
        if any(
            self.states[vehicle_id].engagement_requested
            for vehicle_id in vehicle_ids
        ):
            self.statusMessage.emit(
                "ATR 교전 중에는 탐색 웨이포인트를 수정할 수 없습니다."
            )
            return
        editable_routes = {
            vehicle_id: self._effective_route_for_editing(vehicle_id)
            for vehicle_id in vehicle_ids
        }
        for vehicle_id in vehicle_ids:
            route = editable_routes[vehicle_id]
            exact_index = runtime_route_index_for_context_key(
                route,
                vehicle_id,
                context_key,
            )
            nearest_index = (
                nearest_runtime_route_index_within(
                    route,
                    latitude,
                    longitude,
                )
                if route and context_key is None
                else None
            )
            nearest_by_vehicle[vehicle_id] = (
                exact_index if exact_index is not None else nearest_index
            )

        if self.selected_vehicle_id == 0:
            add_menu = menu.addMenu("웨이포인트 추가")
            delete_menu = menu.addMenu("웨이포인트 삭제")
            for vehicle_id in vehicle_ids:
                add_action = add_menu.addAction(f"LM-{vehicle_id:02d}")
                delete_action = delete_menu.addAction(f"LM-{vehicle_id:02d}")
                delete_action.setEnabled(
                    nearest_by_vehicle[vehicle_id] is not None
                    and len(editable_routes[vehicle_id]) > 1
                )
                action_commands[add_action] = ("ADD", vehicle_id)
                action_commands[delete_action] = ("DELETE", vehicle_id)
        else:
            vehicle_id = self.selected_vehicle_id
            menu.setTitle(f"LM-{vehicle_id:02d} 경로 편집")
            add_action = menu.addAction("웨이포인트 추가")
            delete_action = menu.addAction("웨이포인트 삭제")
            delete_action.setEnabled(
                nearest_by_vehicle[vehicle_id] is not None
                and len(editable_routes[vehicle_id]) > 1
            )
            action_commands[add_action] = ("ADD", vehicle_id)
            action_commands[delete_action] = ("DELETE", vehicle_id)
        source = self.sender()
        global_position = QCursor.pos()
        if position is not None and isinstance(source, QWidget):
            global_position = source.mapToGlobal(position)
        selected_action = menu.exec(global_position)
        self._context_menu_block_until = time.monotonic() + 0.45
        if selected_action is None or selected_action not in action_commands:
            return
        command, vehicle_id = action_commands[selected_action]
        route = editable_routes[vehicle_id]
        if vehicle_id not in self._pending_runtime_routes:
            self._pending_runtime_originals[vehicle_id] = self._copy_route(route)
        delete_index = nearest_by_vehicle[vehicle_id]
        if command == "ADD":
            route.append(
                {
                    "latitude": float(latitude),
                    "longitude": float(longitude),
                    "altitude_m": max(600.0, float(altitude)),
                    "code": "",
                }
            )
            self._renumber_manual_route(route)
            point = route[-1]
            self._pending_runtime_routes[vehicle_id] = route
            self._pending_dirty = True
            self._plan_render_revision += 1
            self._cached_plan_key = None
            self.statusMessage.emit(
                f"LM-{vehicle_id:02d} {point['code']} 추가 대기 // "
                "'임무지도 수정'을 누르면 반영"
            )
        elif command == "DELETE" and delete_index is not None and len(route) > 1:
            deleted = str(route[delete_index].get("code", delete_index + 1))
            route.pop(delete_index)
            self._renumber_manual_route(route)
            self._pending_runtime_routes[vehicle_id] = route
            self._pending_dirty = True
            self._plan_render_revision += 1
            self._cached_plan_key = None
            self.statusMessage.emit(
                f"LM-{vehicle_id:02d} {deleted} 삭제 대기 // "
                "'임무지도 수정'을 누르면 반영"
            )
        self._pending_context_feature = None
        self._emit_map_state()

    def _remember_map_context_position(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        self._pending_context_position = (
            latitude,
            longitude,
            altitude,
            time.monotonic(),
        )

    def _on_map_right_clicked(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        """Open the Qt waypoint menu when the web map reports a right-click."""
        self._pending_context_feature = None
        self._remember_map_context_position(latitude, longitude, altitude)
        QTimer.singleShot(0, self._show_waypoint_context_menu)

    def _on_waypoint_right_clicked(
        self,
        feature_key: str,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        """Open a menu tied to one exact vehicle waypoint marker."""
        self._remember_map_context_position(latitude, longitude, altitude)
        self._pending_context_feature = (
            str(feature_key),
            time.monotonic(),
        )
        QTimer.singleShot(0, self._show_waypoint_context_menu)

    def _apply_pending_mission(self) -> None:
        if not self._pending_dirty:
            self.statusMessage.emit("대기 중인 웨이포인트 수정이 없습니다.")
            return
        if self._pending_runtime_routes:
            hold_duration_s = max(
                1.0,
                float(
                    self.store.mission_metadata.get(
                        "manual_route_hold_s",
                        MANUAL_ROUTE_HOLD_S,
                    )
                ),
            )
            applied_vehicle_ids: list[int] = []
            for vehicle_id, route in sorted(self._pending_runtime_routes.items()):
                state = self.states[vehicle_id]
                if not state.apply_manual_runtime_route(
                    route,
                    hold_duration_s=hold_duration_s,
                ):
                    continue
                applied_vehicle_ids.append(vehicle_id)
                self.store.vehicle_waypoints[vehicle_id] = [
                    MissionPoint(
                        latitude=float(point["latitude"]),
                        longitude=float(point["longitude"]),
                        altitude_m=max(
                            0.0,
                            float(point.get("altitude_m", 600.0)),
                        ),
                        code=f"WP{sequence:03d}",
                        label=f"LM-{vehicle_id:02d} Waypoint {sequence}",
                        point_type="WAYPOINT",
                        sequence=sequence,
                    )
                    for sequence, point in enumerate(route, start=1)
                ]
            if not applied_vehicle_ids:
                self.statusMessage.emit("적용 가능한 웨이포인트 수정이 없습니다.")
                return
            self._pending_dirty = False
            self._pending_runtime_routes.clear()
            self._pending_runtime_originals.clear()
            self._manual_route_applied_vehicle_ids = set(applied_vehicle_ids)
            self._preserve_execution_on_next_plan_change = True
            self.store.notify()
            vehicles_text = ", ".join(
                f"LM-{vehicle_id:02d}" for vehicle_id in applied_vehicle_ids
            )
            self.statusMessage.emit(
                f"Mission Map 수동 경로 반영 // {vehicles_text} // "
                f"자동 RHP 덮어쓰기 {hold_duration_s:g}초 보호"
            )
            return
        self._pending_dirty = False
        self._preserve_execution_on_next_plan_change = True
        self.store.replace_from(self._pending_mission)
        self.statusMessage.emit(
            "Mission Map 웨이포인트 수정 반영 완료 // 6기 임무 재동기화"
        )

    @staticmethod
    def _set_lamp(label: QLabel, enabled: bool) -> None:
        label.setStyleSheet(
            f"color:{GREEN if enabled else RED}; font-size:14pt; border:0;"
        )

    def _refresh_display(self, *, refresh_seeker: bool = True) -> None:
        self._refresh_threat_table()
        self._refresh_information()

        for name, ready in self.state.readiness.items():
            self._set_lamp(self.readiness_lamps[name], ready)
        ready = self.state.launch_ready and not self.state.mission_launched
        if self.state.mission_launched:
            readiness_text = "● 발사 완료"
            readiness_green = True
        else:
            readiness_text = "● 발사 가능" if ready else "● 발사 불가"
            readiness_green = ready
        self.launch_ready_label.setText(readiness_text)
        self.launch_ready_label.setStyleSheet(
            f"color:{GREEN if readiness_green else RED}; background:"
            f"{'#0b1c10' if readiness_green else '#120a08'}; border:1px solid "
            f"{'#2e7640' if readiness_green else '#5f2824'}; padding:5px;"
        )

        for name, complete in self.state.mission_status.items():
            self._set_lamp(self.mission_lamps[name], complete)
        success = self.state.engagement_success
        if success:
            intercept_text = "● 격추 성공"
            intercept_color = GREEN
        elif self.state.engagement_requested:
            intercept_text = "● 교전 중"
            intercept_color = AMBER
        else:
            intercept_text = "● 격추 대기"
            intercept_color = RED
        self.intercept_label.setText(intercept_text)
        self.intercept_label.setStyleSheet(
            f"color:{intercept_color}; background:#120f08; border:1px solid "
            f"{'#2e7640' if success else '#715822'}; padding:5px;"
        )

        mode = self.state.automatic_mode
        self.mode_label.setText(mode)
        self.mode_label.setStyleSheet(
            f"color:{GREEN if mode == 'ARM' else AMBER}; background:"
            f"{'#0d2c15' if mode == 'ARM' else '#251c0d'}; border:1px solid "
            f"{'#2e7640' if mode == 'ARM' else '#715822'}; padding:7px;"
        )
        self.launch_button.setEnabled(self.state.can_press_launch)
        self.launch_button.setText(
            "발사 완료" if self.state.mission_launched else "발사"
        )
        self.launch_button.setStyleSheet(
            (
                f"background:#12351c; color:{GREEN}; border:1px solid #2e7640;"
                if self.state.can_press_launch
                else "background:#111713; color:#586259; border:1px solid #29322b;"
            )
            + "font-weight:700; padding:7px;"
        )
        self.emergency_button.setText(
            "비상 해제" if self.state.emergency_mode else "비상모드"
        )
        if refresh_seeker:
            self.map_stage.refresh_seeker()

    def select_vehicle(self, vehicle_id: int) -> None:
        self.selected_vehicle_id = int(vehicle_id)
        display_id = 1 if vehicle_id == 0 else int(vehicle_id)
        self.state = self.states[display_id]
        self.map_stage.seeker_video.state = self.state
        if isinstance(self.map_bridge, FlyMapBridge):
            self.map_bridge.state = self.state
        self._refresh_display()
        self._emit_map_state()

    def _refresh_threat_table(self) -> None:
        self._refreshing_table = True
        blocker = QSignalBlocker(self.threat_table)
        self.threat_table.setUpdatesEnabled(False)
        try:
            self.threat_table.setRowCount(len(self.state.threats))
            selected_row = -1
            for row, track in enumerate(self.state.threats):
                values = (
                    (
                        f"{track.track_id}/{track.target_type}-{track.country}"
                        + (" X" if track.destroyed else "")
                    ),
                    f"{track.speed_mps:.0f} m/s",
                    f"{track.heading_deg:03.0f}°",
                    track.first_tracked_text,
                )
                for column, value in enumerate(values):
                    item = self.threat_table.item(row, column)
                    if item is None:
                        item = QTableWidgetItem()
                        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                        self.threat_table.setItem(row, column, item)
                    item.setText(value)
                    if column == 0:
                        item.setData(Qt.ItemDataRole.UserRole, track.track_id)
                if track.track_id == self.state.selected_track_id:
                    selected_row = row
            if selected_row >= 0:
                self.threat_table.selectRow(selected_row)
        finally:
            self.threat_table.setUpdatesEnabled(True)
            del blocker
            self._refreshing_table = False
    def _refresh_information(self) -> None:
        vehicle = self.state.vehicle
        self.vehicle_title_label.setText(vehicle.code)
        vehicle_values = {
            "altitude": f"{vehicle.altitude_m:,.1f} m",
            "latitude": f"{vehicle.latitude:.5f}",
            "longitude": f"{vehicle.longitude:.5f}",
            "speed": f"{vehicle.speed_mps:.1f} m/s",
            "heading": f"{vehicle.heading_deg:03.0f}°",
        }
        for name, value in vehicle_values.items():
            self.vehicle_values[name].setText(value)

        target = self.state.selected_threat
        distance_m = (
            horizontal_distance_m(
                vehicle.latitude,
                vehicle.longitude,
                target.latitude,
                target.longitude,
            )
            if target
            else 0.0
        )
        eta_s = distance_m / LM_CRUISE_SPEED_MPS if target else 0.0
        eta_text = (
            f"{int(eta_s // 60):02d}:{int(eta_s % 60):02d}"
            if target
            else "--"
        )
        target_values = {
            "altitude": f"{target.altitude_m:,.0f} m" if target else "--",
            "latitude": f"{target.latitude:.5f}" if target else "--",
            "longitude": f"{target.longitude:.5f}" if target else "--",
            "speed": f"{target.speed_mps:.1f} m/s" if target else "--",
            "heading": f"{target.heading_deg:03.0f}°" if target else "--",
            "distance": f"{distance_m / 1000:.1f} km" if target else "--",
            "eta": eta_text,
        }
        vehicle_values["distance"] = (
            f"{distance_m / 1000:.1f} km" if target else "--"
        )
        vehicle_values["eta"] = eta_text
        for name, value in vehicle_values.items():
            self.vehicle_values[name].setText(value)
        for name, value in target_values.items():
            self.target_values[name].setText(value)

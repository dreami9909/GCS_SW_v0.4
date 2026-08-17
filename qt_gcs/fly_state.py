from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field

from .site_store import SiteStore
from .track_predictor import ConstantVelocityTrackPredictor


EARTH_METERS_PER_DEGREE = 111_320.0
LM_CRUISE_SPEED_MPS = 160.0 / 3.6
LM_INGRESS_SPEED_MPS = 100_000.0 / (8.0 * 60.0)
LM_SEARCH_SPEED_MPS = 100.0 / 3.6
PREDICTION_HORIZON_S = 8.0 * 60.0
SIMULATION_TIME_SCALE = 3.0
DEMO_FLIGHT_TIME_SCALE = SIMULATION_TIME_SCALE
DEMO_GROUND_TIME_SCALE = SIMULATION_TIME_SCALE
INITIAL_GUIDANCE_SPEED_MPS = 160.0 / 3.6
MIDCOURSE_GUIDANCE_SPEED_MPS = 160.0 / 3.6
TERMINAL_GUIDANCE_SPEED_MPS = 200.0 / 3.6
TERMINAL_SPEED_RAMP_S = 5.0
TERMINAL_CONTACT_DWELL_S = 0.75
INTERCEPT_VISUAL_DURATION_S = PREDICTION_HORIZON_S / DEMO_FLIGHT_TIME_SCALE
INITIAL_GUIDANCE_DURATION_S = 8.0
# Keep MIDCOURSE visible for three wall-clock seconds in the 3x demo.  This
# makes the yellow ATR track state operationally distinguishable from the red
# terminal state even when the LM reaches the horizontal intercept point early.
MINIMUM_MIDCOURSE_DURATION_S = 9.0
# Keep a visible midcourse tracking interval inside the nominal 1.2 km
# seeker ground strip before terminal homing begins.
TERMINAL_ENTRY_RADIUS_M = 350.0
MIDCOURSE_TIME_SCALE = SIMULATION_TIME_SCALE

DEMO_THREAT_SPECS = (
    # track, type, country, platform, range, bearing, speed, course, age
    (101, "TEL", "RU", "ISKANDER-E TEL", 36_000.0, 58.0, 40 / 3.6, 224.0, 94),
    (204, "TANK", "RU", "T-90S", 32_000.0, 112.0, 28 / 3.6, 291.0, 61),
)


def horizontal_distance_m(
    latitude_1: float,
    longitude_1: float,
    latitude_2: float,
    longitude_2: float,
) -> float:
    mean_latitude = math.radians((latitude_1 + latitude_2) / 2.0)
    north_m = (latitude_2 - latitude_1) * EARTH_METERS_PER_DEGREE
    east_m = (
        (longitude_2 - longitude_1)
        * EARTH_METERS_PER_DEGREE
        * math.cos(mean_latitude)
    )
    return math.hypot(east_m, north_m)


def bearing_deg(
    latitude_1: float,
    longitude_1: float,
    latitude_2: float,
    longitude_2: float,
) -> float:
    mean_latitude = math.radians((latitude_1 + latitude_2) / 2.0)
    north_m = (latitude_2 - latitude_1) * EARTH_METERS_PER_DEGREE
    east_m = (
        (longitude_2 - longitude_1)
        * EARTH_METERS_PER_DEGREE
        * math.cos(mean_latitude)
    )
    return (math.degrees(math.atan2(east_m, north_m)) + 360.0) % 360.0


def offset_position(
    latitude: float,
    longitude: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    latitude_out = latitude + north_m / EARTH_METERS_PER_DEGREE
    longitude_scale = max(
        10_000.0,
        EARTH_METERS_PER_DEGREE * math.cos(math.radians(latitude)),
    )
    longitude_out = longitude + east_m / longitude_scale
    return latitude_out, longitude_out


def destination_position(
    latitude: float,
    longitude: float,
    distance_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    heading = math.radians(heading_deg)
    return offset_position(
        latitude,
        longitude,
        math.sin(heading) * distance_m,
        math.cos(heading) * distance_m,
    )


def constant_velocity_intercept_time(
    relative_east_m: float,
    relative_north_m: float,
    relative_up_m: float,
    target_velocity_east_mps: float,
    target_velocity_north_mps: float,
    target_velocity_up_mps: float,
    pursuer_speed_mps: float,
    max_horizon_s: float = PREDICTION_HORIZON_S,
) -> tuple[float, bool]:
    """Return reachable intercept time for constant target velocity.

    The quadratic solves ``|relative_position + target_velocity * t| =
    pursuer_speed * t``. The boolean is false when the mathematical solution
    is unavailable or beyond the configured prediction horizon.
    """
    pursuer_speed_mps = max(1.0, float(pursuer_speed_mps))
    relative = (
        float(relative_east_m),
        float(relative_north_m),
        float(relative_up_m),
    )
    velocity = (
        float(target_velocity_east_mps),
        float(target_velocity_north_mps),
        float(target_velocity_up_mps),
    )
    a = sum(component * component for component in velocity) - (
        pursuer_speed_mps * pursuer_speed_mps
    )
    b = 2.0 * sum(
        position * speed
        for position, speed in zip(relative, velocity)
    )
    c = sum(component * component for component in relative)
    positive_roots: list[float] = []
    if abs(a) < 1e-9:
        if abs(b) > 1e-9:
            root = -c / b
            if root > 0.0:
                positive_roots.append(root)
    else:
        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0.0:
            root_scale = math.sqrt(discriminant)
            for root in (
                (-b - root_scale) / (2.0 * a),
                (-b + root_scale) / (2.0 * a),
            ):
                if root > 0.0:
                    positive_roots.append(root)
    if not positive_roots:
        return float(max_horizon_s), False
    intercept_time_s = min(positive_roots)
    reachable = intercept_time_s <= max_horizon_s
    return min(intercept_time_s, float(max_horizon_s)), reachable


@dataclass
class VehicleTrack:
    code: str = "LM-01"
    latitude: float = 37.3422
    longitude: float = 127.9202
    altitude_m: float = 51.0
    speed_mps: float = LM_CRUISE_SPEED_MPS
    heading_deg: float = 0.0


@dataclass
class ThreatTrack:
    track_id: int
    target_type: str
    country: str
    platform_name: str
    latitude: float
    longitude: float
    altitude_m: float
    speed_mps: float
    heading_deg: float
    first_tracked_at: float
    destroyed: bool = False
    position_uncertainty_m: float = 500.0
    source: str = "LOCAL"
    motion_profile: list[dict] = field(default_factory=list)

    @property
    def code(self) -> str:
        return f"THR-{self.track_id}"

    @property
    def first_tracked_text(self) -> str:
        return time.strftime("%H:%M:%S", time.localtime(self.first_tracked_at))


@dataclass
class FlyState:
    """Local-only mission demonstration state.

    No method in this class transmits MAVLink or an external launch command.
    The accelerated motion is a visualization timeline; displayed speed and
    ETA remain based on LM_CRUISE_SPEED_MPS.
    """

    center_latitude: float
    center_longitude: float
    vehicle: VehicleTrack
    threats: list[ThreatTrack]
    selected_track_id: int = 101
    readiness: dict[str, bool] = field(
        default_factory=lambda: {
            "AVS": True,
            "LC": False,
            "RDR": False,
            "DL": True,
            "GCS": False,
        }
    )
    mission_status: dict[str, bool] = field(
        default_factory=lambda: {
            "초기유도": False,
            "중기유도": False,
            "종말유도": False,
            "셔터 ON": False,
            "LOCK ON": False,
            "TDD 탐지": False,
            "신관 작동": False,
        }
    )
    mission_loaded: bool = False
    mission_launched: bool = False
    target_designated: bool = False
    target_detected: bool = False
    engagement_approved: bool = False
    launch_requested: bool = False
    engagement_requested: bool = False
    target_destroyed: bool = False
    detection_source_vehicle_id: int | None = None
    emergency_mode: bool = False
    flight_phase: str = "STANDBY"
    current_waypoint_index: int = 0
    completed_route_segment_count: int = 0
    guidance_elapsed_s: float = 0.0
    simulation_elapsed_s: float = 0.0
    search_elapsed_s: float = 0.0
    last_launch_action: str = "NONE"
    shutdown_position: tuple[float, float, float] | None = None
    flight_path: list[dict[str, float]] = field(default_factory=list)
    _started_at: float = field(default_factory=time.monotonic, repr=False)
    _route_points: list[tuple[float, float, float, str]] = field(
        default_factory=list,
        repr=False,
    )
    _mission_signature: tuple = field(default_factory=tuple, repr=False)
    _track_predictors: dict[int, ConstantVelocityTrackPredictor] = field(
        default_factory=dict,
        repr=False,
    )
    _last_tick_s: float = field(default=0.0, repr=False)
    _phase_before_emergency: str = field(default="ROUTE", repr=False)
    _loiter_center: tuple[float, float, float, str] | None = field(
        default=None,
        repr=False,
    )
    _loiter_angle_rad: float = field(default=0.0, repr=False)
    _detection_waypoint: tuple[float, float, float, str] | None = field(
        default=None,
        repr=False,
    )
    _return_point: tuple[float, float, float, str] | None = field(
        default=None,
        repr=False,
    )
    _mitl_return_waypoint: tuple[float, float, float, str] | None = field(
        default=None,
        repr=False,
    )
    _phase_elapsed_s: float = field(default=0.0, repr=False)
    _intercept_solution: dict | None = field(default=None, repr=False)
    _intercept_elapsed_s: float = field(default=0.0, repr=False)
    _intercept_vehicle_start: tuple[float, float, float] | None = field(
        default=None,
        repr=False,
    )
    _intercept_target_start: tuple[float, float, float] | None = field(
        default=None,
        repr=False,
    )
    _runtime_route_revision: int = field(default=0, repr=False)
    _runtime_route_update_count: int = field(default=0, repr=False)
    _search_started: bool = field(default=False, repr=False)
    _pending_runtime_route_revision: int = field(default=0, repr=False)
    _pending_runtime_route: list[tuple[float, float, float, str]] | None = field(
        default=None,
        repr=False,
    )
    _manual_route_hold_until_s: float = field(default=0.0, repr=False)
    _manual_route_active: bool = field(default=False, repr=False)

    @classmethod
    def demo(cls, latitude: float, longitude: float) -> "FlyState":
        now = time.time()
        threats: list[ThreatTrack] = []
        for (
            track_id,
            target_type,
            country,
            platform_name,
            distance_m,
            radial_heading,
            speed_mps,
            movement_heading,
            age_s,
        ) in DEMO_THREAT_SPECS:
            track_latitude, track_longitude = destination_position(
                latitude,
                longitude,
                distance_m,
                radial_heading,
            )
            threats.append(
                ThreatTrack(
                    track_id=track_id,
                    target_type=target_type,
                    country=country,
                    platform_name=platform_name,
                    latitude=track_latitude,
                    longitude=track_longitude,
                    altitude_m=0.0,
                    speed_mps=speed_mps,
                    heading_deg=movement_heading,
                    first_tracked_at=now - age_s,
                )
            )

        state = cls(
            center_latitude=latitude,
            center_longitude=longitude,
            vehicle=VehicleTrack(latitude=latitude, longitude=longitude),
            threats=threats,
        )
        state._reset_predictors()
        return state

    @property
    def elapsed_s(self) -> float:
        return max(0.0, time.monotonic() - self._started_at)

    @property
    def selected_threat(self) -> ThreatTrack | None:
        return next(
            (
                track
                for track in self.threats
                if track.track_id == self.selected_track_id
            ),
            None,
        )

    @property
    def launch_ready(self) -> bool:
        return (
            self.mission_loaded
            and all(self.readiness.values())
            and not self.emergency_mode
        )

    @property
    def automatic_mode(self) -> str:
        return (
            "ARM"
            if self.engagement_approved
            and not self.emergency_mode
            and not self.target_destroyed
            else "SAFE"
        )

    @property
    def seeker_mode(self) -> str:
        if self.target_destroyed:
            return "SHUT DOWN"
        if self.emergency_mode or self.flight_phase in {
            "EMERGENCY_SAFE_RETURN",
            "MITL_WAYPOINT_RETURN",
            "MITL_SAFE_RETURN",
            "RETURNED",
        }:
            return "STOW"
        if not self.mission_launched:
            return "STANDBY"
        if not self.target_detected:
            return "SEARCH"
        return {
            "DETECTION_TRANSIT": "ATR HANDOFF",
            "INITIAL_GUIDANCE": "ATR ACQUIRE",
            "MIDCOURSE_GUIDANCE": "ATR TRACK",
            "TERMINAL_GUIDANCE": "ATR LOCK",
            "DESTROYED": "SHUT DOWN",
        }.get(self.flight_phase, "ATR TRACK")

    @property
    def intercept_ready(self) -> bool:
        return (
            self.mission_launched
            and self.target_detected
            and self.engagement_requested
            and not self.target_destroyed
        )

    @property
    def can_press_launch(self) -> bool:
        return self.launch_ready and not self.mission_launched

    @property
    def engagement_success(self) -> bool:
        return self.target_destroyed

    @property
    def runtime_route_revision(self) -> int:
        return self._runtime_route_revision

    @property
    def runtime_route_update_count(self) -> int:
        return self._runtime_route_update_count

    @property
    def search_started(self) -> bool:
        return self._search_started

    @property
    def pending_runtime_route_revision(self) -> int:
        return self._pending_runtime_route_revision

    @property
    def manual_route_active(self) -> bool:
        return self._manual_route_active

    @property
    def manual_route_hold_active(self) -> bool:
        return self._manual_route_active and self.manual_route_hold_remaining_s > 0.0

    @property
    def manual_route_hold_remaining_s(self) -> float:
        return max(
            0.0,
            self._manual_route_hold_until_s - self.simulation_elapsed_s,
        )

    def load_mission(
        self,
        store: SiteStore,
        vehicle_id: int | None = None,
    ) -> None:
        vehicle_id = (
            store.active_vehicle_id if vehicle_id is None else int(vehicle_id)
        )
        route_points = store.waypoints_for(vehicle_id)
        signature = (
            vehicle_id,
            tuple(
                (
                    code,
                    site.latitude,
                    site.longitude,
                    site.altitude_m,
                )
                for code, site in sorted(store.sites.items())
            ),
            tuple(
                (
                    point.code,
                    point.latitude,
                    point.longitude,
                    point.altitude_m,
                )
                for point in route_points
            ),
            tuple(
                (
                    target.track_id,
                    target.latitude,
                    target.longitude,
                    target.speed_mps,
                    target.heading_deg,
                    target.position_uncertainty_m,
                    target.source,
                    tuple(
                        tuple(sorted(segment.items()))
                        for segment in target.motion_profile
                    ),
                )
                for target in store.initial_targets
            ),
        )
        self.sync_plan_readiness(store, vehicle_id)
        if signature == self._mission_signature:
            return
        self._mission_signature = signature
        self.vehicle.code = f"LM-{vehicle_id:02d}"
        self._route_points = [
            (
                point.latitude,
                point.longitude,
                point.altitude_m,
                point.code,
            )
            for point in route_points
        ]
        safe_zone = next(
            (zone for zone in store.zones if zone.zone_type == "SAFE"),
            None,
        )
        safe_center = safe_zone.center() if safe_zone is not None else None
        self._return_point = (
            (
                safe_center.latitude,
                safe_center.longitude,
                60.0,
                "SAFE",
            )
            if safe_center is not None
            else None
        )
        self.mission_loaded = (
            store.shared_configuration_ready and bool(route_points)
        )
        self._reset_execution()

        launcher = store.sites.get("LC") or store.sites.get("GCS")
        if launcher is not None:
            self.center_latitude = launcher.latitude
            self.center_longitude = launcher.longitude
            self.vehicle.latitude = launcher.latitude
            self.vehicle.longitude = launcher.longitude
            self.vehicle.altitude_m = launcher.altitude_m
            if store.initial_targets:
                self._load_mission_targets(store)
            else:
                self._rebase_demo_threats(
                    launcher.latitude,
                    launcher.longitude,
                )

    def update_route_from_store(
        self,
        store: SiteStore,
        vehicle_id: int,
    ) -> None:
        """Replace only an in-flight route while preserving execution state."""
        vehicle_id = int(vehicle_id)
        route_points = store.waypoints_for(vehicle_id)
        previous_target = (
            self._route_points[self.current_waypoint_index]
            if self._route_points
            and self.current_waypoint_index < len(self._route_points)
            else None
        )
        self._route_points = [
            (
                point.latitude,
                point.longitude,
                point.altitude_m,
                point.code,
            )
            for point in route_points
        ]
        self._mission_signature = (
            vehicle_id,
            tuple(
                (
                    code,
                    site.latitude,
                    site.longitude,
                    site.altitude_m,
                )
                for code, site in sorted(store.sites.items())
            ),
            tuple(
                (
                    point.code,
                    point.latitude,
                    point.longitude,
                    point.altitude_m,
                )
                for point in route_points
            ),
        )
        self.sync_plan_readiness(store, vehicle_id)
        if not self._route_points:
            self.current_waypoint_index = 0
            self.completed_route_segment_count = 0
            return
        if previous_target is not None:
            self.current_waypoint_index = min(
                range(len(self._route_points)),
                key=lambda index: horizontal_distance_m(
                    previous_target[0],
                    previous_target[1],
                    self._route_points[index][0],
                    self._route_points[index][1],
                ),
            )
        else:
            self.current_waypoint_index = min(
                self.current_waypoint_index,
                len(self._route_points) - 1,
            )
        self.completed_route_segment_count = min(
            self.current_waypoint_index,
            len(self._route_points),
        )

    def set_external_intercept(self, solution: dict | None) -> None:
        """Accept the latest shared IMM-PF intercept solution.

        The legacy CV-KF estimator remains available as a fallback, but a
        fresh external solution becomes the displayed and commanded midcourse
        aim point for this vehicle.
        """
        if solution is None or not self.engagement_requested:
            return
        self._intercept_solution = dict(solution)

    def engagement_route_payload(self) -> list[dict[str, float | str]]:
        """Return the live ATR approach and intercept route for map display."""
        if not self.engagement_requested or self.target_destroyed:
            return []
        points: list[dict[str, float | str]] = []
        if (
            self.flight_phase == "DETECTION_TRANSIT"
            and self._detection_waypoint is not None
        ):
            points.append(
                {
                    "latitude": self._detection_waypoint[0],
                    "longitude": self._detection_waypoint[1],
                    "altitude_m": self._detection_waypoint[2],
                    "code": self._detection_waypoint[3],
                }
            )
        solution = self.predicted_intercept()
        if solution is not None:
            points.append(
                {
                    "latitude": float(solution["latitude"]),
                    "longitude": float(solution["longitude"]),
                    "altitude_m": float(solution.get("altitude_m", 0.0)),
                    "code": "ATR-IP",
                }
            )
        return points

    def route_points_payload(self) -> list[dict[str, float | str]]:
        if not self._route_points:
            return []
        start_index = min(
            max(0, self.current_waypoint_index),
            len(self._route_points) - 1,
        )
        return [
            {
                "latitude": point[0],
                "longitude": point[1],
                "altitude_m": point[2],
                "code": point[3],
            }
            for point in self._route_points[start_index:]
        ]

    def queue_runtime_route(
        self,
        revision: int,
        points: tuple[dict, ...] | list[dict],
    ) -> bool:
        """Accept an automatic probability-weighted route revision.

        Ingress is protected until the common rally point. During search, a
        confirmed planner revision immediately replaces the AUTO waypoint
        suffix so the operator can see the route being recomputed in real time.
        """
        if self.manual_route_hold_active:
            # An operator-applied route owns the command path for a bounded
            # hold window.  The background RHP/PF estimator keeps running, but
            # its candidate must not overwrite the manual edit immediately.
            return False
        revision = int(revision)
        if revision <= max(
            self._runtime_route_revision,
            self._pending_runtime_route_revision,
        ):
            return False
        parsed = [
            (
                float(point["latitude"]),
                float(point["longitude"]),
                max(0.0, float(point.get("altitude_m", 600.0))),
                str(point.get("code") or f"AUTO{index:02d}"),
            )
            for index, point in enumerate(points, start=1)
        ]
        if not parsed:
            return False
        self._pending_runtime_route = parsed
        self._pending_runtime_route_revision = revision
        if (
            self.flight_phase == "ROUTE"
            and (
                self._search_started
                or self.completed_route_segment_count >= 1
            )
        ):
            self._search_started = True
            self._commit_pending_runtime_route()
        return True

    def _commit_pending_runtime_route(self) -> bool:
        if not self._pending_runtime_route:
            return False
        self._route_points = self._pending_runtime_route
        self._runtime_route_revision = self._pending_runtime_route_revision
        self._runtime_route_update_count += 1
        self._pending_runtime_route = None
        self._pending_runtime_route_revision = 0
        self._manual_route_active = False
        self._manual_route_hold_until_s = 0.0
        self.current_waypoint_index = 0
        self.completed_route_segment_count = 0
        return True

    def apply_manual_runtime_route(
        self,
        points: tuple[dict, ...] | list[dict],
        *,
        hold_duration_s: float = 50.0,
    ) -> bool:
        """Apply an operator-edited route without resetting flight execution.

        Automatic RHP proposals continue to be evaluated during the hold, but
        ``queue_runtime_route`` rejects them until the bounded hold expires.
        This keeps a just-applied map edit visible and commandable while still
        returning control to the automatic planner afterwards.
        """
        parsed = [
            (
                float(point["latitude"]),
                float(point["longitude"]),
                max(0.0, float(point.get("altitude_m", 600.0))),
                str(point.get("code") or f"MWP{index:03d}"),
            )
            for index, point in enumerate(points, start=1)
        ]
        if not parsed:
            return False
        previous_target = (
            self._route_points[self.current_waypoint_index]
            if self._route_points
            and self.current_waypoint_index < len(self._route_points)
            else None
        )
        self._route_points = parsed
        self._pending_runtime_route = None
        self._pending_runtime_route_revision = 0
        self._runtime_route_revision = max(1, self._runtime_route_revision)
        self._runtime_route_update_count += 1
        self._manual_route_active = True
        self._manual_route_hold_until_s = (
            self.simulation_elapsed_s + max(1.0, float(hold_duration_s))
        )
        if previous_target is None:
            self.current_waypoint_index = min(
                self.current_waypoint_index,
                len(parsed) - 1,
            )
        else:
            self.current_waypoint_index = min(
                range(len(parsed)),
                key=lambda index: horizontal_distance_m(
                    previous_target[0],
                    previous_target[1],
                    parsed[index][0],
                    parsed[index][1],
                ),
            )
        self.completed_route_segment_count = min(
            self.current_waypoint_index,
            len(parsed),
        )
        return True

    def runtime_route_payload(self) -> list[dict[str, float | str]]:
        if self._runtime_route_revision <= 0:
            return []
        return [
            {
                "latitude": point[0],
                "longitude": point[1],
                "altitude_m": point[2],
                "code": point[3],
            }
            for point in self._route_points
        ]

    def pending_runtime_route_payload(self) -> list[dict[str, float | str]]:
        return [
            {
                "latitude": point[0],
                "longitude": point[1],
                "altitude_m": point[2],
                "code": point[3],
            }
            for point in (self._pending_runtime_route or [])
        ]

    def _rebase_demo_threats(
        self,
        latitude: float,
        longitude: float,
    ) -> None:
        now = time.time()
        tracks_by_id = {track.track_id: track for track in self.threats}
        for (
            track_id,
            target_type,
            country,
            platform_name,
            distance_m,
            radial_heading,
            speed_mps,
            movement_heading,
            age_s,
        ) in DEMO_THREAT_SPECS:
            track_latitude, track_longitude = destination_position(
                latitude,
                longitude,
                distance_m,
                radial_heading,
            )
            track = tracks_by_id.get(track_id)
            if track is None:
                track = ThreatTrack(
                    track_id=track_id,
                    target_type=target_type,
                    country=country,
                    platform_name=platform_name,
                    latitude=track_latitude,
                    longitude=track_longitude,
                    altitude_m=0.0,
                    speed_mps=speed_mps,
                    heading_deg=movement_heading,
                    first_tracked_at=now - age_s,
                )
                self.threats.append(track)
            else:
                track.target_type = target_type
                track.country = country
                track.platform_name = platform_name
                track.latitude = track_latitude
                track.longitude = track_longitude
                track.altitude_m = 0.0
                track.speed_mps = speed_mps
                track.heading_deg = movement_heading
                track.first_tracked_at = now - age_s
                track.destroyed = False
                track.position_uncertainty_m = 500.0
                track.source = "LOCAL"
                track.motion_profile = []
        self._reset_predictors()

    def _load_mission_targets(self, store: SiteStore) -> None:
        now = time.time()
        self.threats = [
            ThreatTrack(
                track_id=target.track_id,
                target_type=target.target_type,
                country=target.country,
                platform_name=target.platform_name,
                latitude=target.latitude,
                longitude=target.longitude,
                altitude_m=target.altitude_m,
                speed_mps=min(40.0 / 3.6, target.speed_mps),
                heading_deg=target.heading_deg,
                first_tracked_at=now - target.track_age_s,
                position_uncertainty_m=target.position_uncertainty_m,
                source=target.source,
                motion_profile=[
                    dict(segment) for segment in target.motion_profile
                ],
            )
            for target in store.initial_targets
        ]
        if self.threats and not any(
            track.track_id == self.selected_track_id
            for track in self.threats
        ):
            self.selected_track_id = self.threats[0].track_id
        self._reset_predictors()

    def _reset_execution(self) -> None:
        self.mission_launched = False
        self.target_designated = False
        self.target_detected = False
        self.engagement_approved = False
        self.launch_requested = False
        self.engagement_requested = False
        self.target_destroyed = False
        self.detection_source_vehicle_id = None
        self.emergency_mode = False
        self.flight_phase = "STANDBY"
        self.current_waypoint_index = 0
        self.completed_route_segment_count = 0
        self.guidance_elapsed_s = 0.0
        self.simulation_elapsed_s = 0.0
        self.search_elapsed_s = 0.0
        self.last_launch_action = "NONE"
        self.shutdown_position = None
        self.flight_path = []
        self._loiter_center = None
        self._detection_waypoint = None
        self._mitl_return_waypoint = None
        self._phase_elapsed_s = 0.0
        self._intercept_solution = None
        self._intercept_elapsed_s = 0.0
        self._intercept_vehicle_start = None
        self._intercept_target_start = None
        self._runtime_route_revision = 0
        self._runtime_route_update_count = 0
        self._search_started = False
        self._pending_runtime_route_revision = 0
        self._pending_runtime_route = None
        self._manual_route_hold_until_s = 0.0
        self._manual_route_active = False
        self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
        for name in self.mission_status:
            self.mission_status[name] = False
        for threat in self.threats:
            threat.destroyed = False

    def select_threat(self, track_id: int) -> bool:
        if not any(track.track_id == track_id for track in self.threats):
            return False
        self.selected_track_id = track_id
        return True

    def designate_threat(
        self,
        track_id: int,
        *,
        cooperative_approach: tuple[float, float, float, str] | None = None,
        detector_vehicle_id: int | None = None,
    ) -> bool:
        if not self.select_threat(track_id):
            return False
        if not self.mission_launched:
            return True
        self.target_designated = True
        self.target_detected = True
        self.engagement_approved = True
        self.guidance_elapsed_s = 0.0
        self._phase_elapsed_s = 0.0
        self.launch_requested = False
        self.engagement_requested = True
        self.target_destroyed = False
        self.detection_source_vehicle_id = detector_vehicle_id
        self.shutdown_position = None
        self.vehicle.speed_mps = INITIAL_GUIDANCE_SPEED_MPS
        self._intercept_solution = self._calculate_predicted_intercept()
        self._detection_waypoint = cooperative_approach
        self._pending_runtime_route = None
        self._pending_runtime_route_revision = 0
        self._manual_route_hold_until_s = 0.0
        self._manual_route_active = False
        self._loiter_center = None
        self._intercept_elapsed_s = 0.0
        self._intercept_vehicle_start = None
        self._intercept_target_start = None
        for name in self.mission_status:
            self.mission_status[name] = False
        if not self.emergency_mode:
            if cooperative_approach is not None:
                self.flight_phase = "DETECTION_TRANSIT"
            else:
                # ATR doctrine: the first detection aborts every search arc.
                # All six LM immediately turn toward the live ground target.
                self.flight_phase = "INITIAL_GUIDANCE"
                self.mission_status["초기유도"] = True
        return True

    def sync_plan_readiness(
        self,
        store: SiteStore,
        vehicle_id: int | None = None,
    ) -> None:
        if vehicle_id is None:
            try:
                vehicle_id = int(self.vehicle.code.rsplit("-", 1)[1])
            except (IndexError, ValueError):
                vehicle_id = store.active_vehicle_id
        ready = (
            store.shared_configuration_ready
            and bool(store.waypoints_for(vehicle_id))
        )
        self.readiness["AVS"] = True
        self.readiness["LC"] = "LC" in store.sites
        self.readiness["RDR"] = "RDR" in store.sites
        self.readiness["DL"] = True
        self.readiness["GCS"] = "GCS" in store.sites
        self.mission_loaded = ready

    def request_simulated_launch(self) -> bool:
        self.last_launch_action = "DENIED"
        if not self.launch_ready:
            return False
        if not self.mission_launched:
            self.mission_launched = True
            self.flight_phase = "ROUTE"
            self.vehicle.speed_mps = LM_INGRESS_SPEED_MPS
            self.current_waypoint_index = 0
            self.completed_route_segment_count = 0
            self.last_launch_action = "MISSION_LAUNCH"
            return True
        return False

    def stop_engagement(self) -> None:
        self.engagement_approved = False
        self.engagement_requested = False
        self.target_designated = False
        self.target_detected = False
        self.detection_source_vehicle_id = None
        if self.mission_launched and not self.emergency_mode:
            self.flight_phase = "ROUTE"
            self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
        self._mitl_return_waypoint = None
        self._detection_waypoint = None
        self._intercept_solution = None
        for name in self.mission_status:
            self.mission_status[name] = False

    def request_safe_return_via_waypoint(self) -> bool:
        self.engagement_approved = False
        self.engagement_requested = False
        self.target_designated = False
        self.target_detected = False
        self._intercept_solution = None
        self._loiter_center = None
        self._detection_waypoint = None
        for name in self.mission_status:
            self.mission_status[name] = False
        if not self.mission_launched or self.emergency_mode:
            return False

        self._mitl_return_waypoint = self._nearest_route_point_to_vehicle()
        self.flight_phase = (
            "MITL_WAYPOINT_RETURN"
            if self._mitl_return_waypoint is not None
            else "MITL_SAFE_RETURN"
        )
        self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
        return True

    def toggle_emergency(self) -> bool:
        self.emergency_mode = not self.emergency_mode
        if self.emergency_mode:
            self._phase_before_emergency = self.flight_phase
            self.flight_phase = "EMERGENCY_SAFE_RETURN"
            self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
        else:
            self.flight_phase = (
                self._phase_before_emergency
                if self.mission_launched
                else "STANDBY"
            )
            self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
            if self.flight_phase == "TERMINAL_GUIDANCE":
                self._begin_intercept_animation()
        return self.emergency_mode

    def _nearest_route_point_to_target(
        self,
        target: ThreatTrack | None,
    ) -> tuple[float, float, float, str] | None:
        if target is None or not self._route_points:
            return None
        return min(
            self._route_points,
            key=lambda candidate: horizontal_distance_m(
                target.latitude,
                target.longitude,
                candidate[0],
                candidate[1],
            ),
        )

    def _nearest_route_point_to_vehicle(
        self,
    ) -> tuple[float, float, float, str] | None:
        if not self._route_points:
            return None
        return min(
            self._route_points,
            key=lambda candidate: horizontal_distance_m(
                self.vehicle.latitude,
                self.vehicle.longitude,
                candidate[0],
                candidate[1],
            ),
        )

    def tick(self, dt_override_s: float | None = None) -> None:
        elapsed = self.elapsed_s
        dt = (
            max(0.01, min(float(dt_override_s), 1.0))
            if dt_override_s is not None
            else (
                max(0.05, min(elapsed - self._last_tick_s, 1.0))
                if self._last_tick_s
                else 0.25
            )
        )
        self._last_tick_s = elapsed
        search_was_active = self._search_started
        if self.mission_launched:
            self.simulation_elapsed_s += dt * DEMO_GROUND_TIME_SCALE
            if search_was_active:
                # The CPP/RHP experiment defines t=0 at physical TP arrival.
                # Ingress and target-scenario time must not consume the first
                # 25-second receding-horizon interval.
                self.search_elapsed_s += dt * DEMO_GROUND_TIME_SCALE
            self._advance_threats(dt)

        if self.emergency_mode:
            self._advance_emergency_return(dt)
            return
        if not self.mission_launched:
            return

        if self.flight_phase == "ROUTE":
            self._advance_route(dt)
        elif self.flight_phase == "DETECTION_TRANSIT":
            self._advance_detection_transit(dt)
        elif self.flight_phase == "INITIAL_GUIDANCE":
            self._advance_initial_guidance(dt)
        elif self.flight_phase == "MIDCOURSE_GUIDANCE":
            self._advance_midcourse_guidance(dt)
        elif self.flight_phase == "TERMINAL_GUIDANCE":
            self._advance_intercept(dt)
        elif self.flight_phase in {
            "MITL_WAYPOINT_RETURN",
            "MITL_SAFE_RETURN",
        }:
            self._advance_mitl_safe_return(dt)
        if self.mission_launched:
            if not self.flight_path or horizontal_distance_m(
                self.flight_path[-1]["latitude"],
                self.flight_path[-1]["longitude"],
                self.vehicle.latitude,
                self.vehicle.longitude,
            ) >= 120.0:
                self.flight_path.append(
                    {
                        "latitude": self.vehicle.latitude,
                        "longitude": self.vehicle.longitude,
                        "altitude_m": self.vehicle.altitude_m,
                    }
                )
                self.flight_path = self.flight_path[-1200:]

    def _advance_route(self, dt: float) -> None:
        if not self._route_points:
            return
        on_initial_ingress = (
            self._runtime_route_revision == 0
            and self.completed_route_segment_count == 0
            and self.current_waypoint_index == 0
        )
        self.vehicle.speed_mps = (
            LM_INGRESS_SPEED_MPS
            if on_initial_ingress
            else LM_SEARCH_SPEED_MPS
        )
        target = self._route_points[self.current_waypoint_index]
        if str(target[3]).startswith("RHP-T"):
            self.vehicle.speed_mps = LM_CRUISE_SPEED_MPS
        reached = self._move_vehicle_toward(
            target[0],
            target[1],
            target[2],
            self.vehicle.speed_mps * dt * DEMO_FLIGHT_TIME_SCALE,
        )
        if reached:
            if on_initial_ingress:
                self._search_started = True
            if self._commit_pending_runtime_route():
                return
            self.completed_route_segment_count = min(
                len(self._route_points),
                self.completed_route_segment_count + 1,
            )
            self.current_waypoint_index = (
                self.current_waypoint_index + 1
            ) % len(self._route_points)

    def _advance_detection_transit(self, dt: float) -> None:
        if self._detection_waypoint is None:
            self.flight_phase = "INITIAL_GUIDANCE"
            self.mission_status["초기유도"] = True
            self._phase_elapsed_s = 0.0
            return
        reached = self._move_vehicle_toward(
            self._detection_waypoint[0],
            self._detection_waypoint[1],
            max(80.0, self._detection_waypoint[2]),
            INITIAL_GUIDANCE_SPEED_MPS * dt * DEMO_FLIGHT_TIME_SCALE,
        )
        if reached:
            self.flight_phase = "INITIAL_GUIDANCE"
            self.vehicle.speed_mps = INITIAL_GUIDANCE_SPEED_MPS
            self._phase_elapsed_s = 0.0
            self.mission_status["초기유도"] = True

    def _advance_initial_guidance(self, dt: float) -> None:
        target = self.selected_threat
        if target is not None:
            self._move_vehicle_toward(
                target.latitude,
                target.longitude,
                600.0,
                INITIAL_GUIDANCE_SPEED_MPS * dt * MIDCOURSE_TIME_SCALE,
            )
        self._phase_elapsed_s += dt * MIDCOURSE_TIME_SCALE
        if self._phase_elapsed_s >= INITIAL_GUIDANCE_DURATION_S:
            self.flight_phase = "MIDCOURSE_GUIDANCE"
            self.vehicle.speed_mps = MIDCOURSE_GUIDANCE_SPEED_MPS
            self.mission_status["중기유도"] = True
            self._phase_elapsed_s = 0.0

    def _advance_midcourse_guidance(self, dt: float) -> None:
        self._phase_elapsed_s += dt * MIDCOURSE_TIME_SCALE
        solution = self._intercept_solution
        if solution is None or str(solution.get("model", "")).startswith(
            "CV-KF"
        ):
            solution = self._calculate_predicted_intercept()
            self._intercept_solution = solution
        if solution is None:
            return
        distance = horizontal_distance_m(
            self.vehicle.latitude,
            self.vehicle.longitude,
            float(solution["latitude"]),
            float(solution["longitude"]),
        )
        minimum_midcourse_complete = (
            self._phase_elapsed_s >= MINIMUM_MIDCOURSE_DURATION_S
        )
        if distance <= TERMINAL_ENTRY_RADIUS_M:
            if minimum_midcourse_complete:
                self._start_terminal_guidance()
                return
            target = self.selected_threat
            if target is not None:
                self._move_vehicle_toward(
                    target.latitude,
                    target.longitude,
                    600.0,
                    MIDCOURSE_GUIDANCE_SPEED_MPS
                    * dt
                    * MIDCOURSE_TIME_SCALE,
                )
            return
        step = min(
            MIDCOURSE_GUIDANCE_SPEED_MPS * dt * MIDCOURSE_TIME_SCALE,
            max(1.0, distance - TERMINAL_ENTRY_RADIUS_M),
        )
        self._move_vehicle_toward(
            float(solution["latitude"]),
            float(solution["longitude"]),
            600.0,
            step,
        )
        remaining = horizontal_distance_m(
            self.vehicle.latitude,
            self.vehicle.longitude,
            float(solution["latitude"]),
            float(solution["longitude"]),
        )
        if (
            minimum_midcourse_complete
            and remaining <= TERMINAL_ENTRY_RADIUS_M + 1.0
        ):
            self._start_terminal_guidance()

    def _start_terminal_guidance(self) -> None:
        self.flight_phase = "TERMINAL_GUIDANCE"
        self.vehicle.speed_mps = MIDCOURSE_GUIDANCE_SPEED_MPS
        self.mission_status["종말유도"] = True
        self._phase_elapsed_s = 0.0
        self._begin_intercept_animation()

    def _advance_intercept(self, dt: float) -> None:
        target = self.selected_threat
        if target is None or target.destroyed:
            return
        if (
            self._intercept_solution is None
            or str(self._intercept_solution.get("model", "")).startswith(
                "CV-KF"
            )
        ):
            self._intercept_solution = self._calculate_predicted_intercept()
        self._phase_elapsed_s += dt * MIDCOURSE_TIME_SCALE
        fraction = min(1.0, self._phase_elapsed_s / 5.0)
        ramp_fraction = min(
            1.0,
            self._phase_elapsed_s / TERMINAL_SPEED_RAMP_S,
        )
        smooth_ramp = ramp_fraction * ramp_fraction * (
            3.0 - 2.0 * ramp_fraction
        )
        self.vehicle.speed_mps = (
            MIDCOURSE_GUIDANCE_SPEED_MPS
            + (TERMINAL_GUIDANCE_SPEED_MPS - MIDCOURSE_GUIDANCE_SPEED_MPS)
            * smooth_ramp
        )
        self.mission_status["종말유도"] = True
        self.mission_status["셔터 ON"] = fraction >= 0.20
        self.mission_status["LOCK ON"] = fraction >= 0.40
        self.mission_status["TDD 탐지"] = fraction >= 0.65
        self.mission_status["신관 작동"] = fraction >= 0.85
        reached = self._move_vehicle_toward_3d(
            target.latitude,
            target.longitude,
            target.altitude_m,
            self.vehicle.speed_mps * dt * MIDCOURSE_TIME_SCALE,
        )
        horizontal_separation_m = horizontal_distance_m(
            self.vehicle.latitude,
            self.vehicle.longitude,
            target.latitude,
            target.longitude,
        )
        altitude_separation_m = abs(
            self.vehicle.altitude_m - target.altitude_m
        )
        slant_separation_m = math.hypot(
            horizontal_separation_m,
            altitude_separation_m,
        )
        # Hold a full-frame red terminal lock at contact before changing to
        # SHUT DOWN.  This lets the seeker image reach essentially 100% width
        # instead of jumping from a small box directly to the kill overlay.
        if reached or slant_separation_m <= 5.0:
            self._intercept_elapsed_s += dt * MIDCOURSE_TIME_SCALE
        else:
            self._intercept_elapsed_s = 0.0
        if self._intercept_elapsed_s >= TERMINAL_CONTACT_DWELL_S:
            destination = (
                target.latitude,
                target.longitude,
                target.altitude_m,
            )
            self._complete_intercept(target, destination)

    def _begin_intercept_animation(self) -> None:
        target = self.selected_threat
        if target is None:
            return
        if self._intercept_solution is None:
            self._intercept_solution = self._calculate_predicted_intercept()
        self._intercept_elapsed_s = 0.0
        self._intercept_vehicle_start = (
            self.vehicle.latitude,
            self.vehicle.longitude,
            self.vehicle.altitude_m,
        )
        self._intercept_target_start = (
            target.latitude,
            target.longitude,
            target.altitude_m,
        )

    @staticmethod
    def _interpolate(start: float, end: float, fraction: float) -> float:
        return start + (end - start) * max(0.0, min(1.0, fraction))

    def _complete_intercept(
        self,
        target: ThreatTrack,
        intercept_position: tuple[float, float, float],
    ) -> None:
        target.destroyed = True
        self.target_destroyed = True
        self.engagement_requested = False
        self.flight_phase = "DESTROYED"
        self.vehicle.speed_mps = TERMINAL_GUIDANCE_SPEED_MPS
        target.latitude, target.longitude, target.altitude_m = intercept_position
        self.vehicle.latitude = intercept_position[0]
        self.vehicle.longitude = intercept_position[1]
        self.vehicle.altitude_m = intercept_position[2]
        self.shutdown_position = intercept_position
        for name in self.mission_status:
            self.mission_status[name] = True

    def _advance_emergency_return(self, dt: float) -> None:
        if self._return_point is None:
            self.flight_phase = "RETURNED"
            self.vehicle.speed_mps = 0.0
            return
        reached = self._move_vehicle_toward(
            self._return_point[0],
            self._return_point[1],
            max(30.0, self._return_point[2]),
            LM_CRUISE_SPEED_MPS * dt * DEMO_FLIGHT_TIME_SCALE,
        )
        if reached:
            self.flight_phase = "RETURNED"
            self.vehicle.speed_mps = 0.0

    def _advance_mitl_safe_return(self, dt: float) -> None:
        step_m = LM_CRUISE_SPEED_MPS * dt * DEMO_FLIGHT_TIME_SCALE
        if (
            self.flight_phase == "MITL_WAYPOINT_RETURN"
            and self._mitl_return_waypoint is not None
        ):
            reached_waypoint = self._move_vehicle_toward(
                self._mitl_return_waypoint[0],
                self._mitl_return_waypoint[1],
                max(80.0, self._mitl_return_waypoint[2]),
                step_m,
            )
            if not reached_waypoint:
                return
            self.flight_phase = "MITL_SAFE_RETURN"
            return

        if self._return_point is None:
            self.flight_phase = "RETURNED"
            self.vehicle.speed_mps = 0.0
            return
        reached_safe_zone = self._move_vehicle_toward(
            self._return_point[0],
            self._return_point[1],
            max(30.0, self._return_point[2]),
            step_m,
        )
        if reached_safe_zone:
            self.flight_phase = "RETURNED"
            self.vehicle.speed_mps = 0.0

    def _move_vehicle_toward(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        step_m: float,
    ) -> bool:
        distance = horizontal_distance_m(
            self.vehicle.latitude,
            self.vehicle.longitude,
            latitude,
            longitude,
        )
        if distance <= max(1.0, step_m):
            self.vehicle.latitude = latitude
            self.vehicle.longitude = longitude
            self.vehicle.altitude_m = altitude_m
            return True
        heading = bearing_deg(
            self.vehicle.latitude,
            self.vehicle.longitude,
            latitude,
            longitude,
        )
        self.vehicle.heading_deg = heading
        self.vehicle.latitude, self.vehicle.longitude = destination_position(
            self.vehicle.latitude,
            self.vehicle.longitude,
            step_m,
            heading,
        )
        altitude_fraction = min(1.0, step_m / max(1.0, distance))
        self.vehicle.altitude_m += (
            altitude_m - self.vehicle.altitude_m
        ) * altitude_fraction
        return False

    def _move_vehicle_toward_3d(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        step_m: float,
    ) -> bool:
        """Move along the full slant vector for terminal seeker guidance."""
        horizontal_m = horizontal_distance_m(
            self.vehicle.latitude,
            self.vehicle.longitude,
            latitude,
            longitude,
        )
        vertical_m = float(altitude_m) - self.vehicle.altitude_m
        slant_m = math.hypot(horizontal_m, vertical_m)
        if slant_m <= max(1.0, step_m):
            self.vehicle.latitude = latitude
            self.vehicle.longitude = longitude
            self.vehicle.altitude_m = altitude_m
            return True
        fraction = min(1.0, max(0.0, step_m) / slant_m)
        horizontal_step_m = horizontal_m * fraction
        if horizontal_step_m > 1e-6:
            heading = bearing_deg(
                self.vehicle.latitude,
                self.vehicle.longitude,
                latitude,
                longitude,
            )
            self.vehicle.heading_deg = heading
            self.vehicle.latitude, self.vehicle.longitude = destination_position(
                self.vehicle.latitude,
                self.vehicle.longitude,
                horizontal_step_m,
                heading,
            )
        self.vehicle.altitude_m += vertical_m * fraction
        return False

    def _advance_threats(self, dt: float) -> None:
        for track in self.threats:
            if not track.destroyed:
                simulated_dt = dt * DEMO_GROUND_TIME_SCALE
                orbital_motion = False
                if track.motion_profile:
                    segment = next(
                        (
                            item
                            for item in track.motion_profile
                            if float(item.get("start_s", 0.0))
                            <= self.simulation_elapsed_s
                            < float(item.get("end_s", float("inf")))
                        ),
                        track.motion_profile[-1],
                    )
                    track.speed_mps = min(
                        40.0 / 3.6,
                        max(
                            0.0,
                            float(
                                segment.get(
                                    "speed_kph",
                                    track.speed_mps * 3.6,
                                )
                            )
                            / 3.6,
                        ),
                    )
                    if str(segment.get("mode", "")).upper() == "ORBIT":
                        center_latitude = float(
                            segment.get("center_latitude", track.latitude)
                        )
                        center_longitude = float(
                            segment.get("center_longitude", track.longitude)
                        )
                        radius_m = max(
                            50.0,
                            float(segment.get("radius_m", 2_400.0)),
                        )
                        clockwise = bool(segment.get("clockwise", True))
                        radial_bearing = bearing_deg(
                            center_latitude,
                            center_longitude,
                            track.latitude,
                            track.longitude,
                        )
                        angular_step_deg = math.degrees(
                            track.speed_mps * simulated_dt / radius_m
                        )
                        if not clockwise:
                            angular_step_deg *= -1.0
                        radial_bearing = (
                            radial_bearing + angular_step_deg
                        ) % 360.0
                        track.latitude, track.longitude = destination_position(
                            center_latitude,
                            center_longitude,
                            radius_m,
                            radial_bearing,
                        )
                        track.heading_deg = (
                            radial_bearing + (90.0 if clockwise else -90.0)
                        ) % 360.0
                        orbital_motion = True
                    elif "turn_rate_dps" in segment:
                        track.heading_deg = (
                            track.heading_deg
                            + float(segment["turn_rate_dps"])
                            * simulated_dt
                        ) % 360.0
                    elif "heading_deg" in segment:
                        track.heading_deg = float(
                            segment["heading_deg"]
                        ) % 360.0
                if not orbital_motion:
                    track.latitude, track.longitude = destination_position(
                        track.latitude,
                        track.longitude,
                        track.speed_mps * simulated_dt,
                        track.heading_deg,
                    )
                if not track.motion_profile:
                    track.heading_deg = (
                        track.heading_deg
                        + math.sin(
                            self.elapsed_s / 13.0 + track.track_id
                        )
                        * 0.12
                    ) % 360.0
            predictor = self._track_predictors.get(track.track_id)
            if predictor is None:
                predictor = self._new_predictor(track)
                self._track_predictors[track.track_id] = predictor
            predictor.update(
                track.latitude,
                track.longitude,
                track.altitude_m,
                dt,
            )

    def _new_predictor(
        self,
        track: ThreatTrack,
    ) -> ConstantVelocityTrackPredictor:
        predictor = ConstantVelocityTrackPredictor(
            self.center_latitude,
            self.center_longitude,
            track.latitude,
            track.longitude,
            track.altitude_m,
        )
        heading = math.radians(track.heading_deg)
        predictor.east.velocity = math.sin(heading) * track.speed_mps
        predictor.north.velocity = math.cos(heading) * track.speed_mps
        return predictor

    def _reset_predictors(self) -> None:
        self._track_predictors = {
            track.track_id: self._new_predictor(track)
            for track in self.threats
        }

    def _calculate_predicted_intercept(self) -> dict | None:
        threat = self.selected_threat
        if threat is None:
            return None
        predictor = self._track_predictors.get(threat.track_id)
        if predictor is None:
            return None
        estimate = predictor.estimate()
        mean_latitude = math.radians(
            (self.vehicle.latitude + estimate.latitude) / 2.0
        )
        relative_east_m = (
            (estimate.longitude - self.vehicle.longitude)
            * EARTH_METERS_PER_DEGREE
            * math.cos(mean_latitude)
        )
        relative_north_m = (
            estimate.latitude - self.vehicle.latitude
        ) * EARTH_METERS_PER_DEGREE
        relative_up_m = estimate.altitude_m - self.vehicle.altitude_m
        pursuer_speed_mps = max(1.0, self.vehicle.speed_mps)
        intercept_time_s, reachable = constant_velocity_intercept_time(
            relative_east_m,
            relative_north_m,
            relative_up_m,
            estimate.velocity_east_mps,
            estimate.velocity_north_mps,
            estimate.velocity_up_mps,
            pursuer_speed_mps,
        )
        prediction = predictor.predict(intercept_time_s)
        return {
            "code": "INT",
            "label": "예상 요격지점",
            "model": "CV-KF RELATIVE INTERCEPT",
            "horizon_s": intercept_time_s,
            "max_horizon_s": PREDICTION_HORIZON_S,
            "reachable": reachable,
            "vehicle_code": self.vehicle.code,
            "vehicle_speed_mps": pursuer_speed_mps,
            "vehicle_latitude": self.vehicle.latitude,
            "vehicle_longitude": self.vehicle.longitude,
            "vehicle_altitude_m": self.vehicle.altitude_m,
            "latitude": prediction.latitude,
            "longitude": prediction.longitude,
            "altitude_m": prediction.altitude_m,
            "estimated_speed_mps": prediction.estimated_speed_mps,
            "estimated_heading_deg": prediction.estimated_heading_deg,
            "uncertainty_east_95_m": prediction.uncertainty_east_95_m,
            "uncertainty_north_95_m": prediction.uncertainty_north_95_m,
            "uncertainty_altitude_95_m": prediction.uncertainty_altitude_95_m,
        }

    def predicted_intercept(self) -> dict | None:
        if not self.engagement_requested:
            return None
        if self._intercept_solution is None:
            self._intercept_solution = self._calculate_predicted_intercept()
        return (
            dict(self._intercept_solution)
            if self._intercept_solution is not None
            else None
        )

    def render_dict(
        self,
        store: SiteStore,
        *,
        include_plan: bool = True,
        include_flight_path: bool = True,
    ) -> dict:
        threat_payloads = []
        for track in self.threats:
            distance_m = horizontal_distance_m(
                self.vehicle.latitude,
                self.vehicle.longitude,
                track.latitude,
                track.longitude,
            )
            threat_payloads.append(
                {
                    **asdict(track),
                    "code": track.code,
                    "first_tracked_text": track.first_tracked_text,
                    "selected": track.track_id == self.selected_track_id,
                    "distance_m": distance_m,
                    "eta_s": distance_m / LM_CRUISE_SPEED_MPS,
                }
            )
        shutdown_marker = None
        if self.shutdown_position is not None:
            shutdown_marker = {
                "latitude": self.shutdown_position[0],
                "longitude": self.shutdown_position[1],
                "altitude_m": self.shutdown_position[2],
                "label": "SHUT DOWN",
            }
        guidance_orbit = None
        emergency_return = None
        if self.emergency_mode and self._return_point is not None:
            emergency_return = {
                "latitude": self._return_point[0],
                "longitude": self._return_point[1],
                "altitude_m": self._return_point[2],
                "code": self._return_point[3],
                "phase": self.flight_phase,
            }
        mitl_return = None
        if self.flight_phase in {
            "MITL_WAYPOINT_RETURN",
            "MITL_SAFE_RETURN",
        } and self._return_point is not None:
            mitl_return = {
                "phase": self.flight_phase,
                "via_waypoint": (
                    {
                        "latitude": self._mitl_return_waypoint[0],
                        "longitude": self._mitl_return_waypoint[1],
                        "altitude_m": self._mitl_return_waypoint[2],
                        "code": self._mitl_return_waypoint[3],
                    }
                    if self.flight_phase == "MITL_WAYPOINT_RETURN"
                    and self._mitl_return_waypoint is not None
                    else None
                ),
                "safe_zone": {
                    "latitude": self._return_point[0],
                    "longitude": self._return_point[1],
                    "altitude_m": self._return_point[2],
                    "code": self._return_point[3],
                },
            }
        payload = {
            "vehicle": asdict(self.vehicle),
            "threats": threat_payloads,
            "selected_track_id": self.selected_track_id,
            "target_designated": self.target_designated,
            "target_detected": self.target_detected,
            "detection_source_vehicle_id": self.detection_source_vehicle_id,
            "engagement_approved": self.engagement_approved,
            "predicted_intercept": self.predicted_intercept(),
            "readiness": dict(self.readiness),
            "mission_status": dict(self.mission_status),
            "mission_loaded": self.mission_loaded,
            "mission_launched": self.mission_launched,
            "flight_phase": self.flight_phase,
            "search_elapsed_s": self.search_elapsed_s,
            "current_waypoint_index": self.current_waypoint_index,
            "completed_route_segment_count": (
                self.completed_route_segment_count
            ),
            "automatic_mode": self.automatic_mode,
            "seeker_mode": self.seeker_mode,
            "launch_ready": self.launch_ready,
            "can_press_launch": self.can_press_launch,
            "launch_requested": self.launch_requested,
            "intercept_ready": self.intercept_ready,
            "engagement_success": self.engagement_success,
            "emergency_mode": self.emergency_mode,
            "guidance_orbit": guidance_orbit,
            "emergency_return": emergency_return,
            "mitl_return": mitl_return,
            "shutdown_marker": shutdown_marker,
            "lm_cruise_speed_mps": LM_CRUISE_SPEED_MPS,
            "prediction_horizon_s": PREDICTION_HORIZON_S,
            "demo_time_scale": DEMO_FLIGHT_TIME_SCALE,
            "display_source": "LOCAL_SIMULATION",
        }
        if include_plan:
            payload["plan"] = store.render_dict()
        if include_flight_path:
            payload["flight_path"] = list(self.flight_path)
        return payload

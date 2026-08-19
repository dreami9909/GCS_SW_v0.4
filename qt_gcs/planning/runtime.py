from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from .geometry import (
    LocalFrame,
    LocalPoint,
    point_along_polyline,
    polyline_length,
)
from .imm_filter import ForecastPath, IMMParticleFilter
from .rhp_fe_pf_pw_arc import (
    MODEL_NAME,
    PF_CONFIGURATION,
    IsotropicTargetParticleFilter,
    RHPFEPFPWARCPlanner,
)
from .sensor_model import (
    SeekerSpec,
    SensorFootprint,
    build_footprint,
    build_local_footprint,
)


@dataclass(frozen=True, slots=True)
class RouteAssessment:
    vehicle_id: int
    target_id: int
    points: tuple[LocalPoint, ...]
    detection_probability: float
    expected_detection_time_s: float
    score: float
    signature: str


@dataclass(slots=True)
class _RouteGate:
    signature: str = ""
    streak: int = 0
    last_applied_s: float = -1e9


@dataclass(frozen=True, slots=True)
class PlanningCycleResult:
    revision: int
    calculation_ms: float
    planner: dict[str, Any]
    sensor: dict[str, Any]
    footprints: tuple[dict[str, Any], ...]
    beliefs: tuple[dict[str, Any], ...]
    intercepts: dict[int, dict[str, Any]]
    route_candidates: dict[int, dict[str, Any]]
    route_updates: dict[int, tuple[dict[str, Any], ...]]
    detections: tuple[dict[str, Any], ...]

    def render_dict(self) -> dict[str, Any]:
        return {
            "revision": self.revision,
            "calculation_ms": self.calculation_ms,
            "model": MODEL_NAME,
            "planner": self.planner,
            "sensor": self.sensor,
            "footprints": list(self.footprints),
            "beliefs": list(self.beliefs),
            "intercepts": list(self.intercepts.values()),
            "route_candidates": list(self.route_candidates.values()),
            "route_update_vehicle_ids": sorted(self.route_updates),
            "detections": list(self.detections),
        }


class RuleBasedPlanningEngine:
    """Stateful RHP-FE-PF-PW-ARC search and post-detection intercept engine.

    Search uses the validated isotropic PF baseline and evaluates a common set
    of future target samples every display cycle.  Only the best 25-second
    action prefix is committed at an RHP epoch.  The existing IMM estimator is
    retained solely for the post-detection moving-target intercept solution.
    """

    PREDICTION_HORIZON_S = 8.0 * 60.0
    FORECAST_STEP_S = 5.0
    SEARCH_RADIUS_M = 40_000.0 / 3_600.0 * 5.0 * 60.0
    SEARCH_SPEED_MPS = 100_000.0 / 3_600.0
    TRANSIT_SPEED_MPS = 160_000.0 / 3_600.0
    GUIDANCE_SPEED_MPS = 160_000.0 / 3_600.0

    def __init__(
        self,
        reference_latitude: float,
        reference_longitude: float,
        *,
        seeker: SeekerSpec | None = None,
        particle_count: int = 3_000,
        route_improvement_ratio: float = 0.03,
        probability_gain: float = 0.01,
        confirmation_cycles: int = 2,
        minimum_route_hold_s: float = 9.0,
        rhp_decision_interval_s: float = 25.0,
        rhp_encounter_sample_count: int = 64,
        rhp_radial_shortlist_count: int = 5,
        search_center_latitude: float | None = None,
        search_center_longitude: float | None = None,
        search_radius_m: float | None = None,
        minimum_route_updates_before_detection: int = 0,
    ) -> None:
        self.frame = LocalFrame(reference_latitude, reference_longitude)
        self.search_center = self.frame.to_local(
            (
                reference_latitude
                if search_center_latitude is None
                else search_center_latitude
            ),
            (
                reference_longitude
                if search_center_longitude is None
                else search_center_longitude
            ),
        )
        self.seeker = seeker or SeekerSpec()
        self.particle_count = particle_count
        self.route_improvement_ratio = route_improvement_ratio
        self.probability_gain = probability_gain
        self.confirmation_cycles = confirmation_cycles
        self.minimum_route_hold_s = minimum_route_hold_s
        self.filters: dict[int, IMMParticleFilter] = {}
        self.search_filters: dict[int, IsotropicTargetParticleFilter] = {}
        self._last_surveillance_update_s: dict[int, float] = {}
        self._route_gates = {
            vehicle_id: _RouteGate() for vehicle_id in range(1, 7)
        }
        self.revision = 0
        self._last_elapsed_s: float | None = None
        self._last_engagement_track_id: int | None = None
        self.search_radius_m = float(
            self.SEARCH_RADIUS_M if search_radius_m is None else search_radius_m
        )
        self.minimum_route_updates_before_detection = max(
            0,
            int(minimum_route_updates_before_detection),
        )
        self.rhp_planner = RHPFEPFPWARCPlanner(
            self.frame,
            self.search_center,
            search_radius_m=self.search_radius_m,
            track_spacing_m=self.seeker.track_spacing_m,
            detection_radius_m=self.seeker.ideal_detection_radius_m,
            transit_speed_mps=self.TRANSIT_SPEED_MPS,
            search_speed_mps=self.SEARCH_SPEED_MPS,
            decision_interval_s=rhp_decision_interval_s,
            encounter_sample_count=rhp_encounter_sample_count,
            radial_shortlist_count=rhp_radial_shortlist_count,
            detection_probability=self.seeker.detection_probability,
            swarm_coordination=False,
        )
        self._last_search_positions: dict[int, LocalPoint] = {}

    def reset(self) -> None:
        self.filters.clear()
        self.search_filters.clear()
        self._last_surveillance_update_s.clear()
        self._route_gates = {
            vehicle_id: _RouteGate() for vehicle_id in range(1, 7)
        }
        self.revision = 0
        self._last_elapsed_s = None
        self._last_engagement_track_id = None
        self.rhp_planner.reset()
        self._last_search_positions.clear()

    def update(
        self,
        *,
        elapsed_s: float,
        vehicles: list[dict[str, Any]],
        targets: list[dict[str, Any]],
        routes: dict[int, list[dict[str, Any]]],
        selected_track_id: int | None,
        engagement_track_id: int | None,
    ) -> PlanningCycleResult:
        started = time.perf_counter()
        previous_elapsed_s = self._last_elapsed_s
        dt = (
            0.0
            if previous_elapsed_s is None
            else max(0.0, min(float(elapsed_s) - previous_elapsed_s, 10.0))
        )
        self._last_elapsed_s = float(elapsed_s)
        self.revision += 1

        active_targets = [
            target for target in targets if not bool(target.get("destroyed"))
        ]
        active_track_ids = {
            int(target["track_id"]) for target in active_targets
        }
        for removed_track_id in set(self.filters) - active_track_ids:
            self.filters.pop(removed_track_id, None)
            self.search_filters.pop(removed_track_id, None)
            self._last_surveillance_update_s.pop(removed_track_id, None)
        engagement_changed = (
            engagement_track_id != self._last_engagement_track_id
        )
        for target in active_targets:
            track_id = int(target["track_id"])
            measurement = self.frame.to_local(
                float(target.get("measurement_latitude", target["latitude"])),
                float(target.get("measurement_longitude", target["longitude"])),
            )
            particle_filter = self.filters.get(track_id)
            search_filter = self.search_filters.get(track_id)
            if particle_filter is None:
                particle_filter = IMMParticleFilter(
                    measurement,
                    initial_speed_mps=float(
                        target.get(
                            "estimator_speed_mps",
                            target.get("speed_mps", 0.0),
                        )
                    ),
                    initial_heading_deg=float(target.get("heading_deg", 0.0)),
                    particle_count=self.particle_count,
                    initial_position_std_m=max(
                        1.0,
                        float(
                            target.get("position_uncertainty_m", 450.0)
                        ),
                    ),
                    seed=20_260_809 + track_id * 97,
                )
                self.filters[track_id] = particle_filter
                self._last_surveillance_update_s[track_id] = float(elapsed_s)
            else:
                particle_filter.predict(dt)
            if search_filter is None:
                search_filter = IsotropicTargetParticleFilter(
                    self.search_center,
                    self.search_radius_m,
                    maximum_speed_mps=40_000.0 / 3_600.0,
                    particle_count=self.particle_count,
                    seed=20_260_811 + track_id * 97,
                )
                self.search_filters[track_id] = search_filter
            else:
                search_filter.predict(dt)

            # The displayed threat track represents a low-accuracy surveillance
            # report.  Manual designation upgrades it to a seeker-quality track.
            last_update = self._last_surveillance_update_s.get(track_id, -1e9)
            designated = engagement_track_id == track_id
            static_surveillance = bool(
                target.get("static_surveillance_measurement", False)
            )
            update_interval_s = (
                1.0
                if designated
                else float("inf") if static_surveillance else 5.0
            )
            if (
                float(elapsed_s) - last_update >= update_interval_s
                or (designated and engagement_changed)
            ):
                particle_filter.observe_position(
                    measurement,
                    measurement_std_m=35.0 if designated else 180.0,
                )
                if designated:
                    search_filter.observe_position(
                        measurement,
                        measurement_std_m=35.0,
                    )
                self._last_surveillance_update_s[track_id] = float(elapsed_s)

        footprints = self._build_footprints(vehicles, float(elapsed_s))
        detection_footprints = self._build_detection_footprints(
            vehicles,
            previous_elapsed_s=previous_elapsed_s,
            elapsed_s=float(elapsed_s),
        )
        detections = self._detect_contacts(
            active_targets,
            detection_footprints,
            vehicles,
            engagement_track_id,
        )
        detected_track_ids = {
            int(detection["track_id"]) for detection in detections
        }
        sensing_segments = self._build_rhp_sensing_segments(vehicles)
        observation_enabled = bool(
            float(elapsed_s) > 1e-9
            and any(
                bool(vehicle.get("search_started", False))
                and not bool(vehicle.get("rhp_preview_only", False))
                for vehicle in vehicles
            )
        )
        if footprints and observation_enabled:
            for track_id, particle_filter in self.filters.items():
                if (
                    track_id != engagement_track_id
                    and track_id not in detected_track_ids
                ):
                    particle_filter.observe_no_detection(footprints)
        if sensing_segments and observation_enabled:
            for track_id, search_filter in self.search_filters.items():
                if (
                    track_id != engagement_track_id
                    and track_id not in detected_track_ids
                ):
                    search_filter.observe_no_detection_segments(
                        sensing_segments,
                        detection_radius_m=self.seeker.ideal_detection_radius_m,
                        detection_probability=self.seeker.detection_probability,
                    )

        forecasts: dict[int, tuple[ForecastPath, ...]] = {}
        beliefs = []
        for target in active_targets:
            track_id = int(target["track_id"])
            particle_filter = self.filters[track_id]
            search_filter = self.search_filters[track_id]
            summary = search_filter.summary()
            latitude, longitude = self.frame.to_geographic(
                LocalPoint(summary.mean_east_m, summary.mean_north_m)
            )
            beliefs.append(
                {
                    "track_id": track_id,
                    "latitude": latitude,
                    "longitude": longitude,
                    "estimated_speed_mps": summary.mean_speed_mps,
                    "estimated_heading_deg": summary.mean_heading_deg,
                    "uncertainty_east_95_m": summary.uncertainty_east_95_m,
                    "uncertainty_north_95_m": summary.uncertainty_north_95_m,
                    "effective_particle_count": (
                        summary.effective_particle_count
                    ),
                    "outside_probability": summary.outside_probability,
                    "mode_probabilities": {"ISOTROPIC": 1.0},
                    "pf_configuration": PF_CONFIGURATION,
                    "revision": summary.revision,
                }
            )
            if engagement_track_id == track_id:
                # The expensive IMM intercept forecast is irrelevant during
                # search.  RHP uses the isotropic search_filter directly;
                # deferring IMM forecast work keeps the 1 s PF update cadence
                # responsive without changing any route score.
                forecasts[track_id] = particle_filter.forecast(
                    self.PREDICTION_HORIZON_S,
                    step_s=self.FORECAST_STEP_S,
                    maximum_paths=72,
                    seed=20_260_809 + track_id * 1_009 + self.revision,
                )

        intercepts: dict[int, dict[str, Any]] = {}
        if engagement_track_id in forecasts:
            target_forecast = forecasts[int(engagement_track_id)]
            for vehicle in vehicles:
                if not bool(vehicle.get("mission_launched")):
                    continue
                vehicle_id = int(vehicle["vehicle_id"])
                solution = self._estimate_intercept(
                    vehicle,
                    int(engagement_track_id),
                    target_forecast,
                )
                if solution is not None:
                    intercepts[vehicle_id] = solution

        route_candidates: dict[int, dict[str, Any]] = {}
        route_updates: dict[int, tuple[dict[str, Any], ...]] = {}
        route_target_id = (
            int(selected_track_id)
            if selected_track_id in self.search_filters
            else next(iter(self.search_filters), None)
        )
        if engagement_track_id is None and route_target_id is not None:
            decision = self.rhp_planner.evaluate(
                elapsed_s=float(elapsed_s),
                revision=self.revision,
                track_id=route_target_id,
                belief=self.search_filters[route_target_id],
                vehicles=vehicles,
            )
            route_candidates = decision.candidates
            route_updates = decision.route_updates

        self._last_engagement_track_id = engagement_track_id
        calculation_ms = (time.perf_counter() - started) * 1_000.0
        search_center_latitude, search_center_longitude = self.frame.to_geographic(
            self.search_center
        )
        return PlanningCycleResult(
            revision=self.revision,
            calculation_ms=calculation_ms,
            planner={
                "model": MODEL_NAME,
                "belief_representation": "particle-filter",
                "motion_model": "isotropic",
                "particle_count": self.particle_count,
                "encounter_sample_count": (
                    self.rhp_planner.encounter_sample_count
                ),
                "radial_shortlist_count": (
                    self.rhp_planner.radial_shortlist_count
                ),
                "belief_update_interval_s": 1.0,
                "candidate_evaluation_interval_s": (
                    self.rhp_planner.decision_interval_s
                ),
                "decision_interval_s": (
                    self.rhp_planner.decision_interval_s
                ),
                "search_radius_m": self.search_radius_m,
                "target_lead_time_s": (
                    self.search_radius_m / (40_000.0 / 3_600.0)
                ),
                "track_spacing_m": self.seeker.track_spacing_m,
                "search_center": {
                    "latitude": search_center_latitude,
                    "longitude": search_center_longitude,
                    "altitude_m": 5.0,
                },
                "swarm_coordination": self.rhp_planner.swarm_coordination,
                "planning_scope": "GLOBAL_TP_THEN_LOCAL_RHP",
                "sector_ownership": "FIXED_60_DEG_PER_LM",
                "route_deconfliction": "FIXED_SECTOR_OWNERSHIP",
                "minimum_route_updates_before_detection": (
                    self.minimum_route_updates_before_detection
                ),
                "pf_configuration": PF_CONFIGURATION,
            },
            sensor=self.seeker.display_dict(),
            footprints=tuple(
                footprint.render_dict(self.frame) for footprint in footprints
            ),
            beliefs=tuple(beliefs),
            intercepts=intercepts,
            route_candidates=route_candidates,
            route_updates=route_updates,
            detections=detections,
        )

    def _build_rhp_sensing_segments(
        self,
        vehicles: list[dict[str, Any]],
    ) -> tuple[tuple[LocalPoint, LocalPoint], ...]:
        """Build the actual flown search segments for optimized PF evidence."""
        segments = []
        for vehicle in vehicles:
            if bool(vehicle.get("rhp_preview_only", False)):
                # Initial-prefix caching is computation only.  It must not
                # create a synthetic sensor-on segment from ingress to TP.
                continue
            vehicle_id = int(vehicle["vehicle_id"])
            current = self.frame.to_local(
                float(vehicle["latitude"]),
                float(vehicle["longitude"]),
            )
            previous = self._last_search_positions.get(vehicle_id, current)
            self._last_search_positions[vehicle_id] = current
            if (
                bool(vehicle.get("mission_launched"))
                and not bool(vehicle.get("emergency_mode"))
                and bool(vehicle.get("search_started", False))
                and str(vehicle.get("flight_phase")) == "ROUTE"
            ):
                segments.append((previous, current))
        return tuple(segments)

    def _detect_contacts(
        self,
        targets: list[dict[str, Any]],
        footprints: list[SensorFootprint],
        vehicles: list[dict[str, Any]],
        engagement_track_id: int | None,
    ) -> tuple[dict[str, Any], ...]:
        if engagement_track_id is not None:
            return ()
        if max(
            (
                int(vehicle.get("runtime_route_update_count", 0))
                for vehicle in vehicles
                if bool(vehicle.get("search_started", False))
            ),
            default=0,
        ) < self.minimum_route_updates_before_detection:
            return ()
        detections = []
        vehicle_positions = {
            int(vehicle["vehicle_id"]): self.frame.to_local(
                float(vehicle["latitude"]),
                float(vehicle["longitude"]),
            )
            for vehicle in vehicles
            if bool(vehicle.get("search_started", False))
        }
        for target in targets:
            target_position = self.frame.to_local(
                float(target["latitude"]),
                float(target["longitude"]),
            )
            best: tuple[float, SensorFootprint] | None = None
            for footprint in footprints:
                vehicle_position = vehicle_positions.get(footprint.vehicle_id)
                if vehicle_position is None:
                    continue
                # 1,200 m is the complete -45/+45 centre-line sweep width,
                # not a radius around the LM.  Clip the swept looks to the
                # corresponding 600 m reach and require the target to be in an
                # actual instantaneous FOV.  PD=1 remains deterministic once
                # those geometric conditions are met.
                if (
                    vehicle_position.distance_to(target_position)
                    > self.seeker.gimbal_centerline_reach_m + 1e-3
                ):
                    continue
                if footprint.detection_probability_at(target_position) <= 0.0:
                    continue
                probability = self.seeker.detection_probability
                if best is None or probability > best[0]:
                    best = probability, footprint
            if best is None:
                continue
            probability, footprint = best
            detections.append(
                {
                    "track_id": int(target["track_id"]),
                    "vehicle_id": footprint.vehicle_id,
                    "probability": probability,
                    "source": "SEEKER_SWEPT_FOV",
                }
            )
        return tuple(
            sorted(
                detections,
                key=lambda item: (-item["probability"], item["track_id"]),
            )
        )

    def _build_footprints(
        self,
        vehicles: list[dict[str, Any]],
        elapsed_s: float,
    ) -> list[SensorFootprint]:
        footprints = []
        for vehicle in vehicles:
            if (
                not bool(vehicle.get("mission_launched"))
                or bool(vehicle.get("emergency_mode"))
                or not bool(vehicle.get("search_started", False))
                or bool(vehicle.get("rhp_preview_only", False))
            ):
                continue
            footprints.append(
                build_footprint(
                    self.seeker,
                    self.frame,
                    vehicle_id=int(vehicle["vehicle_id"]),
                    latitude=float(vehicle["latitude"]),
                    longitude=float(vehicle["longitude"]),
                    altitude_m=max(
                        self.seeker.altitude_m,
                        float(vehicle.get("altitude_m", 0.0)),
                    ),
                    heading_deg=float(vehicle.get("heading_deg", 0.0)),
                    elapsed_s=elapsed_s,
                )
            )
        return footprints

    def _build_detection_footprints(
        self,
        vehicles: list[dict[str, Any]],
        *,
        previous_elapsed_s: float | None,
        elapsed_s: float,
    ) -> list[SensorFootprint]:
        """Sample every FOV swept since the previous 1 Hz planning update.

        Rendering still uses one footprint at the current scan angle.  Contact
        detection samples the intervening scan at an angular interval no wider
        than the configured overlapped FOV, avoiding both temporal aliasing and
        the former 1,200 m radial shortcut.
        """
        current_elapsed_s = float(elapsed_s)
        if previous_elapsed_s is None or current_elapsed_s <= previous_elapsed_s:
            sample_times = (current_elapsed_s,)
        else:
            # One complete triangular scan contains every possible look.  A
            # longer delayed update need not replay duplicate scan periods.
            window_start_s = max(
                float(previous_elapsed_s),
                current_elapsed_s - self.seeker.gimbal_scan_period_s,
            )
            duration_s = current_elapsed_s - window_start_s
            sample_count = max(
                1,
                int(math.ceil(duration_s / self.seeker.scan_sample_interval_s)),
            )
            sample_times = tuple(
                window_start_s + duration_s * index / sample_count
                for index in range(sample_count + 1)
            )

        footprints: list[SensorFootprint] = []
        full_interval_s = (
            0.0
            if previous_elapsed_s is None
            else max(0.0, current_elapsed_s - float(previous_elapsed_s))
        )
        for vehicle in vehicles:
            if (
                not bool(vehicle.get("mission_launched"))
                or bool(vehicle.get("emergency_mode"))
                or not bool(vehicle.get("search_started", False))
                or bool(vehicle.get("rhp_preview_only", False))
            ):
                continue
            vehicle_id = int(vehicle["vehicle_id"])
            current_point = self.frame.to_local(
                float(vehicle["latitude"]),
                float(vehicle["longitude"]),
            )
            previous_point = self._last_search_positions.get(
                vehicle_id,
                current_point,
            )
            for sample_time_s in sample_times:
                fraction = (
                    1.0
                    if full_interval_s <= 1e-9
                    else max(
                        0.0,
                        min(
                            1.0,
                            (sample_time_s - float(previous_elapsed_s))
                            / full_interval_s,
                        ),
                    )
                )
                sample_point = LocalPoint(
                    previous_point.east_m
                    + (current_point.east_m - previous_point.east_m) * fraction,
                    previous_point.north_m
                    + (current_point.north_m - previous_point.north_m) * fraction,
                )
                footprints.append(
                    build_local_footprint(
                        self.seeker,
                        vehicle_id=vehicle_id,
                        vehicle_point=sample_point,
                        altitude_m=max(
                            self.seeker.altitude_m,
                            float(vehicle.get("altitude_m", 0.0)),
                        ),
                        heading_deg=float(vehicle.get("heading_deg", 0.0)),
                        elapsed_s=sample_time_s,
                    )
                )
        return footprints

    def _estimate_intercept(
        self,
        vehicle: dict[str, Any],
        track_id: int,
        forecast: tuple[ForecastPath, ...],
    ) -> dict[str, Any] | None:
        vehicle_position = self.frame.to_local(
            float(vehicle["latitude"]),
            float(vehicle["longitude"]),
        )
        pursuer_speed = max(
            self.GUIDANCE_SPEED_MPS,
            float(vehicle.get("speed_mps", 0.0)),
        )
        feasible: list[tuple[float, float, LocalPoint]] = []
        for path in forecast:
            for time_s, target_position in zip(path.times_s[1:], path.points[1:]):
                if vehicle_position.distance_to(target_position) <= (
                    pursuer_speed * time_s
                ):
                    feasible.append((path.weight, time_s, target_position))
                    break
        reachable_probability = sum(weight for weight, _time, _point in feasible)
        if reachable_probability <= 1e-12:
            return None
        mean_time = sum(weight * time_s for weight, time_s, _ in feasible) / (
            reachable_probability
        )
        mean_east = sum(weight * point.east_m for weight, _, point in feasible) / (
            reachable_probability
        )
        mean_north = sum(
            weight * point.north_m for weight, _, point in feasible
        ) / reachable_probability
        variance_east = sum(
            weight * (point.east_m - mean_east) ** 2
            for weight, _, point in feasible
        ) / reachable_probability
        variance_north = sum(
            weight * (point.north_m - mean_north) ** 2
            for weight, _, point in feasible
        ) / reachable_probability
        latitude, longitude = self.frame.to_geographic(
            LocalPoint(mean_east, mean_north)
        )
        confidence_scale = math.sqrt(5.991)
        return {
            "code": "INT",
            "label": "예상 요격지점",
            "model": "IMM-FE-PF RELATIVE INTERCEPT",
            "track_id": track_id,
            "vehicle_id": int(vehicle["vehicle_id"]),
            "vehicle_code": f"LM-{int(vehicle['vehicle_id']):02d}",
            "latitude": latitude,
            "longitude": longitude,
            "altitude_m": 0.0,
            "horizon_s": mean_time,
            "max_horizon_s": self.PREDICTION_HORIZON_S,
            "reachable": reachable_probability >= 0.5,
            "reachable_probability": reachable_probability,
            "uncertainty_east_95_m": confidence_scale
            * math.sqrt(max(0.0, variance_east)),
            "uncertainty_north_95_m": confidence_scale
            * math.sqrt(max(0.0, variance_north)),
            "uncertainty_altitude_95_m": 0.0,
            "revision": self.revision,
            "valid_for_s": 3.0,
        }

    def _route_points(
        self,
        current_position: LocalPoint,
        raw_points: list[dict[str, Any]],
    ) -> tuple[LocalPoint, ...]:
        points = [current_position]
        points.extend(
            self.frame.to_local(
                float(point["latitude"]),
                float(point["longitude"]),
            )
            for point in raw_points
        )
        return tuple(points)

    def _arc_candidates(
        self,
        vehicle_id: int,
        target_id: int,
        current_position: LocalPoint,
        search_center: LocalPoint,
        target_center: LocalPoint,
        uncertainty_east_95_m: float,
        uncertainty_north_95_m: float,
    ) -> list[tuple[str, tuple[LocalPoint, ...]]]:
        radial_spacing = self.seeker.track_spacing_m
        radius = radial_spacing / 2.0
        all_radii = []
        while radius < self.SEARCH_RADIUS_M:
            all_radii.append(radius)
            radius += radial_spacing

        # The CPP planner builds every concentric 60-degree arc, then sends the
        # probability/future-encounter shortlist to the expensive evaluator.
        # Keep the same candidate library and a top-K=12 shortlist here: six
        # radial bands, each evaluated in both directions.
        belief_radius = search_center.distance_to(target_center)
        belief_sigma = max(
            radial_spacing,
            (uncertainty_east_95_m + uncertainty_north_95_m) / 4.0,
        )
        prior_mean = self.SEARCH_RADIUS_M * 0.65
        prior_sigma = self.SEARCH_RADIUS_M * 0.15

        def radial_priority(candidate_radius: float) -> float:
            belief_z = (candidate_radius - belief_radius) / belief_sigma
            prior_z = (candidate_radius - prior_mean) / prior_sigma
            return (
                0.65 * math.exp(-0.5 * belief_z * belief_z)
                + 0.35 * math.exp(-0.5 * prior_z * prior_z)
            )

        radii = sorted(
            sorted(
                all_radii,
                key=lambda candidate_radius: (
                    -radial_priority(candidate_radius),
                    abs(candidate_radius - belief_radius),
                ),
            )[:6]
        )
        # Fixed-sector ownership remains exact: LM-N never receives an arc
        # outside its own 60-degree wedge.
        sector_heading_deg = (vehicle_id - 1) * 60.0
        candidates: list[tuple[str, tuple[LocalPoint, ...]]] = []
        for radius_band, radius in enumerate(radii):
            for direction in (-1, 1):
                angles = [
                    sector_heading_deg
                    + direction * (-30.0 + index * (60.0 / 8.0))
                    for index in range(9)
                ]
                arc = tuple(
                    LocalPoint(
                        search_center.east_m
                        + math.sin(math.radians(angle)) * radius,
                        search_center.north_m
                        + math.cos(math.radians(angle)) * radius,
                    )
                    for angle in angles
                )
                signature = (
                    f"T{target_id}-V{vehicle_id}-R{radius:.1f}-"
                    f"D{direction:+d}"
                )
                candidates.append((signature, (current_position, *arc)))
        return candidates

    def _assess_route(
        self,
        vehicle_id: int,
        target_id: int,
        route: tuple[LocalPoint, ...],
        forecast: tuple[ForecastPath, ...],
        signature: str,
        start_elapsed_s: float,
    ) -> RouteAssessment:
        if len(route) < 2:
            return RouteAssessment(
                vehicle_id,
                target_id,
                route,
                0.0,
                self.PREDICTION_HORIZON_S,
                0.0,
                signature,
            )
        total_length = polyline_length(route)
        transit_distance = route[0].distance_to(route[1])
        if not forecast:
            return RouteAssessment(
                vehicle_id,
                target_id,
                route,
                0.0,
                self.PREDICTION_HORIZON_S,
                0.0,
                signature,
            )

        # Vehicle motion and gimbal scan law depend on route/time, not on a
        # target particle.  Precomputing these looks avoids rebuilding the
        # exact same footprint for every forecast sample (the dominant cost
        # in a six-vehicle, six-candidate planning cycle).
        route_looks: list[tuple[int, float, SensorFootprint]] = []
        for time_index, time_s in enumerate(forecast[0].times_s[1:], start=1):
            travelled = min(
                total_length,
                self.TRANSIT_SPEED_MPS * time_s
                if self.TRANSIT_SPEED_MPS * time_s <= transit_distance
                else transit_distance
                + self.SEARCH_SPEED_MPS
                * (time_s - transit_distance / self.TRANSIT_SPEED_MPS),
            )
            if travelled < transit_distance:
                continue
            vehicle_position = point_along_polyline(route, travelled)
            ahead_position = point_along_polyline(
                route,
                min(total_length, travelled + 5.0),
            )
            delta_east = ahead_position.east_m - vehicle_position.east_m
            delta_north = ahead_position.north_m - vehicle_position.north_m
            heading_deg = (
                math.degrees(math.atan2(delta_east, delta_north)) + 360.0
            ) % 360.0
            route_looks.append(
                (
                    time_index,
                    time_s,
                    build_local_footprint(
                        self.seeker,
                        vehicle_id=vehicle_id,
                        vehicle_point=vehicle_position,
                        altitude_m=self.seeker.altitude_m,
                        heading_deg=heading_deg,
                        elapsed_s=start_elapsed_s + time_s,
                    ),
                )
            )

        hit_probability = 0.0
        weighted_time = 0.0
        for target_path in forecast:
            survival_probability = 1.0
            path_expected_time = 0.0
            for time_index, time_s, footprint in route_looks:
                look_probability = footprint.detection_probability_at(
                    target_path.points[time_index]
                )
                first_detection_probability = (
                    survival_probability * look_probability
                )
                path_expected_time += first_detection_probability * time_s
                survival_probability *= 1.0 - look_probability
                if survival_probability <= 1e-6:
                    break
            path_detection_probability = 1.0 - survival_probability
            hit_probability += (
                target_path.weight * path_detection_probability
            )
            weighted_time += target_path.weight * path_expected_time
        hit_probability = min(1.0, hit_probability)
        expected_detection_time = (
            weighted_time / hit_probability
            if hit_probability > 1e-12
            else self.PREDICTION_HORIZON_S
        )
        expected_action_time = (
            hit_probability * expected_detection_time
            + (1.0 - hit_probability) * self.PREDICTION_HORIZON_S
        )
        score = (
            hit_probability / max(1.0, expected_action_time)
            if hit_probability > 0.0
            else 0.0
        )
        return RouteAssessment(
            vehicle_id,
            target_id,
            route,
            hit_probability,
            expected_detection_time,
            score,
            signature,
        )

    def _qualifies(
        self,
        current: RouteAssessment,
        candidate: RouteAssessment,
    ) -> tuple[bool, float, float]:
        denominator = max(abs(current.score), 1e-9)
        improvement_ratio = (candidate.score - current.score) / denominator
        probability_delta = (
            candidate.detection_probability - current.detection_probability
        )
        if current.score <= 1e-9:
            qualifies = candidate.detection_probability >= self.probability_gain
        else:
            qualifies = (
                improvement_ratio >= self.route_improvement_ratio
                and probability_delta >= self.probability_gain
            )
        return qualifies, improvement_ratio, probability_delta

    def _route_payload(
        self,
        points: tuple[LocalPoint, ...],
    ) -> tuple[dict[str, Any], ...]:
        payload = []
        for sequence, point in enumerate(points, start=1):
            latitude, longitude = self.frame.to_geographic(point)
            payload.append(
                {
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude_m": self.seeker.altitude_m,
                    "code": f"AUTO{sequence:02d}",
                    "label": f"AUTO ROUTE {sequence}",
                    "point_type": "AUTO_WAYPOINT",
                    "sequence": sequence,
                }
            )
        return tuple(payload)

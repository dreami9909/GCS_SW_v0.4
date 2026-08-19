from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal, Slot

from .fly_state import FlyState
from .site_store import SiteStore


class FlyMapBridge(QObject):
    stateChanged = Signal(str)
    threatSelected = Signal(int)
    mapRightClicked = Signal(float, float, float)
    featureRightClicked = Signal(str, float, float, float)
    mapStatusChanged = Signal(str)

    def __init__(self, store: SiteStore, state: FlyState) -> None:
        super().__init__()
        self.store = store
        self.state = state
        store.subscribe(self.emit_state)

    def emit_state(self) -> None:
        payload = json.dumps(
            self.state.render_dict(self.store),
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.stateChanged.emit(payload)

    @Slot()
    def requestInitialState(self) -> None:
        self.emit_state()

    @Slot(int)
    def reportThreatSelected(self, track_id: int) -> None:
        self.threatSelected.emit(track_id)

    @Slot(float, float, float)
    def reportMapRightClick(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        self.mapRightClicked.emit(latitude, longitude, altitude)

    @Slot(str, float, float, float)
    def reportFeatureRightClick(
        self,
        feature_key: str,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> None:
        self.featureRightClicked.emit(
            feature_key,
            latitude,
            longitude,
            altitude,
        )

    @Slot(str)
    def reportStatus(self, message: str) -> None:
        self.mapStatusChanged.emit(message)

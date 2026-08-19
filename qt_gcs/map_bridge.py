from __future__ import annotations

import json

from PySide6.QtCore import QObject, Signal, Slot

from .site_store import SiteStore


class MapBridge(QObject):
    """Qt WebChannel bridge between the Python plan model and map JavaScript."""

    planChanged = Signal(str)
    mapClicked = Signal(float, float, float)
    mapRightClicked = Signal(float, float, float)
    featureRightClicked = Signal(str, float, float, float)
    featureSelected = Signal(str)
    mapStatusChanged = Signal(str)

    def __init__(self, store: SiteStore) -> None:
        super().__init__()
        self.store = store
        store.subscribe(self.emit_plan)

    def emit_plan(self) -> None:
        payload = json.dumps(self.store.render_dict(), ensure_ascii=False)
        self.planChanged.emit(payload)

    @Slot()
    def requestInitialState(self) -> None:
        self.emit_plan()

    @Slot(float, float, float)
    def reportMapClick(self, latitude: float, longitude: float, altitude: float) -> None:
        self.mapClicked.emit(latitude, longitude, altitude)

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
    def reportFeatureSelected(self, code: str) -> None:
        self.featureSelected.emit(code)

    @Slot(str)
    def reportStatus(self, message: str) -> None:
        self.mapStatusChanged.emit(message)

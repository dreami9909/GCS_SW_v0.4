from __future__ import annotations

import json
from pathlib import Path


ASSET_DIR = Path(__file__).resolve().parent / "assets"


def load_fly_map_html(api_key: str) -> tuple[str, str]:
    if api_key:
        template = (ASSET_DIR / "google_fly_3d.html").read_text(encoding="utf-8")
        return (
            template.replace("__API_KEY_JSON__", json.dumps(api_key)),
            "GOOGLE 3D HYBRID",
        )
    return (
        (ASSET_DIR / "offline_fly_3d.html").read_text(encoding="utf-8"),
        "OFFLINE TACTICAL PREVIEW",
    )


def load_seeker_map_html(api_key: str) -> str:
    template = (ASSET_DIR / "seeker_map_2d.html").read_text(encoding="utf-8")
    return template.replace("__API_KEY_JSON__", json.dumps(api_key))

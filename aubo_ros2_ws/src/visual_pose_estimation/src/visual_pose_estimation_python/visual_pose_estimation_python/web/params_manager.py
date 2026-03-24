from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from .resources import resolve_web_paths


class ParamsManager:
    """Centralize debug threshold persistence for the web bridge."""

    DEFAULTS = {
        "binary_threshold_min": 0,
        "binary_threshold_max": 2149,
        "component_min_area": 10,
        "component_max_area": 100000,
        "component_min_aspect_ratio": 0.3,
        "component_max_aspect_ratio": 4.0,
        "component_min_width": 60,
        "component_min_height": 60,
        "component_max_count": 3,
        "enable_zero_interp": True,
        "enable_smooth_edges": True,
        "smooth_edges_blur_sigma": 0,
        "use_rembg": False,
    }

    def __init__(self, config_path: Optional[str | Path] = None):
        if config_path is None:
            self.config_path = resolve_web_paths().debug_thresholds_file
        else:
            self.config_path = Path(config_path)

        self.defaults = dict(self.DEFAULTS)
        self._cache: Optional[dict] = None
        self._cache_dirty = False

    def load(self, force_reload: bool = False) -> dict:
        if not force_reload and self._cache is not None and not self._cache_dirty:
            return self._cache.copy()

        try:
            if self.config_path.exists():
                with open(self.config_path, "r", encoding="utf-8") as file_obj:
                    params = json.load(file_obj)
            else:
                params = {}

            for key, value in self.defaults.items():
                params.setdefault(key, value)

            self._cache = params.copy()
            self._cache_dirty = False
            return params
        except Exception:
            self._cache = self.defaults.copy()
            self._cache_dirty = False
            return self.defaults.copy()

    def save(self, params: Optional[dict] = None) -> bool:
        try:
            if params is None:
                params = self.load() if self._cache is None else self._cache.copy()

            for key, value in self.defaults.items():
                params.setdefault(key, value)

            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.config_path, "w", encoding="utf-8") as file_obj:
                json.dump(params, file_obj, indent=2, ensure_ascii=False)

            self._cache = params.copy()
            self._cache_dirty = False
            return True
        except Exception:
            return False

    def update(self, key: str, value) -> bool:
        params = self.load()
        params[key] = value
        self._cache = params.copy()
        self._cache_dirty = True
        return self.save(params)

    def update_batch(self, updates: dict) -> bool:
        params = self.load()
        params.update(updates)
        self._cache = params.copy()
        self._cache_dirty = True
        return self.save(params)

    def reload(self) -> dict:
        return self.load(force_reload=True)

    def get_preprocessor_params(self) -> dict:
        params = self.load()
        return {
            "binary_threshold_min": params.get("binary_threshold_min", self.defaults["binary_threshold_min"]),
            "binary_threshold_max": params.get("binary_threshold_max", self.defaults["binary_threshold_max"]),
            "component_min_area": params.get("component_min_area", self.defaults["component_min_area"]),
            "component_max_area": params.get("component_max_area", self.defaults["component_max_area"]),
            "component_min_aspect_ratio": params.get(
                "component_min_aspect_ratio", self.defaults["component_min_aspect_ratio"]
            ),
            "component_max_aspect_ratio": params.get(
                "component_max_aspect_ratio", self.defaults["component_max_aspect_ratio"]
            ),
            "component_min_width": params.get("component_min_width", self.defaults["component_min_width"]),
            "component_min_height": params.get("component_min_height", self.defaults["component_min_height"]),
            "component_max_count": params.get("component_max_count", self.defaults["component_max_count"]),
            "enable_zero_interp": params.get("enable_zero_interp", self.defaults["enable_zero_interp"]),
            "enable_smooth_edges": params.get("enable_smooth_edges", self.defaults["enable_smooth_edges"]),
            "smooth_edges_blur_sigma": params.get(
                "smooth_edges_blur_sigma", self.defaults["smooth_edges_blur_sigma"]
            ),
        }

    def get_feature_extractor_params(self) -> dict:
        params = self.load()
        return {
            "component_min_area": params.get("component_min_area", self.defaults["component_min_area"]),
            "component_max_area": params.get("component_max_area", self.defaults["component_max_area"]),
            "component_min_aspect_ratio": params.get(
                "component_min_aspect_ratio", self.defaults["component_min_aspect_ratio"]
            ),
            "component_max_aspect_ratio": params.get(
                "component_max_aspect_ratio", self.defaults["component_max_aspect_ratio"]
            ),
            "component_min_width": params.get("component_min_width", self.defaults["component_min_width"]),
            "component_min_height": params.get("component_min_height", self.defaults["component_min_height"]),
        }

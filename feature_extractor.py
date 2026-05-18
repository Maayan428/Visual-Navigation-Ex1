import json
import math
import os

import cv2
import numpy as np

MAP_H      = 4000
MAP_W      = 8000
MAP_SEED   = 42
PATCH_W    = 640
PATCH_H    = 360
ORB_N      = 500
MARGIN     = 0.05   # 5% border margin around GPS bounding box

LAT_PER_M  = 1.0 / 111111.0


def _lon_per_m(lat_deg: float) -> float:
    """Degrees of longitude per metre at the given latitude."""
    return 1.0 / (111111.0 * math.cos(math.radians(lat_deg)))


def _build_map() -> np.ndarray:
    """Generate a deterministic synthetic overhead aerial map."""
    rng = np.random.default_rng(MAP_SEED)
    canvas = np.zeros((MAP_H, MAP_W), dtype=np.uint8)

    # Layer 1 — multi-scale checkerboard (gives reliable ORB corner anchors)
    gy, gx = np.mgrid[:MAP_H, :MAP_W]
    for scale in [20, 40, 80]:
        mask = (((gx // scale) + (gy // scale)) % 2 == 0)
        canvas[mask] = np.clip(canvas[mask].astype(np.int32) + 30, 0, 255).astype(np.uint8)

    # Layer 2 — random blobs (simulate buildings, trees; breaks periodicity)
    for _ in range(800):
        cx  = rng.integers(0, MAP_W)
        cy  = rng.integers(0, MAP_H)
        r   = rng.integers(3, 41)
        val = int(rng.integers(40, 201))
        oy, ox = np.ogrid[:MAP_H, :MAP_W]
        mask = ((ox - cx) ** 2 + (oy - cy) ** 2) < r ** 2
        canvas[mask] = np.clip(canvas[mask].astype(np.int32) + val, 0, 255).astype(np.uint8)

    # Layer 3 — rectangular outlines (roads, block boundaries)
    for _ in range(60):
        x0  = int(rng.integers(0, MAP_W - 200))
        y0  = int(rng.integers(0, MAP_H - 200))
        x1  = x0 + int(rng.integers(30, 201))
        y1  = y0 + int(rng.integers(20, 121))
        x1  = min(x1, MAP_W - 1)
        y1  = min(y1, MAP_H - 1)
        bv  = int(rng.integers(60, 201))
        canvas[y0:y1, x0]  = bv
        canvas[y0:y1, x1]  = bv
        canvas[y0, x0:x1]  = bv
        canvas[y1, x0:x1]  = bv

    canvas = cv2.GaussianBlur(canvas, (5, 5), 1.5)
    return canvas


class FeatureExtractor:
    """
    Extracts ORB features from geo-referenced overhead patches.
    Uses a real satellite map (real_map.png) when available; falls back to a
    procedurally generated synthetic texture otherwise.  Both the database build
    and navigation queries share the same canvas so overlapping footprints
    produce matching features.
    """

    def __init__(self, all_frames: list, out_dir: str):
        """Build or load the overhead map canvas covering all_frames' GPS bounding box."""
        self.out_dir  = out_dir
        self.cfg_path = os.path.join(out_dir, 'map_config.json')
        os.makedirs(out_dir, exist_ok=True)

        real_map_path = os.path.join(out_dir, 'real_map.png')

        if os.path.exists(real_map_path) and os.path.exists(self.cfg_path):
            # Satellite map already downloaded by map_fetcher.py — use it.
            print("  Loading real satellite map (real_map.png)...")
            self.canvas   = cv2.imread(real_map_path, cv2.IMREAD_GRAYSCALE)
            self.map_path = real_map_path
            with open(self.cfg_path) as fh:
                cfg = json.load(fh)
            self.lat_min  = cfg['lat_min']
            self.lat_max  = cfg['lat_max']
            self.lon_min  = cfg['lon_min']
            self.lon_max  = cfg['lon_max']
        else:
            # Compute bounding box from all frames and build/load synthetic map.
            lats = [f['lat'] for f in all_frames if 'lat' in f]
            lons = [f['lon'] for f in all_frames if 'lon' in f]

            lat_range = max(lats) - min(lats)
            lon_range = max(lons) - min(lons)

            self.lat_min = min(lats) - lat_range * MARGIN
            self.lat_max = max(lats) + lat_range * MARGIN
            self.lon_min = min(lons) - lon_range * MARGIN
            self.lon_max = max(lons) + lon_range * MARGIN

            config = {
                'lat_min': self.lat_min, 'lat_max': self.lat_max,
                'lon_min': self.lon_min, 'lon_max': self.lon_max,
                'map_h': MAP_H, 'map_w': MAP_W,
            }
            with open(self.cfg_path, 'w') as fh:
                json.dump(config, fh)

            synth_path = os.path.join(out_dir, 'synthetic_map.npy')
            self.map_path = synth_path
            if os.path.exists(synth_path):
                print("  Loading cached synthetic map...")
                self.canvas = np.load(synth_path)
            else:
                print("  Generating synthetic map (8000×4000 px)...")
                self.canvas = _build_map()
                np.save(synth_path, self.canvas)
                print(f"  Saved: {synth_path}")

        self._map_h, self._map_w = self.canvas.shape

    @classmethod
    def from_saved(cls, out_dir: str) -> 'FeatureExtractor':
        """Load a previously built extractor without rebuilding the map."""
        cfg_path = os.path.join(out_dir, 'map_config.json')
        if not os.path.exists(cfg_path):
            raise FileNotFoundError(
                f"map_config.json not found in {out_dir}. "
                "Run --mode preprocess first."
            )
        with open(cfg_path) as fh:
            config = json.load(fh)

        obj = object.__new__(cls)
        obj.out_dir  = out_dir
        obj.cfg_path = cfg_path
        obj.lat_min  = config['lat_min']
        obj.lat_max  = config['lat_max']
        obj.lon_min  = config['lon_min']
        obj.lon_max  = config['lon_max']

        real_map_path = os.path.join(out_dir, 'real_map.png')
        synth_path    = os.path.join(out_dir, 'synthetic_map.npy')

        if os.path.exists(real_map_path):
            print("  Loading real satellite map (real_map.png)...")
            obj.canvas   = cv2.imread(real_map_path, cv2.IMREAD_GRAYSCALE)
            obj.map_path = real_map_path
        elif os.path.exists(synth_path):
            obj.canvas   = np.load(synth_path)
            obj.map_path = synth_path
        else:
            raise FileNotFoundError(
                f"Neither real_map.png nor synthetic_map.npy found in {out_dir}. "
                "Run --mode preprocess first."
            )

        obj._map_h, obj._map_w = obj.canvas.shape
        return obj

    def _gps_to_pixel(self, lat: float, lon: float):
        """Convert GPS coordinates to (row, col) pixel position in the map canvas."""
        row = int((self.lat_max - lat) / (self.lat_max - self.lat_min) * self._map_h)
        col = int((lon - self.lon_min) / (self.lon_max - self.lon_min) * self._map_w)
        return row, col

    def extract_patch(self, frame: dict) -> np.ndarray:
        """
        Crop the synthetic map region corresponding to this frame's ground footprint.
        Returns a grayscale uint8 image resized to (PATCH_W, PATCH_H).
        """
        lat = frame['center_lat']
        lon = frame['center_lon']
        w_m = frame['width_m']
        h_m = frame['height_m']

        # NW corner (max lat, min lon) → top-left pixel
        nw_row, nw_col = self._gps_to_pixel(
            lat + (h_m / 2.0) * LAT_PER_M,
            lon - (w_m / 2.0) * _lon_per_m(lat),
        )
        # SE corner (min lat, max lon) → bottom-right pixel
        se_row, se_col = self._gps_to_pixel(
            lat - (h_m / 2.0) * LAT_PER_M,
            lon + (w_m / 2.0) * _lon_per_m(lat),
        )

        r0 = max(0, nw_row)
        r1 = min(self._map_h, se_row)
        c0 = max(0, nw_col)
        c1 = min(self._map_w, se_col)

        if r1 - r0 < 10 or c1 - c0 < 10:
            return np.zeros((PATCH_H, PATCH_W), dtype=np.uint8)

        crop  = self.canvas[r0:r1, c0:c1]
        patch = cv2.resize(crop, (PATCH_W, PATCH_H), interpolation=cv2.INTER_LINEAR)
        return patch

    def extract_features(self, patch: np.ndarray):
        """Run ORB on patch; return (kp_ser, descriptors) or (None, None) if no features found."""
        orb = cv2.ORB_create(nfeatures=ORB_N)
        kp, des = orb.detectAndCompute(patch, None)

        if kp is None or len(kp) == 0 or des is None:
            return None, None

        kp_ser = [
            (float(k.pt[0]), float(k.pt[1]), float(k.size),
             float(k.angle), float(k.response), int(k.octave))
            for k in kp
        ]
        return kp_ser, des


def extract_features_from_image(image: np.ndarray):
    """
    Real mode: extract ORB features from an actual image (numpy array).
    Returns (kp_ser, descriptors) or (None, None) if no features found.
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    orb = cv2.ORB_create(nfeatures=ORB_N)
    kp, des = orb.detectAndCompute(gray, None)

    if kp is None or len(kp) == 0 or des is None:
        return None, None

    kp_ser = [
        (float(k.pt[0]), float(k.pt[1]), float(k.size),
         float(k.angle), float(k.response), int(k.octave))
        for k in kp
    ]
    return kp_ser, des

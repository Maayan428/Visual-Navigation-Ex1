"""
map_fetcher.py — Download a real satellite tile for the combined GPS bounding box
of two SRT flights and save it as real_map.png.

Uses the ESRI World Imagery service (no API key required).
After running this script, feature_extractor.py will automatically prefer
real_map.png over the procedurally generated synthetic_map.npy.

Usage
-----
  python map_fetcher.py --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT
  python map_fetcher.py --srt1 data/DJI_0006.SRT --srt2 data/DJI_0007.SRT --out-dir out/

Tile source
-----------
  ESRI World Imagery
  https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer
  Free for non-commercial use. Resolution ≈ 0.5 m/px at zoom 18 in Israel.
"""

import argparse
import json
import math
import os

import cv2
import numpy as np
from staticmap import StaticMap

from srt_parser import parse_srt

_ESRI_AERIAL = (
    'https://server.arcgisonline.com/ArcGIS/rest/services/'
    'World_Imagery/MapServer/tile/{z}/{y}/{x}'
)
_MARGIN = 1.35   # 35 % padding around the GPS bounding box
_MAX_PX = 4096   # hard cap per side to stay within memory


# ── Slippy-map (Web Mercator) helpers ─────────────────────────────────────────

def _lon_to_tx(lon: float, zoom: int) -> float:
    return (lon + 180.0) / 360.0 * (2 ** zoom)


def _lat_to_ty(lat: float, zoom: int) -> float:
    lat_r = math.radians(lat)
    return (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * (2 ** zoom)


def _tx_to_lon(tx: float, zoom: int) -> float:
    return tx / (2 ** zoom) * 360.0 - 180.0


def _ty_to_lat(ty: float, zoom: int) -> float:
    return math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / (2 ** zoom)))))


def _best_zoom_and_size(lat_min: float, lat_max: float,
                        lon_min: float, lon_max: float) -> tuple:
    """
    Return (zoom, img_w, img_h) — the highest zoom where the bbox fits inside
    _MAX_PX×_MAX_PX pixels with _MARGIN padding.
    """
    lat_c = (lat_min + lat_max) / 2.0
    lon_span = (lon_max - lon_min) * _MARGIN
    lat_span = (lat_max - lat_min) * _MARGIN * math.cos(math.radians(lat_c))  # Mercator approx

    for zoom in range(20, 7, -1):
        deg_per_px = 360.0 / (2 ** zoom) / 256.0          # longitude degrees per pixel
        img_w = int(lon_span / deg_per_px) + 256           # +256 pixel buffer
        img_h = int(lat_span / deg_per_px) + 256
        if img_w <= _MAX_PX and img_h <= _MAX_PX:
            return zoom, img_w, img_h

    return 10, _MAX_PX, _MAX_PX


def _rendered_extent(center_lat: float, center_lon: float,
                     zoom: int, img_w: int, img_h: int) -> tuple:
    """
    Compute the exact (lat_min, lat_max, lon_min, lon_max) covered by the
    rendered image given its center and zoom level.
    """
    tx_c = _lon_to_tx(center_lon, zoom)
    ty_c = _lat_to_ty(center_lat, zoom)
    half_w = (img_w / 2.0) / 256.0    # in tile units (1 tile = 256 px)
    half_h = (img_h / 2.0) / 256.0

    lon_min = _tx_to_lon(tx_c - half_w, zoom)
    lon_max = _tx_to_lon(tx_c + half_w, zoom)
    lat_max = _ty_to_lat(ty_c - half_h, zoom)   # smaller ty → higher lat
    lat_min = _ty_to_lat(ty_c + half_h, zoom)
    return lat_min, lat_max, lon_min, lon_max


# ── Public API ────────────────────────────────────────────────────────────────

def fetch_map(frames_all: list, out_dir: str) -> tuple:
    """
    Download satellite imagery covering all frames.

    Saves real_map.png (grayscale) and map_config.json in out_dir.
    Returns (lat_min, lat_max, lon_min, lon_max) of the downloaded image.
    """
    lats = [f['lat'] for f in frames_all]
    lons = [f['lon'] for f in frames_all]
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)
    center_lat = (lat_min + lat_max) / 2.0
    center_lon = (lon_min + lon_max) / 2.0

    zoom, img_w, img_h = _best_zoom_and_size(lat_min, lat_max, lon_min, lon_max)

    print(f"GPS bbox : lat [{lat_min:.6f}, {lat_max:.6f}]  "
          f"lon [{lon_min:.6f}, {lon_max:.6f}]")
    print(f"Map size : {img_w} × {img_h} px  |  zoom {zoom}  |  "
          f"source: ESRI World Imagery")

    sm = StaticMap(img_w, img_h, url_template=_ESRI_AERIAL)

    print("Downloading tiles…  (this may take a few seconds)")
    pil_img = sm.render(zoom=zoom, center=[center_lon, center_lat])

    img_np = np.array(pil_img)
    if img_np.ndim == 3:
        gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    else:
        gray = img_np

    os.makedirs(out_dir, exist_ok=True)
    map_path = os.path.join(out_dir, 'real_map.png')
    cv2.imwrite(map_path, gray)
    print(f"Saved: {map_path}")

    actual_lat_min, actual_lat_max, actual_lon_min, actual_lon_max = \
        _rendered_extent(center_lat, center_lon, zoom, img_w, img_h)

    config = {
        'lat_min':  actual_lat_min,
        'lat_max':  actual_lat_max,
        'lon_min':  actual_lon_min,
        'lon_max':  actual_lon_max,
        'map_h':    img_h,
        'map_w':    img_w,
        'source':   'satellite',
        'zoom':     zoom,
    }
    cfg_path = os.path.join(out_dir, 'map_config.json')
    with open(cfg_path, 'w') as fh:
        json.dump(config, fh, indent=2)
    print(f"Saved: {cfg_path}")
    print(f"Actual extent: lat [{actual_lat_min:.6f}, {actual_lat_max:.6f}]  "
          f"lon [{actual_lon_min:.6f}, {actual_lon_max:.6f}]")

    lat_span_m = (actual_lat_max - actual_lat_min) * 111111
    lon_span_m = ((actual_lon_max - actual_lon_min) * 111111
                  * math.cos(math.radians(center_lat)))
    print(f"Coverage  : {lat_span_m:.0f} m × {lon_span_m:.0f} m  "
          f"|  {lat_span_m / img_h:.2f} m/px × {lon_span_m / img_w:.2f} m/px")

    return actual_lat_min, actual_lat_max, actual_lon_min, actual_lon_max


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Download a real satellite map tile for a pair of DJI SRT flights.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--srt1', required=True, help='Database SRT file (e.g. DJI_0017.SRT)')
    parser.add_argument('--srt2', required=True, help='Query SRT file   (e.g. DJI_0019.SRT)')
    parser.add_argument('--out-dir', default='out', help='Output directory (default: out/)')
    args = parser.parse_args()

    print(f"Parsing {os.path.basename(args.srt1)} …")
    frames1 = parse_srt(args.srt1)
    print(f"  {len(frames1)} frames")

    print(f"Parsing {os.path.basename(args.srt2)} …")
    frames2 = parse_srt(args.srt2)
    print(f"  {len(frames2)} frames")

    fetch_map(frames1 + frames2, args.out_dir)

    print("\nDone. Run the experiment with:")
    print(f"  python main.py --mode experiment "
          f"--srt1 {args.srt1} --srt2 {args.srt2} --out-dir {args.out_dir}")

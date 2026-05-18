"""
camera_path.py — Compute the ground point the camera center ray hits for each frame.

Geometry
--------
DJI gimbal pitch convention: 0° = horizontal, -90° = nadir (straight down).
At pitch = -60° the camera is tilted 30° from nadir (60° below horizontal).

                  drone
                    |  \\  <- camera center ray
                    |   \\
              alt   |    \\
                    |     \\  angle from nadir = 90 - 60 = 30 deg
                    |      \\
    ________________|_______*____________  ground
                  nadir   camera center point
                    |←  d  →|
                  d = alt × tan(30°)

The camera center point ("coordinate of the center point of the video") lies
at distance d in front of the drone along its flight heading.

Outputs
-------
  out/drone_path.kml   – LineString of raw drone GPS positions (blue)
  out/camera_path.kml  – LineString of camera center ground points (orange)

Usage
-----
  python camera_path.py                          # defaults to data/DJI_0017.SRT
  python camera_path.py data/DJI_0017.SRT out/
"""

import math
import os
import sys

import simplekml

from srt_parser import parse_srt

# ── Camera constants ──────────────────────────────────────────────────────────
GIMBAL_PITCH_DEG = -60.0   # DJI convention (0 = horizontal, -90 = nadir)
CAMERA_FOV_DEG   = 82.1    # horizontal field of view

# Derived: tilt from nadir = 90 + pitch  →  90 + (-60) = 30°
TILT_FROM_NADIR  = 90.0 + GIMBAL_PITCH_DEG   # degrees
TAN_TILT         = math.tan(math.radians(TILT_FROM_NADIR))

LAT_PER_M        = 1.0 / 111_111.0


def _lon_per_m(lat_deg: float) -> float:
    return 1.0 / (111_111.0 * math.cos(math.radians(lat_deg)))


def _forward_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Forward azimuth from (lat1, lon1) to (lat2, lon2) in degrees [0, 360).
    0 = North, 90 = East.
    """
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon_r = math.radians(lon2 - lon1)
    x = math.sin(dlon_r) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon_r))
    return math.degrees(math.atan2(x, y)) % 360.0


def _camera_center(lat: float, lon: float, alt: float, heading_deg: float) -> tuple:
    """
    Return (camera_lat, camera_lon) — the ground point where the camera
    center ray intersects the ground plane.

    horizontal_offset = alt × tan(tilt_from_nadir)
                      = alt × tan(90° − |gimbal_pitch|)
                      = alt × tan(30°)
    """
    offset_m  = alt * TAN_TILT
    heading_r = math.radians(heading_deg)

    camera_lat = lat + offset_m * math.cos(heading_r) * LAT_PER_M
    camera_lon = lon + offset_m * math.sin(heading_r) * _lon_per_m(lat)
    return camera_lat, camera_lon, offset_m


def compute_camera_centers(frames: list) -> list:
    """
    For each frame estimate the drone heading from consecutive GPS positions,
    then compute the camera center ground point.

    Returns a list of dicts with keys:
      frame_cnt, timestamp, drone_lat, drone_lon, alt_m,
      heading_deg, offset_m, camera_lat, camera_lon
    """
    n = len(frames)
    results = []

    for i, frame in enumerate(frames):
        lat = frame['lat']
        lon = frame['lon']
        alt = max(frame['rel_alt'], 0.1)   # guard against zero altitude

        # Estimate heading from consecutive positions
        if n == 1:
            heading = 0.0                  # only one frame — assume north
        elif i < n - 1:
            heading = _forward_bearing(lat, lon,
                                       frames[i + 1]['lat'], frames[i + 1]['lon'])
        else:
            heading = _forward_bearing(frames[i - 1]['lat'], frames[i - 1]['lon'],
                                       lat, lon)

        cam_lat, cam_lon, offset_m = _camera_center(lat, lon, alt, heading)

        results.append({
            'frame_cnt':   frame['frame_cnt'],
            'timestamp':   frame.get('timestamp', ''),
            'drone_lat':   lat,
            'drone_lon':   lon,
            'alt_m':       alt,
            'heading_deg': round(heading, 2),
            'offset_m':    round(offset_m, 3),
            'camera_lat':  cam_lat,
            'camera_lon':  cam_lon,
        })

    return results


def _write_kml(coords_lonlat: list, path: str, name: str, color: str) -> None:
    """Write a KML file with a single LineString. coords_lonlat: [(lon, lat), ...]."""
    kml = simplekml.Kml()
    if len(coords_lonlat) >= 2:
        ls           = kml.newlinestring(name=name)
        ls.coords    = coords_lonlat
        ls.style.linestyle.color = color
        ls.style.linestyle.width = 3
    kml.save(path)


def run(srt_path: str, out_dir: str) -> list:
    os.makedirs(out_dir, exist_ok=True)

    print(f"Parsing {os.path.basename(srt_path)}...")
    frames = parse_srt(srt_path)
    print(f"  {len(frames)} sampled frames")

    # Print camera geometry summary
    print(f"\nCamera geometry")
    print(f"  Gimbal pitch          : {GIMBAL_PITCH_DEG}°  (DJI: 0=horizontal, -90=nadir)")
    print(f"  Tilt from nadir       : {TILT_FROM_NADIR}°  (= 90 + {GIMBAL_PITCH_DEG})")
    print(f"  Horizontal FOV        : {CAMERA_FOV_DEG}°")
    print(f"  Ground offset formula : alt × tan({TILT_FROM_NADIR:.0f}°) = alt × {TAN_TILT:.4f}")
    print(f"  Example at alt=50 m   : {50 * TAN_TILT:.2f} m forward offset")
    print(f"  Example at alt=100 m  : {100 * TAN_TILT:.2f} m forward offset")

    results = compute_camera_centers(frames)

    # Export KML files
    drone_kml_path  = os.path.join(out_dir, 'drone_path.kml')
    camera_kml_path = os.path.join(out_dir, 'camera_path.kml')

    _write_kml(
        [(r['drone_lon'],  r['drone_lat'])  for r in results],
        drone_kml_path,
        'Drone GPS path',
        simplekml.Color.blue,
    )
    _write_kml(
        [(r['camera_lon'], r['camera_lat']) for r in results],
        camera_kml_path,
        'Camera center path',
        simplekml.Color.orange,
    )
    print(f"\nSaved: {drone_kml_path}")
    print(f"Saved: {camera_kml_path}")

    # Print first 5 rows
    col = '{:>10}  {:>11}  {:>11}  {:>11}  {:>11}  {:>9}  {:>9}'
    header = col.format(
        'frame_cnt', 'drone_lat', 'drone_lon',
        'camera_lat', 'camera_lon', 'heading°', 'offset_m',
    )
    print(f"\nFirst 5 frames:\n{header}")
    print('─' * len(header))
    for r in results[:5]:
        print(col.format(
            r['frame_cnt'],
            f"{r['drone_lat']:.6f}",
            f"{r['drone_lon']:.6f}",
            f"{r['camera_lat']:.6f}",
            f"{r['camera_lon']:.6f}",
            f"{r['heading_deg']:.1f}",
            f"{r['offset_m']:.2f}",
        ))

    return results


if __name__ == '__main__':
    srt_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join('data', 'DJI_0017.SRT')
    out_dir  = sys.argv[2] if len(sys.argv) > 2 else 'out'
    run(srt_path, out_dir)

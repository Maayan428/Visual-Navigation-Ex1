import math

SENSOR_WIDTH_MM = 9.6
FOCAL_LEN_MM    = 8.8   # actual physical focal length (not 35mm equivalent)
IMAGE_W         = 1920
IMAGE_H         = 1080
LAT_PER_M       = 1.0 / 111111.0


def _lon_per_m(lat_deg: float) -> float:
    """Degrees of longitude per metre at the given latitude."""
    return 1.0 / (111111.0 * math.cos(math.radians(lat_deg)))


def compute_footprint(frame: dict) -> dict:
    """Nadir ground footprint for a single frame; returns center, dimensions, GSD, and corner GPS coords."""
    lat  = frame['lat']
    lon  = frame['lon']
    alt  = frame['rel_alt']

    if alt <= 0:
        alt = 0.1  # guard against zero division

    gsd      = (alt * SENSOR_WIDTH_MM) / (FOCAL_LEN_MM * IMAGE_W)  # ground sample distance: metres per pixel at nadir
    width_m  = gsd * IMAGE_W
    height_m = gsd * IMAGE_H

    half_lat = (height_m / 2.0) * LAT_PER_M
    half_lon = (width_m  / 2.0) * _lon_per_m(lat)

    corners = [
        (lat + half_lat, lon - half_lon),  # NW
        (lat + half_lat, lon + half_lon),  # NE
        (lat - half_lat, lon + half_lon),  # SE
        (lat - half_lat, lon - half_lon),  # SW
    ]

    return {
        'center_lat':   lat,
        'center_lon':   lon,
        'width_m':      width_m,
        'height_m':     height_m,
        'gsd_m_per_px': gsd,
        'corners':      corners,
    }


def compute_all_footprints(frames: list) -> list:
    """Merge footprint fields into each frame dict and return the augmented list."""
    for frame in frames:
        fp = compute_footprint(frame)
        frame.update(fp)
    return frames

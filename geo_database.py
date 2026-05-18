import json
import os

import numpy as np

from feature_extractor import extract_features_from_image


def build_database(frames: list, extractor, out_dir: str, video_frames: dict = None):
    """Build a geo-referenced ORB feature database from footprint-augmented frames.
    Saves geo_db.json and geo_db_desc.npy; falls back to synthetic patches when video_frames is absent."""
    os.makedirs(out_dir, exist_ok=True)

    records        = []
    all_descriptors = []
    desc_cursor    = 0
    skipped        = 0

    total = len(frames)
    for i, frame in enumerate(frames):
        if (i + 1) % 50 == 0 or i == 0 or i == total - 1:
            print(f"\r  Processing frame {i+1}/{total}...", end='', flush=True)

        fc = frame['frame_cnt']
        if video_frames is not None and fc in video_frames:
            kp_ser, des = extract_features_from_image(video_frames[fc])
            source = 'video'
        else:
            patch = extractor.extract_patch(frame)
            kp_ser, des = extractor.extract_features(patch)
            source = 'synthetic'

        if des is None:
            skipped += 1
            continue

        n = des.shape[0]
        record = {
            'frame_id':   fc,
            'timestamp':  frame.get('timestamp', ''),
            'lat':        frame['lat'],
            'lon':        frame['lon'],
            'alt':        frame['rel_alt'],
            'source':     source,
            'footprint': {
                'center_lat':   frame['center_lat'],
                'center_lon':   frame['center_lon'],
                'width_m':      frame['width_m'],
                'height_m':     frame['height_m'],
                'gsd_m_per_px': frame['gsd_m_per_px'],
                'corners':      frame['corners'],
            },
            'keypoints':  kp_ser,
            'desc_index': desc_cursor,
            'desc_count': n,
        }
        records.append(record)
        all_descriptors.append(des)
        desc_cursor += n

    print()

    if not records:
        raise RuntimeError("No frames with features found — database is empty.")

    stacked = np.vstack(all_descriptors)  # shape (total_kp, 32)

    json_path = os.path.join(out_dir, 'geo_db.json')
    npy_path  = os.path.join(out_dir, 'geo_db_desc.npy')

    with open(json_path, 'w') as f:
        json.dump(records, f)

    np.save(npy_path, stacked)

    n_video = sum(1 for r in records if r.get('source') == 'video')
    n_synth = len(records) - n_video
    print(f"  Database: {len(records)} frames, {desc_cursor} keypoints")
    print(f"  Sources: {n_video} real-video  |  {n_synth} synthetic")
    print(f"  Saved: {json_path}")
    print(f"  Saved: {npy_path}")
    if skipped:
        print(f"  Skipped (no features): {skipped} frames")

    return records, stacked


def load_database(out_dir: str):
    """
    Load the geo database from out_dir.
    Returns (records: list[dict], stacked: np.ndarray).
    """
    json_path = os.path.join(out_dir, 'geo_db.json')
    npy_path  = os.path.join(out_dir, 'geo_db_desc.npy')

    for p in (json_path, npy_path):
        if not os.path.exists(p):
            raise FileNotFoundError(
                f"{p} not found. Run --mode preprocess first."
            )

    with open(json_path) as f:
        records = json.load(f)

    stacked = np.load(npy_path)
    return records, stacked

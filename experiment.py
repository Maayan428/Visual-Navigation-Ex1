import math
import os

import pandas as pd
import simplekml

from footprint import compute_all_footprints
from srt_parser import parse_srt


def haversine(lat1: float, lon1: float, lat2: float, lon2: float, R: float = 6_371_000.0) -> float:
    """Return the great-circle distance in metres between two GPS coordinates."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2  # spherical law of cosines
    return 2 * R * math.asin(math.sqrt(a))


def _write_kml(coords: list, path: str, name: str, color: str) -> None:
    """Write a KML LineString. coords: list of (lon, lat) tuples (KML order)."""
    kml = simplekml.Kml()
    if len(coords) >= 2:
        ls = kml.newlinestring(name=name)
        ls.coords = coords
        ls.style.linestyle.color = color
        ls.style.linestyle.width = 3
    kml.save(path)


def run_experiment(srt1_path: str, srt2_path: str, out_dir: str,
                   video_frames_db: dict = None,
                   video_frames_query: dict = None) -> None:
    """Run the full localization experiment: parse both SRTs, build DB from srt1, locate srt2 frames.
    Outputs experiment_results.csv and two KML paths to out_dir."""
    from feature_extractor import FeatureExtractor, extract_features_from_image
    from geo_database import build_database, load_database
    from navigator import Navigator

    os.makedirs(out_dir, exist_ok=True)

    # ── Phase 1: parse both SRT files ──────────────────────────────────────────
    print("Parsing SRT files...")
    db_frames = parse_srt(srt1_path)
    for f in db_frames:
        f['source_file'] = os.path.basename(srt1_path)
    db_frames = compute_all_footprints(db_frames)
    print(f"  {os.path.basename(srt1_path)}: {len(db_frames)} sampled frames")

    q_frames = parse_srt(srt2_path)
    for f in q_frames:
        f['source_file'] = os.path.basename(srt2_path)
    q_frames = compute_all_footprints(q_frames)
    print(f"  {os.path.basename(srt2_path)}: {len(q_frames)} sampled frames")

    # ── Phase 2: build feature extractor (shared synthetic map) ───────────────
    print("Building feature extractor (synthetic map)...")
    all_frames = db_frames + q_frames
    extractor  = FeatureExtractor(all_frames, out_dir)

    # ── Phase 3: build geo-database from srt1 ─────────────────────────────────
    # ── Source-mismatch warning ────────────────────────────────────────────────
    if video_frames_db is not None and video_frames_query is None:
        print("WARNING: DB will use real video frames but no query video was provided.")
        print("         ORB features from real video won't match synthetic patches.")
        print("         Pass --video2 for consistent feature sources and better accuracy.")

    print("Extracting features and building geo-database...")
    records, stacked = build_database(db_frames, extractor, out_dir,
                                      video_frames=video_frames_db)

    nav = Navigator(records, stacked)

    # ── Phase 4: navigate srt2 frames ─────────────────────────────────────────
    print(f"Navigating {len(q_frames)} query frames...")
    rows     = []
    gt_path  = []   # (lon, lat) for KML ground-truth
    est_path = []   # (lon, lat) for KML estimated

    for i, frame in enumerate(q_frames):
        print(f"\r  Query {i+1}/{len(q_frames)} (frame_cnt={frame['frame_cnt']})", end='', flush=True)

        true_lat = frame['lat']
        true_lon = frame['lon']
        gt_path.append((true_lon, true_lat))

        fc = frame['frame_cnt']
        if video_frames_query is not None and fc in video_frames_query:
            kp_ser, des = extract_features_from_image(video_frames_query[fc])
        else:
            patch = extractor.extract_patch(frame)
            kp_ser, des = extractor.extract_features(patch)

        if des is None:
            rows.append({
                'frame_cnt':   frame['frame_cnt'],
                'true_lat':    true_lat,
                'true_lon':    true_lon,
                'est_lat':     None,
                'est_lon':     None,
                'error_m':     None,
                'confidence':  None,
            })
            continue

        result = nav.locate(kp_ser, des)

        if result is None:
            est_lat = est_lon = conf = error_m = None
        else:
            est_lat  = result['est_lat']
            est_lon  = result['est_lon']
            conf     = result['confidence']
            error_m  = haversine(true_lat, true_lon, est_lat, est_lon)
            est_path.append((est_lon, est_lat))

        rows.append({
            'frame_cnt':   frame['frame_cnt'],
            'true_lat':    true_lat,
            'true_lon':    true_lon,
            'est_lat':     est_lat,
            'est_lon':     est_lon,
            'error_m':     error_m,
            'confidence':  conf,
        })

    print()

    # ── Phase 5: save outputs ─────────────────────────────────────────────────
    csv_path = os.path.join(out_dir, 'experiment_results.csv')
    df = pd.DataFrame(rows)
    df.to_csv(csv_path, index=False)
    print(f"  Saved: {csv_path}")

    gt_kml_path  = os.path.join(out_dir, 'path_groundtruth.kml')
    est_kml_path = os.path.join(out_dir, 'path_estimated.kml')
    _write_kml(gt_path,  gt_kml_path,  'Ground Truth', simplekml.Color.green)
    _write_kml(est_path, est_kml_path, 'Estimated',    simplekml.Color.red)
    print(f"  Saved: {gt_kml_path}")
    print(f"  Saved: {est_kml_path}")

    # ── Phase 6: print summary ────────────────────────────────────────────────
    errors = df['error_m'].dropna()
    located = len(errors)
    total   = len(q_frames)

    print("\n" + "=" * 50)
    print("EXPERIMENT RESULTS")
    print("=" * 50)
    print(f"Total query frames : {total}")
    print(f"Located            : {located} ({100.0 * located / total:.1f}%)")
    if located > 0:
        print(f"Mean error         : {errors.mean():.2f} m")
        print(f"Median error       : {errors.median():.2f} m")
        print(f"90th pct error     : {errors.quantile(0.9):.2f} m")
        print(f"Max error          : {errors.max():.2f} m")
    print("=" * 50)

    print("\nFirst 5 rows of experiment_results.csv:")
    print(df.head(5).to_string(index=False))

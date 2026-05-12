#!/usr/bin/env python3
"""
Ex1 – Visual Navigation for Drones (GNSS-Denied)

Usage:
  python main.py --mode preprocess --srt data/DJI_0017.SRT
  python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json
  python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT
"""

import argparse
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR  = os.path.join(BASE_DIR, 'out')


def _resolve(path: str) -> str:
    """Return an absolute path; resolve relative paths from the project root."""
    if os.path.isabs(path):
        return path
    return os.path.join(BASE_DIR, path)


def cmd_preprocess(args) -> None:
    from feature_extractor import FeatureExtractor
    from footprint import compute_all_footprints
    from geo_database import build_database
    from srt_parser import parse_srt

    srt1 = _resolve(args.srt)
    # We need the query SRT too so the synthetic map covers both flights.
    # Default to DJI_0019.SRT in data/ if not provided.
    srt2_default = os.path.join(BASE_DIR, 'data', 'DJI_0019.SRT')
    srt2 = srt2_default if os.path.exists(srt2_default) else srt1

    print(f"Parsing {os.path.basename(srt1)} (database source)...")
    db_frames = parse_srt(srt1)
    for f in db_frames:
        f['source_file'] = os.path.basename(srt1)
    db_frames = compute_all_footprints(db_frames)
    print(f"  Sampled {len(db_frames)} frames")

    all_frames = db_frames
    if srt2 != srt1 and os.path.exists(srt2):
        print(f"Parsing {os.path.basename(srt2)} (for map bounding box)...")
        q_frames = parse_srt(srt2)
        for f in q_frames:
            f['source_file'] = os.path.basename(srt2)
        q_frames = compute_all_footprints(q_frames)
        all_frames = db_frames + q_frames
        print(f"  Sampled {len(q_frames)} frames (bounding box only)")

    os.makedirs(OUT_DIR, exist_ok=True)
    print("Building synthetic map...")
    extractor = FeatureExtractor(all_frames, OUT_DIR)

    print("Extracting features and building geo-database...")
    build_database(db_frames, extractor, OUT_DIR)
    print("Preprocessing complete.")


def cmd_navigate(args) -> None:
    from experiment import haversine
    from feature_extractor import FeatureExtractor
    from footprint import compute_all_footprints
    from geo_database import load_database
    from navigator import Navigator
    from srt_parser import parse_srt

    db_dir = os.path.dirname(_resolve(args.db))
    query  = _resolve(args.query)

    print("Loading database...")
    records, stacked = load_database(db_dir)
    nav = Navigator(records, stacked)

    print("Loading extractor...")
    extractor = FeatureExtractor.from_saved(db_dir)

    print(f"Parsing query file: {os.path.basename(query)}")
    q_frames = parse_srt(query)
    q_frames = compute_all_footprints(q_frames)

    # Show results for first 5 sampled frames
    sample = q_frames[:5]
    print(f"\nNavigating {len(sample)} sample frames from {os.path.basename(query)}:\n")

    for frame in sample:
        patch       = extractor.extract_patch(frame)
        kp_ser, des = extractor.extract_features(patch)
        result      = nav.locate(kp_ser, des) if des is not None else None

        true_lat = frame['lat']
        true_lon = frame['lon']
        print(f"Frame {frame['frame_cnt']:5d}  GPS truth: ({true_lat:.6f}, {true_lon:.6f})", end='')

        if result:
            err = haversine(true_lat, true_lon, result['est_lat'], result['est_lon'])
            print(f"  →  est: ({result['est_lat']:.6f}, {result['est_lon']:.6f})"
                  f"  err={err:.1f}m  conf={result['confidence']:.2f}")
        else:
            print("  →  no match found")


def cmd_experiment(args) -> None:
    from experiment import run_experiment
    os.makedirs(OUT_DIR, exist_ok=True)
    run_experiment(_resolve(args.srt1), _resolve(args.srt2), OUT_DIR)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Ex1 – Drone Visual Navigation (GNSS-Denied)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--mode', required=True,
                        choices=['preprocess', 'navigate', 'experiment'],
                        help='Operating mode')

    # preprocess
    parser.add_argument('--srt',   help='SRT file for preprocessing (DJI_0017.SRT)')

    # navigate
    parser.add_argument('--query', help='Query SRT file (DJI_0019.SRT)')
    parser.add_argument('--db',    help='Path to geo_db.json (out/geo_db.json)')

    # experiment
    parser.add_argument('--srt1',  help='Database SRT file (DJI_0017.SRT)')
    parser.add_argument('--srt2',  help='Query SRT file   (DJI_0019.SRT)')

    args = parser.parse_args()

    if args.mode == 'preprocess':
        if not args.srt:
            parser.error('--srt required for preprocess mode')
        if not os.path.exists(_resolve(args.srt)):
            sys.exit(f"File not found: {args.srt}")
        cmd_preprocess(args)

    elif args.mode == 'navigate':
        if not args.query or not args.db:
            parser.error('--query and --db required for navigate mode')
        if not os.path.exists(_resolve(args.query)):
            sys.exit(f"File not found: {args.query}")
        if not os.path.exists(_resolve(args.db)):
            sys.exit(f"Database not found: {args.db}. Run --mode preprocess first.")
        cmd_navigate(args)

    elif args.mode == 'experiment':
        if not args.srt1 or not args.srt2:
            parser.error('--srt1 and --srt2 required for experiment mode')
        for p in (args.srt1, args.srt2):
            if not os.path.exists(_resolve(p)):
                sys.exit(f"File not found: {p}")
        cmd_experiment(args)


if __name__ == '__main__':
    main()

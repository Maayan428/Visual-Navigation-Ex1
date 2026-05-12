#!/usr/bin/env python3
"""
Ex1 – Visual Navigation for Drones (GNSS-Denied)

Usage:
  # Synthetic mode (no video — current default):
  python main.py --mode preprocess --srt data/DJI_0017.SRT
  python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json
  python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT

  # Real-video mode (better accuracy):
  python main.py --mode preprocess --srt data/DJI_0017.SRT --video data/DJI_0017.MP4
  python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json \\
                                   --video data/DJI_0019.MP4
  python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT \\
                                   --video1 data/DJI_0017.MP4 --video2 data/DJI_0019.MP4
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


def _load_video(path: str) -> dict:
    """Load and return video frames dict, or exit with a clear message on failure."""
    from video_processor import extract_video_frames
    abs_path = _resolve(path)
    if not os.path.exists(abs_path):
        sys.exit(f"Video file not found: {path}")
    print(f"Loading video frames from {os.path.basename(abs_path)}...")
    return extract_video_frames(abs_path)


def cmd_preprocess(args, out_dir: str) -> None:
    from feature_extractor import FeatureExtractor
    from footprint import compute_all_footprints
    from geo_database import build_database
    from srt_parser import parse_srt

    srt1 = _resolve(args.srt)
    # Parse companion query SRT (for map bounding box coverage) if it exists alongside.
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

    video_frames = _load_video(args.video) if args.video else None

    os.makedirs(out_dir, exist_ok=True)
    if video_frames is None:
        print("Building synthetic map (no --video provided)...")
    else:
        print("Building synthetic map (used as fallback for missing video frames)...")
    extractor = FeatureExtractor(all_frames, out_dir)

    print("Extracting features and building geo-database...")
    build_database(db_frames, extractor, out_dir, video_frames=video_frames)
    print("Preprocessing complete.")


def cmd_navigate(args, out_dir: str) -> None:
    from experiment import haversine
    from feature_extractor import FeatureExtractor, extract_features_from_image
    from footprint import compute_all_footprints
    from geo_database import load_database
    from navigator import Navigator
    from srt_parser import parse_srt

    db_dir = os.path.dirname(_resolve(args.db))
    query  = _resolve(args.query)

    print("Loading database...")
    records, stacked = load_database(db_dir)
    nav = Navigator(records, stacked)

    # Warn if DB was built from real video but no query video provided.
    db_uses_video = any(r.get('source') == 'video' for r in records)
    if db_uses_video and not args.video:
        print("WARNING: Database was built from real video; --video not provided for query.")
        print("         Accuracy will be poor. Pass --video data/DJI_0019.MP4 for best results.")

    print("Loading synthetic extractor (fallback for frames without video)...")
    extractor = FeatureExtractor.from_saved(db_dir)

    video_frames = _load_video(args.video) if args.video else None

    print(f"Parsing query file: {os.path.basename(query)}")
    q_frames = parse_srt(query)
    q_frames = compute_all_footprints(q_frames)

    sample = q_frames[:5]
    print(f"\nNavigating {len(sample)} sample frames from {os.path.basename(query)}:\n")

    for frame in sample:
        fc = frame['frame_cnt']
        if video_frames is not None and fc in video_frames:
            kp_ser, des = extract_features_from_image(video_frames[fc])
        else:
            patch  = extractor.extract_patch(frame)
            kp_ser, des = extractor.extract_features(patch)

        result   = nav.locate(kp_ser, des) if des is not None else None
        true_lat = frame['lat']
        true_lon = frame['lon']
        print(f"Frame {fc:5d}  GPS truth: ({true_lat:.6f}, {true_lon:.6f})", end='')

        if result:
            err = haversine(true_lat, true_lon, result['est_lat'], result['est_lon'])
            print(f"  →  est: ({result['est_lat']:.6f}, {result['est_lon']:.6f})"
                  f"  err={err:.1f}m  conf={result['confidence']:.2f}")
        else:
            print("  →  no match found")


def cmd_experiment(args, out_dir: str) -> None:
    from experiment import run_experiment
    os.makedirs(out_dir, exist_ok=True)

    video_frames_db    = _load_video(args.video1) if args.video1 else None
    video_frames_query = _load_video(args.video2) if args.video2 else None

    run_experiment(
        _resolve(args.srt1), _resolve(args.srt2), out_dir,
        video_frames_db=video_frames_db,
        video_frames_query=video_frames_query,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Ex1 – Drone Visual Navigation (GNSS-Denied)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--mode', required=True,
                        choices=['preprocess', 'navigate', 'experiment'],
                        help='Operating mode')
    parser.add_argument('--out-dir', default=None,
                        help='Output directory (default: out/ next to main.py)')

    # preprocess
    parser.add_argument('--srt',    help='SRT file for preprocessing (DJI_0017.SRT)')
    parser.add_argument('--video',  help='[optional] MP4 paired with --srt or --query')

    # navigate
    parser.add_argument('--query',  help='Query SRT file (DJI_0019.SRT)')
    parser.add_argument('--db',     help='Path to geo_db.json (out/geo_db.json)')
    # --video is shared with preprocess; for navigate it applies to the query flight

    # experiment
    parser.add_argument('--srt1',   help='Database SRT file (DJI_0017.SRT)')
    parser.add_argument('--srt2',   help='Query SRT file   (DJI_0019.SRT)')
    parser.add_argument('--video1', help='[optional] MP4 for database flight')
    parser.add_argument('--video2', help='[optional] MP4 for query flight')

    args   = parser.parse_args()
    out_dir = _resolve(args.out_dir) if args.out_dir else OUT_DIR

    if args.mode == 'preprocess':
        if not args.srt:
            parser.error('--srt required for preprocess mode')
        if not os.path.exists(_resolve(args.srt)):
            sys.exit(f"File not found: {args.srt}")
        cmd_preprocess(args, out_dir)

    elif args.mode == 'navigate':
        if not args.query or not args.db:
            parser.error('--query and --db required for navigate mode')
        if not os.path.exists(_resolve(args.query)):
            sys.exit(f"File not found: {args.query}")
        if not os.path.exists(_resolve(args.db)):
            sys.exit(f"Database not found: {args.db}. Run --mode preprocess first.")
        cmd_navigate(args, out_dir)

    elif args.mode == 'experiment':
        if not args.srt1 or not args.srt2:
            parser.error('--srt1 and --srt2 required for experiment mode')
        for p in (args.srt1, args.srt2):
            if not os.path.exists(_resolve(p)):
                sys.exit(f"File not found: {p}")
        cmd_experiment(args, out_dir)


if __name__ == '__main__':
    main()

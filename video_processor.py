"""
video_processor.py — Extract 1fps frames from a DJI MP4 aligned to SRT frame numbers.

DJI records at ~30fps. The SRT parser keeps every 30th frame (frame_cnt = 1, 31, 61, …).
This module seeks to the same frame positions so that each returned image corresponds
exactly to the telemetry entry in the paired SRT file.

Public API
----------
extract_video_frames(video_path, sample_every=30) -> dict[int, np.ndarray]
    Returns {frame_cnt: grayscale_image} for all sampled positions that exist in the video.
"""

import os
import cv2
import numpy as np


def extract_video_frames(video_path: str, sample_every: int = 30) -> dict:
    """
    Extract one frame per second from an MP4, aligned to DJI SRT frame_cnt indices.

    SRT frame_cnt is 1-indexed. The SRT parser retains frames where
    (frame_cnt - 1) % sample_every == 0, i.e. frame_cnt = 1, 31, 61, …
    The corresponding 0-indexed video positions are 0, 30, 60, …

    Parameters
    ----------
    video_path   : path to the MP4 file
    sample_every : frames between samples (default 30, matching SRT parser)

    Returns
    -------
    dict mapping frame_cnt (int) → grayscale uint8 ndarray of shape (H, W).
    Frames that could not be read (seek error, corrupted) are silently omitted;
    geo_database.build_database() will fall back to the synthetic patch for those.
    """
    if not os.path.exists(video_path):
        raise FileNotFoundError(f"Video file not found: {video_path}")

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps_reported = cap.get(cv2.CAP_PROP_FPS)
    width        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height       = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"  Video: {os.path.basename(video_path)}")
    print(f"  Resolution: {width}×{height}  FPS: {fps_reported:.2f}  "
          f"Frames: {total_frames}")

    # Build the list of (frame_cnt, video_index) pairs to extract.
    # frame_cnt is 1-indexed; video frame index is 0-indexed.
    targets = []
    frame_cnt = 1
    while True:
        video_idx = frame_cnt - 1          # 0-indexed position in the video
        if video_idx >= total_frames:
            break
        targets.append((frame_cnt, video_idx))
        frame_cnt += sample_every

    print(f"  Extracting {len(targets)} frames (1 per {sample_every} video frames)...")

    result   = {}
    failures = 0

    for i, (fc, video_idx) in enumerate(targets):
        if (i + 1) % 50 == 0 or i == 0 or i == len(targets) - 1:
            print(f"\r  Reading frame {i+1}/{len(targets)} (video idx {video_idx})...",
                  end='', flush=True)

        cap.set(cv2.CAP_PROP_POS_FRAMES, video_idx)
        ret, frame_bgr = cap.read()

        if not ret or frame_bgr is None:
            failures += 1
            continue

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        result[fc] = gray

    cap.release()
    print()

    print(f"  Extracted: {len(result)} frames  |  Read failures: {failures}")
    return result


def video_fps(video_path: str) -> float:
    """Return the reported FPS of a video file without reading any frames."""
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    return fps

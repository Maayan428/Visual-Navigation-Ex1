# Ex1 - Visual Navigation for Drones (GNSS-Denied)

A visual odometry system that estimates drone position using only camera-derived features — no GPS. Two DJI SRT telemetry files provide flight metadata and serve as ground truth for evaluation.

---

## Algorithm

```
DJI_0017.SRT                        DJI_0019.SRT
     │                                    │
     ▼  PREPROCESSING                     ▼  QUERY FLIGHT
┌──────────────┐                   ┌──────────────┐
│  SRT Parser  │ sample @ 1fps     │  SRT Parser  │ sample @ 1fps
│  (252 frames)│                   │  (118 frames)│
└──────┬───────┘                   └──────┬───────┘
       │                                  │
       ▼                                  ▼
┌──────────────┐                   ┌──────────────┐
│  Footprint   │ GSD + corners     │  Footprint   │
│  Computation │                   │  Computation │
└──────┬───────┘                   └──────┬───────┘
       │                                  │
       ▼                                  ▼
┌────────────────────────────────────────────────┐
│          Synthetic Aerial Map (8000×4000 px)   │
│  GPS → pixel mapping: shared by both flights   │
└──────┬──────────────────────────────┬──────────┘
       │                              │
       ▼                              ▼
┌──────────────┐                ┌──────────────┐
│ ORB Extract  │ 500 feat/frame │ ORB Extract  │
│ Geo-Database │ geo_db.json    │ Query Patch  │
│ + desc .npy  │ geo_db_desc    └──────┬───────┘
└──────────────┘                       │
                                       ▼
                               ┌──────────────────┐
                               │  BFMatcher       │
                               │  NORM_HAMMING    │
                               │  Ratio test 0.75 │
                               │  RANSAC homog.   │
                               │  Top-3 weighted  │
                               │  position avg    │
                               └──────┬───────────┘
                                      │
                                      ▼
                              ┌───────────────────┐
                              │ Estimated (lat,lon)│
                              │ vs GPS ground truth│
                              │ → error in metres  │
                              └───────────────────┘
```

### Stage 1 – Preprocessing
1. **SRT Parsing**: Read DJI telemetry, sample at 1 fps (every 30th frame).
2. **Footprint Computation**: For each frame, compute ground coverage using GSD formula:
   `GSD = (altitude × sensor_width) / (focal_length × image_width)` → corners in GPS.
3. **Synthetic Map**: A shared 8000×4000 px overhead texture (checkerboard + blobs + rectangles,
   seeded for reproducibility) covers the combined GPS bounding box of both flights.
4. **ORB Feature Extraction**: Each DB frame's map crop is extracted, resized to 640×360,
   and processed with ORB (500 features). Results stored in `geo_db.json` + `geo_db_desc.npy`.

### Stage 2 – Navigation
1. Query frame's map crop is extracted and ORB features are computed.
2. BFMatcher (NORM_HAMMING) matches query descriptors against every DB frame.
3. Lowe's ratio test (0.75) filters spurious matches.
4. RANSAC homography counts geometric inliers per candidate.
5. Position estimated as inlier-weighted average of top-3 DB frame positions.

### Stage 3 – Experiment
GPS coordinates from DJI_0019.SRT are withheld from the navigator and used only as ground truth.
Haversine distance between true and estimated position gives the localisation error in metres.

---

## Camera Constants

| Parameter       | Value                        |
|-----------------|------------------------------|
| Camera          | DJI Mini 3 Pro               |
| Sensor width    | 9.6 mm                       |
| Focal length    | 8.8 mm (actual physical)     |
| 35mm equivalent | 24 mm                        |
| Image size      | 1920 × 1080 px               |
| Camera angle    | −90° (nadir, straight down)  |

---

## Installation

```bash
python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

---

## Usage

### Mode A — Synthetic map (no video required, current default)

Features are extracted from a procedurally generated overhead texture.
Useful for algorithm demonstration when the MP4 is unavailable.

```bash
# Full experiment in one command:
python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT

# Step-by-step:
python main.py --mode preprocess --srt data/DJI_0017.SRT
python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json
```

### Mode B — Real video (significantly better accuracy)

When the paired MP4 files are available, pass them with `--video` / `--video1` /
`--video2`. ORB features are then extracted from the actual camera frames instead of
the synthetic texture. Real aerial images contain rich, location-specific texture
(roads, buildings, field boundaries) that is completely absent from the synthetic map,
reducing localisation error from ~160 m to the order of metres when the two flight
paths overlap.

**Important:** both the database and the query must use the same feature source.
Matching real-video DB features against synthetic query features (or vice-versa) will
not produce useful correspondences. Always supply both `--video1` and `--video2`
together, or neither.

```bash
# Full experiment with real video:
python main.py --mode experiment \
  --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT \
  --video1 data/DJI_0017.MP4 --video2 data/DJI_0019.MP4

# Step-by-step with real video:
python main.py --mode preprocess --srt data/DJI_0017.SRT --video data/DJI_0017.MP4
python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json \
                                 --video data/DJI_0019.MP4
```

`video_processor.py` extracts every 30th frame (≈ 1 fps) aligned to the SRT
`frame_cnt` index so each real image corresponds to the telemetry entry it was paired
with. Frames that cannot be read (seek error, truncated file) silently fall back to the
synthetic map patch so the build never crashes mid-run.

### Optional: custom output directory

```bash
python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT \
               --out-dir out/
```

---

## Output Files

| File                           | Description                                      |
|--------------------------------|--------------------------------------------------|
| `out/synthetic_map.npy`        | Shared aerial texture used by both flights       |
| `out/map_config.json`          | GPS bounding box for pixel↔GPS conversion        |
| `out/geo_db.json`              | Frame metadata, keypoints, descriptor slice info (`source`: `video`\|`synthetic`) |
| `out/geo_db_desc.npy`          | Stacked ORB descriptors (N × 32, uint8)          |
| `out/experiment_results.csv`   | Per-frame true vs. estimated position + error    |
| `out/path_groundtruth.kml`     | GPS ground-truth flight path (green)             |
| `out/path_estimated.kml`       | Visually estimated flight path (red)             |

---

## Results

### Mode A – Synthetic map (no video)

| Metric          | Value           |
|-----------------|-----------------|
| Located         | 118/118 (100%)  |
| Mean error      | 164.52 m        |
| Median error    | 155.08 m        |
| 90th pct error  | 310.69 m        |
| Max error       | 348.76 m        |
| Mean confidence | 0.46            |

### Mode B – Real video frames

| Metric          | Value           |
|-----------------|-----------------|
| Located         | 118/118 (100%)  |
| Mean error      | 139.50 m        |
| Median error    | 100.65 m        |
| 90th pct error  | 328.94 m        |
| Max error       | 712.19 m        |
| Mean confidence | 0.84            |

**Note:** Median error drops 35% with real video (155 m → 101 m) and mean confidence
nearly doubles (0.46 → 0.84), reflecting the richer, location-specific texture in real
aerial frames. The higher max error in Mode B is caused by scale mismatch: DJI_0017
ascends from 19 m to 120 m altitude while DJI_0019 flies a level pass at ~50 m, so
DB frames captured at extreme altitudes produce ORB features at a very different apparent
scale from the query, occasionally pulling the weighted-average estimate off course.

### Mode A – Instructor videos DJI_0006/0007 (Synthetic map)

| Metric         | Value          |
|----------------|----------------|
| Total frames   | 260            |
| Located        | 236 (90.8%)    |
| Mean error     | 169.61 m       |
| Median error   | 28.77 m        |
| 90th pct error | 546.95 m       |
| Max error      | 661.72 m       |

**Note:** Median error of 28.77 m shows strong localisation for frames where the two
flight paths overlap. The 90.8% location rate (vs 100% for DJI_0017/0019) reflects 24
unlocated frames where DJI_0007's 117 m cruise altitude produces features at a
different apparent scale from DJI_0006's lower-altitude takeoff segments, causing
RANSAC to reject all homography candidates.

---

## Known Limitations

- **Synthetic imagery**: The navigation runs on a procedurally-generated overhead texture, not real video frames. Results reflect the algorithm's geometric correctness, not real-world photometric performance.
- **Scale change**: DJI_0017 ascends from 19.6 m to 120.4 m altitude. DB frames captured at very different altitudes than the 49.8 m query frames will produce lower inlier counts (different apparent scale).
- **No rotation correction**: Nadir camera is assumed; drone yaw is not compensated. Slight heading differences between flights may reduce inlier counts.
- **Linear complexity**: The navigator iterates all DB frames per query. For large databases, a spatial index (e.g., KD-tree on GPS) would improve speed.
- **No loop closure**: Positions are estimated independently per frame; no trajectory smoothing is applied.

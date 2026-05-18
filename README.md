# Ex1 – Visual Navigation for Drones (GNSS-Denied)

## What This Project Does

When a drone loses GPS signal, it has no way to know where it is. This project solves that problem using only the camera — no GPS, no IMU, no external signals. We pre-process a reference flight to build a visual map of the area, then match features from a new flight against that map to estimate position frame by frame.

## How It Works

**Stage 1 – Learning the area (preprocessing)**
We take a reference flight (DJI_0017), sample one frame per second, and compute what patch of ground each frame covers. We extract up to 500 visual keypoints (ORB features) from each patch and store them in a geo-referenced database alongside their GPS coordinates.

**Stage 2 – Navigating without GPS**
For each frame in the query flight (DJI_0019), we extract the same kind of features and match them against every entry in the database. We use RANSAC to filter out bad matches geometrically, then estimate position as a weighted average of the top 3 matching database frames.

**Stage 3 – Measuring accuracy**
The query flight's GPS coordinates are hidden from the navigator and used only as ground truth. We compute the Haversine distance between the true and estimated position for each frame, then report mean, median, and worst-case error in metres.

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

---

## Quick Start

### Install

```bash
git clone <repo>
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
```

### Run the experiment

**Without video — works out of the box** (features come from a synthetic overhead texture):

```bash
python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT
```

**With real video — better accuracy** (features come from actual camera frames):

```bash
python main.py --mode experiment \
  --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT \
  --video1 data/DJI_0017.MP4 --video2 data/DJI_0019.MP4
```

Always supply both `--video1` and `--video2` together, or neither — mixing real and synthetic features produces no useful matches.

You can also run preprocessing and navigation as separate steps:

```bash
python main.py --mode preprocess --srt data/DJI_0017.SRT [--video data/DJI_0017.MP4]
python main.py --mode navigate   --query data/DJI_0019.SRT --db out/geo_db.json [--video data/DJI_0019.MP4]
```

### Compute where the camera is looking

The drone's GPS position and where the camera actually points are different things — when the gimbal tilts forward, the camera looks tens of metres ahead of the drone. `camera_path.py` computes the exact ground coordinate the camera center ray hits for each frame.

```bash
python3 camera_path.py --srt data/DJI_0006.SRT --pitch -60 --out-dir out/
```

| Flag | Default | Description |
|------|---------|-------------|
| `--srt` | `data/DJI_0017.SRT` | DJI SRT telemetry file |
| `--pitch` | `-60` | Gimbal pitch in DJI convention (0 = horizontal, −90 = straight down) |
| `--out-dir` | `out/` | Where to save KML files |

This writes two KML files you can open in Google Earth side by side:
- `drone_path.kml` — where the drone flew (blue)
- `camera_path.kml` — what the camera was pointed at (orange)

---

## Results

### On our flight data (DJI_0017 vs DJI_0019)

| Metric          | Synthetic map   | Real video      |
|-----------------|-----------------|-----------------|
| Located         | 118/118 (100%)  | 118/118 (100%)  |
| Mean error      | 164.52 m        | 139.50 m        |
| Median error    | 155.08 m        | 100.65 m        |
| 90th pct error  | 310.69 m        | 328.94 m        |
| Max error       | 348.76 m        | 712.19 m        |
| Mean confidence | 0.46            | 0.84            |

Real video cuts median error by 35% and nearly doubles confidence, because actual aerial frames contain location-specific texture (roads, buildings, field edges) that a synthetic checkerboard cannot replicate. The higher max error in real-video mode comes from scale mismatch: the reference flight rises from 19 m to 120 m, so some database frames look very different in scale from the 50 m query frames.

### On instructor flight data (DJI_0006 vs DJI_0007)

| Metric         | Synthetic map  |
|----------------|----------------|
| Total frames   | 260            |
| Located        | 236 (90.8%)    |
| Mean error     | 169.61 m       |
| Median error   | 28.77 m        |
| 90th pct error | 546.95 m       |
| Max error      | 661.72 m       |

The 28 m median shows the system localises well when the two flight paths actually overlap. The 24 unlocated frames occur where DJI_0007 cruises at 117 m — too different in apparent scale from DJI_0006's lower-altitude takeoff for RANSAC to accept any match.

### Best mode — Instructor videos with real video + satellite map

| Metric | Value |
|--------|-------|
| Total frames | 260 |
| Located | 260 (100%) |
| Mean error | 130.33 m |
| Median error | 83.12 m |
| 90th pct error | 316.79 m |
| Max error | 697.57 m |
| Mean confidence | 0.95 |

Real video frames from DJI_0006.MP4 + DJI_0007.MP4 matched against the real satellite map. This is the only configuration that achieves 100% location rate on the instructor data — combining real imagery in the database with real imagery in the query eliminates the feature mismatch that left 24 frames unlocated in synthetic mode. First frame error: 12.6 m at 98% confidence.

---

## Output Files

| File | Description |
|------|-------------|
| `out/synthetic_map.npy` | Shared aerial texture used by both flights |
| `out/map_config.json` | GPS bounding box for pixel ↔ GPS conversion |
| `out/geo_db.json` | Frame metadata, keypoints, descriptor slice info (`source`: `video`\|`synthetic`) |
| `out/geo_db_desc.npy` | Stacked ORB descriptors (N × 32, uint8) |
| `out/experiment_results.csv` | Per-frame true vs. estimated position and error |
| `out/path_groundtruth.kml` | GPS ground-truth flight path (green) |
| `out/path_estimated.kml` | Visually estimated flight path (red) |

---

## Camera Specifications

| Parameter | Value |
|-----------|-------|
| Camera | DJI Mini 3 Pro |
| Sensor width | 9.6 mm |
| Focal length | 8.8 mm (physical) |
| 35mm equivalent | 24 mm |
| Image size | 1920 × 1080 px |
| Camera angle | −90° (nadir, straight down) |

---

## Literature Review

See [literature_review.md](literature_review.md) for a full review of 5 state-of-the-art papers on visual navigation and place recognition, each with open-source implementations, plus a justification of the ORB + BFMatcher design choice.

---

## Known Limitations

- **Synthetic texture**: The default mode runs on a procedurally generated map, not real photos — results show geometric correctness, not real-world performance.
- **Scale mismatch**: A database frame shot at 120 m looks very different from a query frame at 50 m, which hurts match quality at altitude extremes.
- **No yaw correction**: The system assumes the camera points straight down and doesn't compensate for drone heading differences between flights.
- **Linear search**: Every query frame checks every database frame, which gets slow for large databases — a spatial index would fix this.
- **No trajectory smoothing**: Each frame's position is estimated independently; a Kalman filter or similar would reduce noise across frames.

---

## File Structure

| File | What it does |
|------|--------------|
| `srt_parser.py` | Reads DJI SRT telemetry files (supports both format variants) and samples at 1 fps |
| `footprint.py` | Computes the ground footprint (width, height, corners) for each frame using the GSD formula |
| `feature_extractor.py` | Builds the shared synthetic map and extracts ORB features from map patches |
| `geo_database.py` | Builds and loads the geo-referenced feature database (`geo_db.json` + `geo_db_desc.npy`) |
| `navigator.py` | Matches query features against the database using BFMatcher + RANSAC and estimates position |
| `experiment.py` | Runs the full end-to-end experiment and computes localisation error against GPS ground truth |
| `video_processor.py` | Extracts 1 fps frames from an MP4 file, aligned to SRT frame indices |
| `camera_path.py` | Computes the ground coordinate the camera center ray hits for each frame |
| `main.py` | CLI entry point for `preprocess`, `navigate`, and `experiment` modes |

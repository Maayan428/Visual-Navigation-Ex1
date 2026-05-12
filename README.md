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

### Full experiment (recommended)
```bash
python main.py --mode experiment --srt1 data/DJI_0017.SRT --srt2 data/DJI_0019.SRT
```

### Step-by-step

**1. Preprocess** (parse SRT, build synthetic map, extract features, save database):
```bash
python main.py --mode preprocess --srt data/DJI_0017.SRT
```

**2. Navigate** (locate first 5 query frames and print errors):
```bash
python main.py --mode navigate --query data/DJI_0019.SRT --db out/geo_db.json
```

---

## Output Files

| File                           | Description                                      |
|--------------------------------|--------------------------------------------------|
| `out/synthetic_map.npy`        | Shared aerial texture used by both flights       |
| `out/map_config.json`          | GPS bounding box for pixel↔GPS conversion        |
| `out/geo_db.json`              | Frame metadata, keypoints, descriptor slice info |
| `out/geo_db_desc.npy`          | Stacked ORB descriptors (N × 32, uint8)          |
| `out/experiment_results.csv`   | Per-frame true vs. estimated position + error    |
| `out/path_groundtruth.kml`     | GPS ground-truth flight path (green)             |
| `out/path_estimated.kml`       | Visually estimated flight path (red)             |

---

## Results

| Metric         | Value   |
|----------------|---------|
| Total frames   | 118     |
| Located (%)    | 100.0%  |
| Mean error (m) | 164.52  |
| Median error   | 155.08  |
| 90th pct (m)   | 310.69  |
| Max error (m)  | 348.76  |

Early frames (DJI_0019 start position overlaps DJI_0017 start) achieve ~16 m error. Higher errors occur where DJI_0019 traverses areas at the edge of DJI_0017's coverage, causing the navigator to fall back to less-geographically-close database frames.

---

## Known Limitations

- **Synthetic imagery**: The navigation runs on a procedurally-generated overhead texture, not real video frames. Results reflect the algorithm's geometric correctness, not real-world photometric performance.
- **Scale change**: DJI_0017 ascends from 19.6 m to 120.4 m altitude. DB frames captured at very different altitudes than the 49.8 m query frames will produce lower inlier counts (different apparent scale).
- **No rotation correction**: Nadir camera is assumed; drone yaw is not compensated. Slight heading differences between flights may reduce inlier counts.
- **Linear complexity**: The navigator iterates all DB frames per query. For large databases, a spatial index (e.g., KD-tree on GPS) would improve speed.
- **No loop closure**: Positions are estimated independently per frame; no trajectory smoothing is applied.

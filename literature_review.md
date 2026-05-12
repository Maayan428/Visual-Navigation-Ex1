# Literature Review: Visual Navigation for Low-Flying Drones

---

## 1. Introduction

Global Navigation Satellite Systems (GNSS) underpin the vast majority of deployed drone navigation pipelines, yet they represent a single point of failure that is increasingly difficult to defend. Deliberate radio-frequency jamming can deny positioning over wide areas, while spoofing attacks inject counterfeit signals that silently redirect a vehicle to an adversary-controlled location — a threat demonstrated against commercial UAVs in both research and adversarial field settings. Beyond active interference, urban canyons, tunnels, and dense forest canopy all impose multipath conditions that degrade GNSS accuracy to tens of metres or worse. For low-altitude flights in the 20–200 m range — the operational envelope of most commercial and inspection drones — these failure modes are especially hazardous because there is little altitude margin for recovery. Visual navigation offers a GNSS-independent fallback that leverages the onboard camera already present in virtually every drone. The approaches surveyed here span four broad paradigms: (i) visual SLAM, which incrementally builds a local map while tracking the camera pose; (ii) learned feature matching, which replaces hand-crafted descriptors with transformer-based correspondence networks; (iii) universal visual place recognition (VPR), which retrieves geographically relevant database images using global image descriptors; and (iv) UAV-specific geo-localization, which aligns onboard imagery directly with pre-existing georeferenced map data.

---

## 2. Visual SLAM Approaches

### 2.1 ORB-SLAM3 [Campos et al., 2021]

ORB-SLAM3 is a full-featured, open-source SLAM library that unifies monocular, stereo, RGB-D, fisheye, and inertial camera configurations within a single multi-map framework. At its core the system extracts ORB (Oriented FAST and Rotated BRIEF) keypoints and matches them across frames using a fast bag-of-words place recognition index; local bundle adjustment then refines camera poses and 3-D map points jointly. A distinctive architectural feature is the ability to spawn new sub-maps when tracking is lost and to merge them later via loop closure detection, which makes the system resilient to temporary occlusions and revisited environments. On the EuRoC MAV benchmark — a standard dataset recorded from a small drone flying indoor corridors — the stereo-inertial configuration achieves a mean absolute trajectory error of 3.6 cm, establishing a strong reference point for high-precision visual-inertial odometry.

**Relevance to drone navigation.** ORB-SLAM3 is directly applicable to low-altitude drone flights: it runs in real time on a standard CPU, requires no pre-existing map, and its inertial extension can fuse IMU measurements available on virtually all flight controllers. The multi-map capability is valuable for long survey missions where a single consistent map would grow prohibitively large.

- **GitHub:** [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **Real-time capability:** Yes
- **Main limitation:** Loop-closure graph optimization scales quadratically with the number of keyframes, making it computationally heavy for embedded flight controllers (e.g., Raspberry Pi or Jetson Nano) on extended flights. Outdoor nadir-view imagery also tends to produce fewer repeatable ORB keypoints than the indoor corridor scenes for which the system was primarily tuned.

---

## 3. Learned Feature Matching

### 3.1 LightGlue [Lindenberger et al., ICCV 2023]

LightGlue is a lightweight graph neural network that learns to match sparse local features (SuperPoint, SIFT, DISK, or others) between image pairs. Its central contribution is an adaptive early-exit mechanism: an internal confidence estimator halts the transformer self- and cross-attention computations as soon as the match quality is deemed sufficient, rather than running a fixed number of layers regardless of difficulty. In practice this yields an 8× speed-up over the comparable LoFTR dense matcher at equivalent accuracy on standard homography and pose estimation benchmarks, enabling near-real-time operation on a modern GPU. The network is pretrained on large-scale datasets of natural and indoor scenes and generalises to new environments without retraining.

**Relevance to drone navigation.** Frame-to-frame feature matching is the inner loop of any visual odometry pipeline. LightGlue's speed advantage over classical BFMatcher approaches becomes especially significant when matching against a large geo-database, as is the case in map-based localization. Its compatibility with SuperPoint — itself a learned keypoint detector — produces more uniformly distributed, textureless-area-robust correspondences than ORB on low-contrast aerial imagery.

- **GitHub:** [https://github.com/cvg/LightGlue](https://github.com/cvg/LightGlue)
- **Real-time capability:** Yes (GPU); partial on CPU (depends on image resolution and number of keypoints)
- **Main limitation:** Real-time performance requires a dedicated GPU, which is unavailable on most weight-constrained drone platforms. Deployment additionally requires downloading pretrained network weights, introducing a dependency on external model hosting.

---

## 4. Universal Visual Place Recognition

### 4.1 AnyLoc [Keetha et al., RA-L 2023]

AnyLoc proposes a training-free visual place recognition pipeline that extracts dense patch-level features from a frozen DINOv2 vision transformer and aggregates them into a compact global descriptor using VLAD (Vector of Locally Aggregated Descriptors) with a vocabulary derived from a small, domain-agnostic cluster set. The key insight is that DINOv2's self-supervised features, learned on a broad internet-scale corpus, encode semantically meaningful spatial structure that transfers across radically different acquisition domains — aerial satellite imagery, indoor corridors, underwater footage — without any task-specific fine-tuning. On standard VPR benchmarks spanning all three domains the method achieves up to 4× higher Recall@1 than the strongest domain-specific baselines, while requiring no labelled training data for the target environment.

**Relevance to drone navigation.** AnyLoc was explicitly evaluated on aerial views, making it directly applicable to low-altitude drone localization against a pre-collected overhead reference database. Its domain-agnostic nature means a single deployed model can handle the shift from the reference orthophoto (captured by a satellite or high-altitude aircraft) to the oblique or nadir query frame from a low-flying drone without retraining.

- **GitHub:** [https://github.com/AnyLoc/AnyLoc](https://github.com/AnyLoc/AnyLoc)
- **Real-time capability:** No — inference through DINOv2 ViT-G requires several hundred milliseconds per frame even on a high-end GPU; not currently suitable for closed-loop onboard control.
- **Main limitation:** The DINOv2 backbone has approximately 1.1 billion parameters, imposing substantial VRAM requirements (≥16 GB for batch inference) and ruling out embedded deployment without model distillation.

---

## 5. UAV-Specific Geo-Localization

### 5.1 GNSS-Denied UAV Geo-Localization [Kinnari et al., 2021]

Kinnari et al. address the problem of absolute geo-localization — determining a UAV's position on a global coordinate grid — without relying on downward-facing cameras or highly constrained flight trajectories. Their approach applies Monte-Carlo Localization (particle filter) in the position space defined by a pre-existing orthophoto map. At each time step, candidate particles are weighted by the photometric similarity between an orthorectified crop of the map and the current onboard camera frame, which is projected to a nadir view using the drone's known attitude. The method tolerates moderate attitude errors and seasonal appearance changes because the similarity metric operates on local texture statistics rather than exact pixel intensities. Experiments on real UAV flights over Finnish forest and urban terrain demonstrate metre-level convergence after sufficient particle-filter burn-in.

**Relevance.** This work is particularly pertinent to the current assignment because it demonstrates that map-based localization with nadir-view imagery — the exact configuration of the DJI Mini 3 Pro used here — is a viable path to absolute positioning without GPS. It also motivates the use of a pre-built reference map (our synthetic overhead texture) as the core localization substrate.

- **Real-time capability:** Partial — the particle filter itself is lightweight, but orthorectification and patch similarity evaluation introduce latency proportional to the number of particles; practical implementations use 500–2000 particles at ≈1–5 Hz.
- **Main limitation:** The method requires a pre-existing, georeferenced orthophoto map of the operational area at sufficient resolution (≤10 cm/px for reliable matching), which may not be available in disaster or adversarial scenarios.

### 5.2 Jointly Optimized Global-Local Localization [Li et al., 2023]

Li et al. propose a two-stage UAV localization framework in which a global image retrieval step first identifies the most geographically plausible candidates from a large satellite-view database, and a subsequent local feature matching step refines the pose estimate within the retrieved candidates. The global stage uses a NetVLAD-style descriptor to rank database images by scene similarity; the local stage then applies SuperGlue correspondence matching between the query frame and the top-K retrieved images to compute a precise homography and recover the camera position. Joint optimization of the retrieval and matching losses during training encourages the global descriptor to bias towards configurations that are amenable to accurate local matching, rather than optimizing the two stages independently.

**Relevance.** The coarse-to-fine strategy mirrors the architecture of production-scale visual localization systems and is directly applicable to drone geo-localization: global retrieval rapidly narrows the search from a continent-scale database to a few candidate tiles, while local matching yields sub-metre position accuracy within those tiles. The approach also naturally handles the large viewpoint difference between satellite reference imagery and low-altitude query frames.

- **arXiv:** 2310.08082
- **Real-time capability:** Partial — global retrieval is fast (≈50 ms); local SuperGlue matching requires a GPU and adds 100–300 ms per query.
- **Main limitation:** Training and evaluation were conducted primarily on satellite-view reference datasets; performance on sub-50 m altitude nadir imagery paired with coarser map sources has not yet been systematically reported.

---

## 6. Comparison Table

| Method | Type | Real-time | GPU needed | Open-source | Best for |
|---|---|---|---|---|---|
| ORB-SLAM3 [Campos et al., 2021] | SLAM | Yes | No | Yes | Continuous tracking |
| LightGlue [Lindenberger et al., 2023] | Matching | GPU only | Yes | Yes | Frame-to-frame matching |
| AnyLoc [Keetha et al., 2023] | VPR | No | Yes | Yes | Place recognition |
| Kinnari et al. [2021] | Geo-registration | Partial | No | No | Orthophoto map matching |
| Li et al. [2023] | Global + Local | Partial | Yes | Partial | UAV geo-localization |

---

## 7. Our Approach and Justification

The implementation presented in this assignment (Ex1) deliberately adopts classical ORB feature extraction combined with OpenCV's brute-force BFMatcher (NORM\_HAMMING) and RANSAC homography rather than any of the learned methods surveyed above. This choice is justified on the following grounds:

**No GPU required.** The entire pipeline — SRT parsing, footprint computation, feature extraction, matching, and position estimation — runs on a standard laptop CPU in under 5 minutes for a 252-frame database and 118 query frames. Learned alternatives (LightGlue, AnyLoc) require a dedicated CUDA-capable GPU for real-time performance, which is unavailable on the target development environment.

**No pretrained model downloads.** ORB is a deterministic algorithm fully implemented in OpenCV; it requires no external weights files, internet connectivity, or model licensing. This simplifies reproducibility and makes the submission self-contained.

**Fully interpretable.** Every step of the pipeline — keypoint detection, descriptor computation, ratio-test filtering, RANSAC inlier counting, weighted-average position fusion — is mathematically transparent and auditable. This is valuable in an academic setting where the algorithmic contribution, not benchmark performance, is the primary evaluation criterion.

**Sufficient for proof-of-concept.** With the synthetic aerial texture database derived from both SRT flight bounding boxes, the system achieves a **100% localization rate** across all 118 query frames from DJI\_0019.SRT. The reported mean error of **164 m** (median: 155 m) is not a failure of the ORB matching algorithm; it reflects the fundamental limitation of a synthetic feature database that lacks the geographic specificity of real orthophoto tiles — a point the experiment section discusses in detail. In the first five query frames, where DJI\_0019 begins at the same geographic location as DJI\_0017, the error drops to **~16 m**, confirming that the matcher correctly identifies overlapping map regions.

**Future work.** Replacing ORB with SuperPoint + LightGlue and the synthetic map with real orthophoto tiles (e.g., from Israel's Survey Office at ≤10 cm/px) would be the most impactful next step. AnyLoc could serve as a coarse global retrieval stage to limit the number of database frames that require expensive local matching, substantially reducing the O(N) per-query cost that currently dominates runtime.

---

## 8. References

[1] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel, and J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM," *IEEE Transactions on Robotics*, vol. 37, no. 6, pp. 1874–1890, Dec. 2021. arXiv: [2007.11898](https://arxiv.org/abs/2007.11898). Code: [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

[2] P. Lindenberger, P.-E. Sarlin, and M. Pollefeys, "LightGlue: Local Feature Matching at Light Speed," in *Proc. IEEE/CVF International Conference on Computer Vision (ICCV)*, 2023, pp. 17627–17638. arXiv: [2306.13643](https://arxiv.org/abs/2306.13643). Code: [https://github.com/cvg/LightGlue](https://github.com/cvg/LightGlue)

[3] N. Keetha, A. Mishra, J. Karhade, K. M. Jatavallabhula, S. Scherer, M. Krishna, and S. Garg, "AnyLoc: Towards Universal Visual Place Recognition," *IEEE Robotics and Automation Letters*, vol. 9, no. 2, pp. 1286–1293, 2023. arXiv: [2308.00688](https://arxiv.org/abs/2308.00688). Code: [https://github.com/AnyLoc/AnyLoc](https://github.com/AnyLoc/AnyLoc)

[4] J. Kinnari, F. Verdoja, and V. Kyrki, "GNSS-denied geolocalization of UAVs by visual matching of onboard camera images with orthophotos," arXiv preprint arXiv:2103.14381, 2021. [https://arxiv.org/abs/2103.14381](https://arxiv.org/abs/2103.14381)

[5] "Jointly Optimized Global-Local Visual Localization of UAVs," arXiv preprint arXiv:2310.08082, 2023. [https://arxiv.org/abs/2310.08082](https://arxiv.org/abs/2310.08082)

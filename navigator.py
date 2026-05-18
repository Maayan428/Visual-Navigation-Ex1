import cv2
import numpy as np


class Navigator:
    """
    Estimates drone position from visual features using a pre-built geo-database.

    Algorithm:
      1. Match query ORB descriptors against every database frame using BFMatcher + NORM_HAMMING.
      2. Filter matches with Lowe's ratio test (threshold 0.75).
      3. Run RANSAC homography on each candidate to count geometric inliers.
      4. Weighted-average position from top-3 matches (weight = inlier count).
    """

    RATIO       = 0.75
    RANSAC_THRESH = 5.0
    TOP_K       = 3
    MIN_MATCHES = 4   # minimum good matches needed to attempt homography

    def __init__(self, records: list, stacked_desc: np.ndarray):
        self.records = records
        self.stacked = stacked_desc  # shape (total_kp, 32) uint8
        self.bf      = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def locate(self, query_kp_ser: list, query_des: np.ndarray) -> dict | None:
        """Match query descriptors against the database and return a weighted-average position estimate.
        Returns dict with est_lat, est_lon, confidence, total_inliers, top_matches; or None on failure."""
        if query_des is None or len(query_des) == 0:
            return None

        results = []  # (frame_idx, inlier_count, matched_count)

        for i, record in enumerate(self.records):
            idx0   = record['desc_index']
            count  = record['desc_count']
            db_des = self.stacked[idx0: idx0 + count]

            if db_des.shape[0] < 2:
                continue

            try:
                raw_matches = self.bf.knnMatch(query_des, db_des, k=2)
            except cv2.error:
                continue

            # Lowe's ratio test
            good = []
            for m_pair in raw_matches:
                if len(m_pair) < 2:
                    continue
                m, n = m_pair[0], m_pair[1]
                if m.distance < self.RATIO * n.distance:
                    good.append(m)

            if len(good) < self.MIN_MATCHES:
                continue

            # Build point arrays for homography
            src_pts = np.float32(
                [query_kp_ser[m.queryIdx][:2] for m in good]
            ).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [record['keypoints'][m.trainIdx][:2] for m in good]
            ).reshape(-1, 1, 2)

            _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, self.RANSAC_THRESH)
            inliers = int(mask.sum()) if mask is not None else 0

            results.append((i, inliers, len(good)))

        if not results:
            return None

        # Sort by inlier count, take top-K
        results.sort(key=lambda x: x[1], reverse=True)
        top = results[:self.TOP_K]

        total_inliers = sum(r[1] for r in top)
        total_matched = sum(r[2] for r in top)

        if total_inliers == 0:
            return None

        est_lat = sum(self.records[r[0]]['lat'] * r[1] for r in top) / total_inliers
        est_lon = sum(self.records[r[0]]['lon'] * r[1] for r in top) / total_inliers
        confidence = total_inliers / total_matched if total_matched > 0 else 0.0

        return {
            'est_lat':       est_lat,
            'est_lon':       est_lon,
            'confidence':    confidence,
            'total_inliers': total_inliers,
            'top_matches': [
                {
                    'frame_id': self.records[r[0]]['frame_id'],
                    'lat':      self.records[r[0]]['lat'],
                    'lon':      self.records[r[0]]['lon'],
                    'inliers':  r[1],
                    'matched':  r[2],
                }
                for r in top
            ],
        }

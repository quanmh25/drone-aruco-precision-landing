"""ArUco pose estimation and measurement evaluation."""

import csv
import os

import cv2
import numpy as np
import yaml

from landing_control.geometry import marker_in_odom


def evaluate_detections(detections, stamp, pose, orientation, camera_xyz, camera_rpy,
                        max_reprojection_px, evaluator):
    """Filter poses, transform to odom, and record errors without using truth for pose."""
    for detection in detections:
        if detection['reprojection_error_px'] > max_reprojection_px:
            continue
        world = marker_in_odom(
            detection['tvec'], pose, orientation, camera_xyz, camera_rpy,
        )
        absolute_error = evaluator.record(stamp, detection, world)
        yield detection, world, absolute_error


class ArucoDetector:
    def __init__(self, marker_sizes, dictionary_id=cv2.aruco.DICT_4X4_250):
        self.marker_sizes = {int(mid): float(size) for mid, size in marker_sizes.items()}
        if not self.marker_sizes:
            raise ValueError('Configure marker sizes in ground_truth_yaml')
        if any(not np.isfinite(size) or size <= 0 for size in self.marker_sizes.values()):
            raise ValueError('Marker sizes must be finite and positive')

        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
            self.legacy = None
        else:
            self.detector = None
            self.legacy = (dictionary, cv2.aruco.DetectorParameters_create())

    def estimate(self, bgr, K, D):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(bgr)
        else:
            dictionary, params = self.legacy
            corners, ids, _ = cv2.aruco.detectMarkers(bgr, dictionary, parameters=params)

        if ids is None:
            return []

        K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        D = np.asarray(D, dtype=np.float64)
        output = []

        for pts, mid in zip(corners, ids.reshape(-1)):
            marker_id = int(mid)
            if marker_id not in self.marker_sizes:
                continue

            L = self.marker_sizes[marker_id]
            obj = np.array([[-L/2, L/2, 0], [L/2, L/2, 0], [L/2, -L/2, 0], [-L/2, -L/2, 0]], dtype=np.float64)
            img = np.asarray(pts, dtype=np.float64).reshape(4, 2)

            ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)) or tvec[2, 0] <= 0:
                continue

            projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            reproj = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - img) ** 2, axis=1))))

            output.append({
                'marker_id': marker_id,
                'tvec': tvec.reshape(3),
                'rvec': rvec.reshape(3),
                'reprojection_error_px': reproj,
                'corners': img
            })

        return output


class Evaluator:
    def __init__(self, truth_yaml, output_csv):
        self.truth = {}
        self.marker_sizes = {}

        if truth_yaml:
            with open(truth_yaml, encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}

            for marker_id, data in cfg.get('markers', {}).items():
                marker_id = int(marker_id)
                xyz = data['ground_truth'] if isinstance(data, dict) else data
                if len(xyz) != 3:
                    raise ValueError(f'Ground truth marker {marker_id} must contain xyz')
                self.truth[marker_id] = np.asarray(xyz, dtype=float)
                if isinstance(data, dict) and 'size' in data:
                    self.marker_sizes[marker_id] = float(data['size'])

        os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
        self.f = open(output_csv, 'w', newline='', encoding='utf-8', buffering=1)
        self.writer = csv.writer(self.f)
        self.writer.writerow(['time_sec', 'id', 'cx', 'cy', 'cz', 'range_m', 'wx', 'wy', 'wz',
                              'gt_x', 'gt_y', 'gt_z', 'error_x', 'error_y', 'error_z',
                              'absolute_error_m', 'reprojection_px'])


    def record(self, stamp, detection, world_xyz):
        world_xyz = np.asarray(world_xyz, dtype=float)
        truth = self.truth.get(detection['marker_id'])

        if truth is not None:
            diff = world_xyz - truth
            error_xyz = np.abs(diff)
            absolute_error = float(np.linalg.norm(diff))
        else:
            error_xyz = [float('nan')] * 3
            absolute_error = float('nan')

        gt = truth if truth is not None else [float('nan')] * 3
        self.writer.writerow([
            f'{stamp:.3f}', detection['marker_id'],
            *[f'{x:.6f}' for x in detection['tvec']],
            f"{np.linalg.norm(detection['tvec']):.6f}",
            *[f'{x:.6f}' for x in world_xyz],
            *[f'{x:.6f}' for x in gt],
            *[f'{x:.6f}' for x in error_xyz],
            f'{absolute_error:.6f}',
            f"{detection['reprojection_error_px']:.4f}"
        ])

        return absolute_error


    def close(self):
        self.f.close()

"""Find an ArUco marker in a camera image: where its centre is relative to the camera.

The pick world's marker (``models/pick_marker``: DICT_4X4_50, id 0, a 4.4 cm square)
lies flat beside the part; the wrist camera on the OpenManipulator-X sees it. Works
with OpenCV 4.6 (the system's, which the ROS nodes use) and 4.7+ (ArucoDetector).
"""

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
MARKER_SIZE = 0.044


def _detect(gray, dictionary, min_distance_rate=None):
    new_api = hasattr(cv2.aruco, "ArucoDetector")
    params = cv2.aruco.DetectorParameters() if new_api else cv2.aruco.DetectorParameters_create()
    if min_distance_rate is not None:
        params.minMarkerDistanceRate = min_distance_rate
    if new_api:
        return cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)[:2]
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def marker_image(size_px, marker_id=MARKER_ID):
    """The marker's black square as a ``size_px`` image (for tests and textures)."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, size_px)


def locate(gray, K, marker_id=MARKER_ID, size=MARKER_SIZE, min_range=0.05, max_range=0.4):
    """Centre of the marker in the camera's optical frame (x right, y down, z forward), m.

    ``K`` is the 3 x 3 camera matrix (no distortion). None when the marker is not seen,
    or seen outside ``min_range``-``max_range`` along the camera's axis (a wrong square).
    """
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    corners, ids = _detect(gray, dictionary)
    if ids is None:
        # The white card round the square has its own outline a few pixels outside the
        # square's; at the default 0.05 the detector can drop the square as that
        # outline's near-duplicate. Nearer candidates allowed, it finds it.
        corners, ids = _detect(gray, dictionary, min_distance_rate=0.01)
    if ids is None:
        return None
    # Nearer candidates allowed, a smaller square inside the marker can decode as it too:
    # take the largest.
    found = [c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if i == marker_id]
    if not found:
        return None
    c = max(found, key=lambda q: cv2.arcLength(q.astype(np.float32), True))
    h = size / 2.0
    square = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    ok, _rvec, tvec = cv2.solvePnP(square, c.astype(np.float64), np.asarray(K, dtype=np.float64),
                                   np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not min_range < tvec[2, 0] < max_range:
        return None
    return tvec.ravel()


def optical_to_sensor(p):
    """A point from the optical frame to Gazebo's camera frame (x forward, y left, z up)."""
    return np.array([p[2], -p[0], -p[1]])

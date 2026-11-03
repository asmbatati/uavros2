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


def _detect(gray, dictionary):
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers(gray)[:2]
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())[:2]


def marker_image(size_px, marker_id=MARKER_ID):
    """The marker's black square as a ``size_px`` image (for tests and textures)."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, size_px)


def locate(gray, K, marker_id=MARKER_ID, size=MARKER_SIZE):
    """Centre of the marker in the camera's optical frame (x right, y down, z forward), m.

    ``K`` is the 3 x 3 camera matrix (no distortion). None when the marker is not seen.
    """
    corners, ids = _detect(gray, cv2.aruco.getPredefinedDictionary(DICTIONARY))
    if ids is None:
        return None
    for c, i in zip(corners, ids.ravel()):
        if i != marker_id:
            continue
        h = size / 2.0
        square = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
        ok, _rvec, tvec = cv2.solvePnP(square, c.reshape(4, 2).astype(np.float64),
                                       np.asarray(K, dtype=np.float64), np.zeros(5),
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return tvec.ravel() if ok else None
    return None


def optical_to_sensor(p):
    """A point from the optical frame to Gazebo's camera frame (x forward, y left, z up)."""
    return np.array([p[2], -p[0], -p[1]])

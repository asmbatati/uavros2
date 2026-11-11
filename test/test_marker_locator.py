"""The wrist camera's marker locator (uavros2.marker_locator) on a synthetic image."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from uavros2.marker_locator import locate, marker_image, MARKER_SIZE, optical_to_sensor  # noqa: E402

K = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 240.0], [0.0, 0.0, 1.0]])


def _render(centre):
    """A white image with the marker flat in front of the camera, centred at ``centre`` (optical)."""
    h = MARKER_SIZE / 2.0
    world = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) + centre[:2]
    image_pts = np.array([[K[0, 0] * x / centre[2] + K[0, 2], K[1, 1] * y / centre[2] + K[1, 2]]
                          for x, y in world], dtype=np.float32)
    tag = marker_image(240)
    src = np.array([[0, 0], [240, 0], [240, 240], [0, 240]], dtype=np.float32)
    warp = cv2.warpPerspective(tag, cv2.getPerspectiveTransform(src, image_pts), (640, 480),
                               borderValue=255)
    # The detector needs the white quiet zone round the black square.
    return warp


@pytest.mark.parametrize("centre", [(0.0, 0.0, 0.15), (0.03, -0.02, 0.12), (-0.05, 0.04, 0.25)])
def test_locates_the_marker_centre(centre):
    found = locate(_render(np.array(centre)), K)
    assert found is not None
    # Sideways within 2 mm; along the axis 2 % of the range (the corners' pixels).
    np.testing.assert_allclose(found[:2], centre[:2], atol=2e-3)
    assert found[2] == pytest.approx(centre[2], rel=0.02)


def test_no_marker_no_answer():
    assert locate(np.full((480, 640), 255, dtype=np.uint8), K) is None


def test_optical_to_gazebo_camera_frame():
    np.testing.assert_allclose(optical_to_sensor((0.1, 0.2, 0.3)), (0.3, -0.1, -0.2))


def test_finds_the_marker_in_a_wrist_camera_frame():
    """A frame from the simulated wrist camera: the marker on its white card, on the grey post.

    The card's outline lies a few pixels outside the square's; at OpenCV's default
    minimum distance between candidates the square was dropped and nothing found.
    """
    from pathlib import Path

    image = cv2.imread(str(Path(__file__).parent / "data" / "wrist_camera_marker.png"), cv2.IMREAD_GRAYSCALE)
    if image is None:
        pytest.skip("test/data/wrist_camera_marker.png not fetched (git lfs pull)")
    fx = 320.0 / np.tan(0.75)   # the camera's 1.5 rad field of view at 640 px
    found = locate(image, np.array([[fx, 0.0, 320.0], [0.0, fx, 240.0], [0.0, 0.0, 1.0]]))
    assert found is not None
    assert 0.10 < found[2] < 0.25   # about 15 cm below the camera

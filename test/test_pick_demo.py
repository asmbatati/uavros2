"""Pick-and-place metrics (uavros2.pick_demo)."""

import math
from types import SimpleNamespace

import pytest

from uavros2.pick_demo import part_tilt, pick_metrics

POST_TOP = 1.0
REST_Z = 1.025


def _row(phase, part, ee=None, tilt=0.0):
    row = {"phase": phase, "part_x": part[0], "part_y": part[1], "part_z": part[2],
           "part_tilt": tilt}
    if ee is not None:
        row["true_ee_x"], row["true_ee_y"], row["true_ee_z"] = ee
    return row


def _flight(final, carried_z=1.12):
    return [
        _row("over_part", (1.5, 0.0, REST_Z)),
        _row("close", (1.5, 0.0, REST_Z), ee=(1.51, -0.004, 1.025)),
        _row("lift", (1.51, 0.0, 1.08)),
        _row("to_place", (1.5, 0.5, carried_z)),
        _row("over_place", (1.5, 1.0, carried_z)),
        _row("release", final),
    ]


def test_a_part_carried_and_set_down_on_the_second_post_is_placed():
    m = pick_metrics(_flight((1.52, 1.01, REST_Z)), POST_TOP, (1.5, 1.0))
    assert m["grasp_offset_xy_m"] == pytest.approx([0.01, -0.004])
    assert m["grasp_offset_z_m"] == pytest.approx(0.005)
    assert m["lifted"] and m["carried"] and m["placed"]
    assert m["place_error_m"] == pytest.approx(math.hypot(0.02, 0.01))


def test_a_dropped_part_is_neither_carried_nor_placed():
    m = pick_metrics(_flight((1.5, 0.6, 0.02), carried_z=0.02), POST_TOP, (1.5, 1.0))
    assert m["lifted"] and not m["carried"] and not m["placed"]


def test_a_part_left_lying_on_the_post_is_not_placed():
    m = pick_metrics(_flight((1.5, 1.0, 1.02), carried_z=1.12)[:-1]
                     + [_row("release", (1.5, 1.0, 1.02), tilt=math.radians(90))], POST_TOP, (1.5, 1.0))
    assert not m["placed"] and m["final_tilt_deg"] == pytest.approx(90.0)


def test_part_tilt():
    s = math.sin(math.radians(15) / 2)
    assert math.degrees(part_tilt(SimpleNamespace(x=s, y=0.0))) == pytest.approx(15.0)


def test_the_grasp_offset_is_the_last_attempts():
    rows = _flight((1.5, 1.0, REST_Z))
    retry = [_row("close", (1.5, 0.0, REST_Z), ee=(1.53, 0.04, 1.05)),
             _row("close", (1.5, 0.0, REST_Z), ee=(1.53, 0.04, 1.05)),
             _row("retry", (1.5, 0.0, REST_Z))]
    m = pick_metrics(rows[:1] + retry + rows[1:], POST_TOP, (1.5, 1.0))
    assert m["grasp_offset_xy_m"] == pytest.approx([0.01, -0.004])

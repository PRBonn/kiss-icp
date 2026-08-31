# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import numpy as np
import pytest

from kiss_icp.preprocess import Preprocessor

# Points far enough from the origin to survive the range filter before and after deskewing
FRAME = np.array([[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0], [5.0, 5.0, 5.0]])

# Constant velocity guess used to deskew: 1 meter forward, no rotation
RELATIVE_MOTION = np.array(
    [
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


# Smallest positive subnormal double, the finest time range representable next to zero
DENORMAL_MIN = np.nextafter(0.0, np.inf)


def _deskewing_preprocessor() -> Preprocessor:
    return Preprocessor(max_range=100.0, min_range=0.0, deskew=True, max_num_threads=1)


def _advanced_by_ulps(value: float, ulps: int) -> float:
    for _ in range(ulps):
        value = np.nextafter(value, np.inf)
    return value


@pytest.mark.parametrize("timestamp", [0.0, 1.0, 1.7e9])
def test_deskew_is_a_noop_when_all_timestamps_are_equal(timestamp):
    """Scans where every point shares one timestamp must not normalize by a zero time range."""
    timestamps = np.full(len(FRAME), timestamp)

    frame = _deskewing_preprocessor().preprocess(FRAME, timestamps, RELATIVE_MOTION)

    np.testing.assert_allclose(frame, FRAME)


@pytest.mark.parametrize("ulps", [1, 2, 4])
@pytest.mark.parametrize("timestamp", [1.0, 1.7e9])
def test_deskew_is_a_noop_when_timestamps_differ_by_a_few_ulps(timestamp, ulps):
    """A time range at the floating point resolution of the timestamps is representation noise.

    Normalization stretches it across the whole [0, 1] interval, so without headroom in the
    tolerance a spread of a couple of ULPs applies the full motion correction.
    """
    timestamps = np.full(len(FRAME), timestamp)
    timestamps[-1] = _advanced_by_ulps(timestamp, ulps)

    frame = _deskewing_preprocessor().preprocess(FRAME, timestamps, RELATIVE_MOTION)

    np.testing.assert_allclose(frame, FRAME)


@pytest.mark.parametrize("subnormals", [1, 8])
def test_deskew_is_a_noop_for_a_subnormal_time_range_at_the_origin(subnormals):
    """Next to zero a purely relative tolerance underflows to zero and stops guarding anything."""
    timestamps = np.zeros(len(FRAME))
    timestamps[-1] = subnormals * DENORMAL_MIN

    frame = _deskewing_preprocessor().preprocess(FRAME, timestamps, RELATIVE_MOTION)

    np.testing.assert_allclose(frame, FRAME)


def test_deskew_is_a_noop_for_a_single_point_scan():
    frame = _deskewing_preprocessor().preprocess(FRAME[:1], np.array([1.7e9]), RELATIVE_MOTION)

    np.testing.assert_allclose(frame, FRAME[:1])


def test_deskew_still_compensates_motion_for_distinct_timestamps():
    timestamps = np.linspace(1.7e9, 1.7e9 + 0.1, len(FRAME))

    frame = _deskewing_preprocessor().preprocess(FRAME, timestamps, RELATIVE_MOTION)

    assert np.isfinite(frame).all()
    assert not np.allclose(frame, FRAME)

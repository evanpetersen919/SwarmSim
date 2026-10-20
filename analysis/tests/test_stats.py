import math

import pytest

from swarmsim_analysis import summarize

# Student t critical values for a two-sided 95 percent interval, from a t table: 4 degrees of
# freedom 2.776445, 1 degree of freedom 12.706205, 2 degrees of freedom 4.302653.
T_DF1 = 12.706205
T_DF2 = 4.302653
T_DF4 = 2.776445


def test_known_sample_gives_the_hand_computed_interval():
    s = summarize([1.0, 2.0, 3.0, 4.0, 5.0])
    assert s.n == 5
    assert s.missing == 0
    assert s.mean == 3.0
    assert s.std == pytest.approx(math.sqrt(2.5), rel=1e-12)
    half = T_DF4 * math.sqrt(2.5) / math.sqrt(5)
    assert s.ci_low == pytest.approx(3.0 - half, abs=1e-5)
    assert s.ci_high == pytest.approx(3.0 + half, abs=1e-5)
    assert s.median == 3.0
    assert (s.minimum, s.maximum) == (1.0, 5.0)


def test_two_values_have_a_very_wide_interval():
    s = summarize([0.0, 2.0])
    assert s.mean == 1.0
    assert s.std == pytest.approx(math.sqrt(2.0), rel=1e-12)
    assert s.ci_high - s.mean == pytest.approx(T_DF1, abs=1e-5)  # sqrt(2) / sqrt(2) cancels
    assert s.median == 1.0


def test_three_values():
    s = summarize([10.0, 12.0, 14.0])
    assert s.mean == 12.0
    assert s.std == 2.0
    assert s.ci_high - s.mean == pytest.approx(T_DF2 * 2.0 / math.sqrt(3), abs=1e-5)


def test_the_interval_is_symmetric_about_the_mean_and_order_does_not_matter():
    a = summarize([5.0, 1.0, 4.0, 2.0, 3.0])
    b = summarize([1.0, 2.0, 3.0, 4.0, 5.0])
    assert a == b
    assert a.mean - a.ci_low == pytest.approx(a.ci_high - a.mean, rel=1e-12)


def test_a_constant_sample_has_a_zero_width_interval():
    s = summarize([7.5, 7.5, 7.5, 7.5])
    assert s.mean == 7.5
    assert s.std == 0.0
    assert s.ci_low == s.ci_high == 7.5


def test_one_value_has_no_interval_and_is_not_reported_as_zero_width():
    s = summarize([4.0])
    assert s.n == 1
    assert s.mean == 4.0
    assert math.isnan(s.std)
    assert math.isnan(s.ci_low) and math.isnan(s.ci_high)
    assert s.median == 4.0


def test_no_values_gives_nan_not_zero():
    s = summarize([])
    assert s.n == 0
    assert all(math.isnan(v) for v in (s.mean, s.std, s.ci_low, s.ci_high, s.median))


def test_nan_entries_are_counted_as_missing_and_left_out():
    s = summarize([1.0, math.nan, 3.0, math.nan])
    assert s.n == 2
    assert s.missing == 2
    assert s.mean == 2.0
    only_missing = summarize([math.nan, math.nan])
    assert only_missing.n == 0 and only_missing.missing == 2


def test_the_median_of_an_even_count_is_the_mean_of_the_middle_two():
    assert summarize([1.0, 2.0, 10.0, 20.0]).median == 6.0


def test_a_different_confidence_level_changes_the_width():
    wide = summarize([1.0, 2.0, 3.0, 4.0, 5.0], confidence=0.99)
    narrow = summarize([1.0, 2.0, 3.0, 4.0, 5.0], confidence=0.90)
    assert wide.ci_high - wide.ci_low > narrow.ci_high - narrow.ci_low


def test_a_confidence_level_outside_zero_to_one_is_rejected():
    for bad in (0.0, 1.0, -0.5, 1.5):
        with pytest.raises(ValueError):
            summarize([1.0, 2.0], confidence=bad)

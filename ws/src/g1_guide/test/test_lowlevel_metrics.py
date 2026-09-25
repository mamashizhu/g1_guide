import pytest
from g1_guide.lowlevel_metrics import settled_window


def samples():
    return [["positive", i / 100, "joint", .2, .18, .001, .02, .1, .001, -.001]
            for i in range(101)]


def test_continuous_stable_window():
    result = settled_window(samples(), "positive", {"joint": .2})
    assert result["max_error_rad"] == pytest.approx(.02)
    assert result["positions"]["joint"] == pytest.approx(.18)


@pytest.mark.parametrize("column,value", [(4, -.5), (5, 3.0), (3, .4), (4, float("nan"))])
def test_single_good_endpoint_cannot_hide_bad_window(column, value):
    rows = samples()
    rows[50][column] = value
    with pytest.raises(ValueError):
        settled_window(rows, "positive", {"joint": .2})


def test_gap_or_missing_joint_rejected():
    with pytest.raises(ValueError):
        settled_window(samples()[20:], "positive", {"joint": .2})
    with pytest.raises(ValueError):
        settled_window(samples(), "positive", {"missing": .2})

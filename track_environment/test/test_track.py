import pytest

from track_environment.track import DEFAULT_TRACK_FILE, Track


def test_track_loading():
    """Verify that the default racetrack loads waypoints and has positive perimeter."""
    track = Track(track_file=DEFAULT_TRACK_FILE)
    assert len(track.waypoints) > 100
    assert track.total_length > 10.0
    x0, y0, psi0 = track.start_pose
    assert x0 == 0.0
    assert y0 == 0.0
    assert isinstance(x0, float)
    assert isinstance(y0, float)
    assert isinstance(psi0, float)


def test_json_track_is_rejected():
    """Verify that track loading accepts CSV files only."""
    with pytest.raises(ValueError, match='must be a CSV file'):
        Track(track_file='global_waypoints.json')


def test_boundary_csv_is_rejected_as_track():
    """Verify that boundary-marker CSV files are not loaded as waypoints."""
    with pytest.raises(ValueError, match="must contain 'x' and 'y' columns"):
        Track(track_file='random_track0.csv')

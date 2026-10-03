"""Guards for the shared rtabmap configuration (no ROS needed)."""
import pytest

from mentorpi_bringup import rtabmap_params as rp


def _rtabmap_core(params):
    # rtabmap's own parameters are the "Group/Name" ones.
    return {k: v for k, v in params.items() if '/' in k}


@pytest.mark.parametrize('build', [rp.slam_params, rp.loc_params])
def test_rtabmap_core_params_are_strings(build):
    # A bool/float here aborts rtabmap at startup (observed 2026-07-12).
    bad = {k: v for k, v in _rtabmap_core(build()).items() if not isinstance(v, str)}
    assert not bad


def test_slam_and_loc_differ_only_by_overrides():
    slam, loc = rp.slam_params(), rp.loc_params()
    assert slam.keys() == loc.keys()
    diff = {k for k in slam if slam[k] != loc[k]}
    assert diff == set(rp.LOC_TUNING_OVERRIDES)


def test_ground_height_is_relative_to_base_link():
    h = float(rp.TUNING_PARAMS['Grid/MaxGroundHeight'])
    # 0 means "disabled" to rtabmap; must be a real threshold.
    assert h != 0.0
    above_floor = h + rp.BASE_LINK_HEIGHT
    assert 0.02 <= above_floor <= 0.05


def test_rgbd_pipeline_topics_match():
    sync_out = dict(rp.RGBD_SYNC_REMAPPINGS)['rgbd_image']
    rtab_in = dict(rp.REMAPPINGS)['rgbd_image']
    assert sync_out == rtab_in == rp.RGBD_IMAGE_TOPIC
    assert rp.TOPIC_PARAMS['subscribe_rgbd'] is True
    assert rp.TOPIC_PARAMS['subscribe_rgb'] is False
    assert rp.TOPIC_PARAMS['subscribe_depth'] is False


def test_rgbd_sync_window_never_starves_15fps():
    # Nearest-neighbour pairing of two 15 fps streams is within half a
    # period; a tighter window could drop every pair.
    assert rp.RGBD_SYNC_MAX_INTERVAL >= 0.5 / 15.0


def test_shutdown_grace_fits_supervisor():
    # Supervisor SIGINTs, waits 90 s, then SIGTERM; launch must not SIGKILL
    # rtabmap before that.
    total = float(rp.RTABMAP_SIGTERM_TIMEOUT) + float(rp.RTABMAP_SIGKILL_TIMEOUT)
    assert total > 90.0
    assert float(rp.RTABMAP_SIGTERM_TIMEOUT) >= 60.0

"""
Shared rtabmap configuration for slam_3d.launch.py and loc_3d.launch.py.

Launch files can't import each other under ament, but this package module
can, so both 3D modes build their parameters from here instead of keeping
two hand-synced copies. Pure Python (no rclpy/launch imports) so it can be
unit-tested off-robot.

rtabmap's own parameters (the "Group/Name" ones) must all be strings: a bool
or float reaching rtabmap aborts it at startup (observed 2026-07-12).
"""

# base_footprint -> base_link height (mecanum.xacro base_z). rtabmap runs in
# base_link, so every Grid/* height is relative to a point 5 cm above the
# floor.
BASE_LINK_HEIGHT = 0.0505

# Points up to this far above the floor may be labelled ground. Low enough
# that 5-10 cm obstacles (flat boxes, door sills) stay in the grid, high
# enough to absorb depth noise on the floor.
GROUND_TOLERANCE = 0.04

# rgbd_sync pairing window. Gemini 2 color and depth both run at 15 fps
# (66 ms period); nearest-neighbour pairing of two free-running 15 fps
# streams is always within half a period, so 0.034 s never starves rtabmap
# but rejects pairs that are a whole frame or more apart (e.g. after a
# dropped frame). Tighten once the real color/depth offset is measured
# (scripts/check_camera_imu_sync.py).
RGBD_SYNC_MAX_INTERVAL = 0.034

# rtabmap flushes its whole database on shutdown; a ~160 MB db takes well
# over 10 s on the Pi's SD card and a kill mid-save corrupts the visual word
# dictionary. ros2 launch's own escalation (default SIGTERM after 5 s, SIGKILL
# 5 s later) would otherwise pre-empt the supervisor's 90 s grace. Launch
# escalates first (SIGTERM at 85 s, SIGKILL at 100 s); supervisor follows
# (SIGTERM at 90 s, SIGKILL at 105 s).
RTABMAP_SIGTERM_TIMEOUT = '85'
RTABMAP_SIGKILL_TIMEOUT = '15'

RGBD_IMAGE_TOPIC = '/camera/rgbd_image'

RGBD_SYNC_PARAMS = {
    'approx_sync': True,
    'approx_sync_max_interval': RGBD_SYNC_MAX_INTERVAL,
    'topic_queue_size': 10,
    'sync_queue_size': 10,
}

RGBD_SYNC_REMAPPINGS = [
    ('rgb/image', '/camera/color/image_raw'),
    ('rgb/camera_info', '/camera/color/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('rgbd_image', RGBD_IMAGE_TOPIC),
]

TOPIC_PARAMS = {
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    # RGB-D arrives pre-paired from rgbd_sync; rtabmap only has to sync
    # that with the lidar scan.
    'subscribe_rgbd': True,
    'subscribe_rgb': False,
    'subscribe_depth': False,
    'subscribe_scan': True,
    'subscribe_odom_info': False,
    'approx_sync': True,
    'topic_queue_size': 20,
    'sync_queue_size': 20,
    # MS200 driver publishes /scan with SensorDataQoS (best effort);
    # a reliable subscription would never match it.
    'qos_scan': 2,
}

TUNING_PARAMS = {
    # Ground robot: lock roll/pitch/z out of the pose graph.
    'Reg/Force3DoF': 'true',
    # 1 = ICP registration. Visual loop closures still get their initial
    # guess from visual matching; ICP on the lidar scan then refines it
    # instead of trusting the (slipping) odom guess. (2 would be joint
    # Vis+ICP.)
    'Reg/Strategy': '1',
    # Scan-match consecutive nodes against odom - the main wheel-slip fix.
    'RGBD/NeighborLinkRefining': 'true',
    # Lidar proximity detection when driving back through a mapped area.
    'RGBD/ProximityBySpace': 'true',
    'RGBD/ProximityPathMaxNeighbors': '10',
    # ICP settings for a sparse 450-point 2D scan.
    'Icp/VoxelSize': '0.05',
    'Icp/MaxCorrespondenceDistance': '0.15',
    'Icp/CorrespondenceRatio': '0.2',
    'Icp/MaxTranslation': '0.5',
    'Icp/PointToPlane': 'false',
    # Pi 5 budget.
    'Rtabmap/DetectionRate': '2.0',
    'RGBD/OptimizeMaxError': '3.0',
    'Kp/MaxFeatures': '300',
    # Occupancy grid from lidar + depth (0=scan, 1=depth, 2=both).
    'Grid/Sensor': '2',
    # Heights are in base_link (BASE_LINK_HEIGHT above the floor). 0 would
    # mean "disabled" to rtabmap, hence the explicit negative value.
    'Grid/MaxGroundHeight': f'{GROUND_TOLERANCE - BASE_LINK_HEIGHT:.4f}',
    'Grid/MaxObstacleHeight': '1.5',
    'Grid/RangeMax': '5.0',
    'Grid/3D': 'true',
    'GridGlobal/MinSize': '20.0',
}

# Localization only reads the map, so it can accept looser optimizations:
# on thin maps (short scans, few loops) 3.0 rejected correct relocalizations
# that were barely over (3.11 observed).
LOC_TUNING_OVERRIDES = {
    'RGBD/OptimizeMaxError': '5.0',
}

REMAPPINGS = [
    ('rgbd_image', RGBD_IMAGE_TOPIC),
    ('scan', '/scan'),
    ('odom', '/odometry/filtered'),
]


def slam_params():
    """Common rtabmap parameters for slam_3d (launch adds db/memory args)."""
    return {**TOPIC_PARAMS, **TUNING_PARAMS}


def loc_params():
    """Common rtabmap parameters for loc_3d (launch adds db/memory args)."""
    return {**TOPIC_PARAMS, **TUNING_PARAMS, **LOC_TUNING_OVERRIDES}

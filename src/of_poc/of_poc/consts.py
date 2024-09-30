# Parameter Names
DEBUG_MODE = 'debug_mode'
MIN_FEATURES_IN_FRAME = 'min_features_in_frame'
MAX_FULL_FRAME_FEATURES = 'max_features_in_frame'
MIN_DIST_BETWEEN_FEATURES = 'min_dist_between_features'
LK_QUALITY_LEVEL = 'lk_quality_level'
SEGMENT_AMOUNT_WIDTH = 'segment_amount_x'
SEGMENT_AMOUNT_HEIGHT = 'segment_amount_y'
MIN_INTERVAL_TO_COMPUTE = 'min_interval_to_compute'
DEFAULT_VAR_VALUE = 'default_var_value'

# topics
CAMERA_FLU_ODOM_TOPIC = "/odom/camera_flu"
TWIST_OUT_TOPIC = "/optical_flow/twist"
GROUND_TRUTH_TOPIC = "/ground_truth/twist"
CAMERA_INFO_TOPIC = '/gimbal_camera/camera_info'
CAMERA_IMAGE_TOPIC = '/gimbal_camera/image_raw'
POSE_ERROR_TEST_TOPIC = '/optical_flow/pose_error'
TWIST_WITH_COV_OUT_TOPIC = "/optical_flow/twist_with_cov"

# frames
CAMERA_LINK_FRAME = 'optical_flow/camera_physical'
MAP_Z_DOWN = 'optical_flow/base_swd'
CAMERA_LINK_OPTICAL_FRAME = 'optical_flow/camera_optical'
BASE_OF_FRAME = 'optical_flow/base'
WORLD_FRAME = 'odom'

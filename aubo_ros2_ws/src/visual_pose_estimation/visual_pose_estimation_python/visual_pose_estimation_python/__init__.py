"""`visual_pose_estimation_python` package exports.

This package is used both inside a ROS2 runtime and in lightweight
tooling/tests that may not have ROS2 dependencies available. Imports are
therefore best-effort so that subpackages such as `web` remain importable
without eagerly requiring `rclpy`.
"""

from importlib import import_module

__all__ = []


def _export(module_name, names):
    try:
        module = import_module(module_name, package=__name__)
    except Exception:
        return

    for name in names:
        globals()[name] = getattr(module, name)
        __all__.append(name)


_export(".preprocessor", ["Preprocessor"])
_export(".feature_extractor", ["FeatureExtractor", "ComponentFeature"])
_export(".template_standardizer", ["TemplateStandardizer", "StandardizedTemplate"])
_export(".config_reader", ["ConfigReader"])
_export(".config", ["AppConfig", "get_config", "load_config", "update_section"])
_export(".pose_estimator", ["PoseEstimator", "TemplateItem", "PoseEstimationResult"])
_export(".ros2_communication", ["ROS2Communication"])
_export(".path_resolver", ["resolve_web_paths", "resolve_templates_root"])
_export(".params_manager", ["ParamsManager"])
_export(".math_utils", [
    "quaternion_to_rotation_matrix",
    "rotation_matrix_to_quaternion",
    "rotation_matrix_to_euler_rpy",
    "normalize_angle_to_180",
    "normalize_angle_to_pi",
    "filter_components_by_params",
])
_export(".main", ["main"])

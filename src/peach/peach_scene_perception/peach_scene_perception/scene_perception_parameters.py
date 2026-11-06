# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import sys
import rclpy
import rclpy.parameter
from generate_parameter_library_py.python_validators import ParameterValidators



class peach_scene_perception_node:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        color_topic = "/camera/color/image_raw"
        depth_topic = "/camera/depth/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        camera_optical_frame = "camera_color_optical_frame"
        output_frame = "base_link"
        tf_timeout_sec = 0.5
        tf_fallback_timeout_sec = 0.5
        depth_scale_unit = 0.25
        sync_slop_s = 0.05
        sync_queue_size = 3
        max_frame_age_s = 0.5
        min_detection_conf = 0.4
        yolo_conf = 0.3
        yolo_nms_iou = 0.5
        yolo_imgsz = 640
        yolo_half = False
        sam_max_bboxes = 16
        sam_min_area = 100
        min_mask_points = 50
        detection_dedup_ios = 0.6
        detection_dedup_area_ratio = 0.5
        publish_debug_image = True
        debug_downscale = 1.0
        publish_masks = True
        publish_detection_cloud = True
        detection_cloud_stride = 2
        yolo_model_path = "$(find-pkg-share peach_scene_perception)/model/best.pt"
        sam_model_path = "$(find-pkg-share peach_scene_perception)/model/FastSAM-s.pt"
        model_version = "yolo:6981750db67a726e|fastsam_s:c9f78716a81c7aff"
        calibration_version = "percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006"
        gravity_hint_xyz = ""
        gravity_mode = "tf"
        class __Pipeline:
            bag_impl = "robust_bag"
            fruit_impl = "robust_fruit"
            min_depth_m = 0.3
            max_depth_m = 1.5
            min_points = 100
            locked_only_segmentation = True
        pipeline = __Pipeline()
        class __Tool:
            D_inner = 0.12
            L_insert = 0.09
            L_blade = 0.079
            entry_d_tool = 0.0
            entry_d_s = 0.0
            clearance_min = 0.005
            margin_neck = 0.015
            version = "1.1"
        tool = __Tool()
        class __TargetMemory:
            enable = True
            match_radius_m = 0.06
            max_targets = 50
            position_ema = 0.3
            recovery_scale = 1.0
            confirm_frames = 2
            tentative_ttl_frames = 20
            anchor_max_age_s = 30.0
            anchor_drop_s = 120.0
            max_age_s = 600.0
        target_memory = __TargetMemory()
        class __Wind:
            swing_threshold_m = 0.03
            swing_frames = 3
        wind = __Wind()
        class __Lighting:
            min_depth_ratio = 0.35
            min_conf_mean = 0.3
            bad_frames = 5
        lighting = __Lighting()
        class __Harvest:
            min_collect_frames = 4
            lock_settle_frames = 2
            max_collect_s = 12.0
            priority_prefer_lower_first = True
        harvest = __Harvest()



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = peach_scene_perception_node.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("peach_scene_perception_node." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.user_callback = None
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def unpack_parameter_dict(self, namespace: str, parameter_dict: dict):
            """
            Flatten a parameter dictionary recursively.

            :param namespace: The namespace to prepend to the parameter names.
            :param parameter_dict: A dictionary of parameters keyed by the parameter names
            :return: A list of rclpy Parameter objects
            """
            parameters = []
            for param_name, param_value in parameter_dict.items():
                full_param_name = namespace + param_name
                # Unroll nested parameters
                if isinstance(param_value, dict):
                    nested_params = self.unpack_parameter_dict(
                            namespace=full_param_name + rclpy.parameter.PARAMETER_SEPARATOR_STRING,
                            parameter_dict=param_value)
                    parameters.extend(nested_params)
                else:
                    parameters.append(rclpy.parameter.Parameter(full_param_name, value=param_value))
            return parameters

        def set_params_from_dict(self, param_dict):
            params_to_set = self.unpack_parameter_dict('', param_dict)
            self.update(params_to_set)

        def set_user_callback(self, callback):
            self.user_callback = callback

        def clear_user_callback(self):
            self.user_callback = None

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "color_topic":
                    updated_params.color_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "depth_topic":
                    updated_params.depth_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "camera_info_topic":
                    updated_params.camera_info_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "camera_optical_frame":
                    updated_params.camera_optical_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "output_frame":
                    updated_params.output_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_timeout_sec":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tf_timeout_sec = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_fallback_timeout_sec":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tf_fallback_timeout_sec = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "depth_scale_unit":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.depth_scale_unit = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sync_slop_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.sync_slop_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sync_queue_size":
                    updated_params.sync_queue_size = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "max_frame_age_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.max_frame_age_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "min_detection_conf":
                    updated_params.min_detection_conf = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "yolo_conf":
                    updated_params.yolo_conf = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "yolo_nms_iou":
                    updated_params.yolo_nms_iou = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "yolo_imgsz":
                    updated_params.yolo_imgsz = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "yolo_half":
                    updated_params.yolo_half = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sam_max_bboxes":
                    updated_params.sam_max_bboxes = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sam_min_area":
                    updated_params.sam_min_area = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "min_mask_points":
                    updated_params.min_mask_points = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detection_dedup_ios":
                    updated_params.detection_dedup_ios = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detection_dedup_area_ratio":
                    updated_params.detection_dedup_area_ratio = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "publish_debug_image":
                    updated_params.publish_debug_image = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug_downscale":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.debug_downscale = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "publish_masks":
                    updated_params.publish_masks = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "publish_detection_cloud":
                    updated_params.publish_detection_cloud = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detection_cloud_stride":
                    updated_params.detection_cloud_stride = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "yolo_model_path":
                    updated_params.yolo_model_path = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sam_model_path":
                    updated_params.sam_model_path = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "model_version":
                    updated_params.model_version = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "calibration_version":
                    updated_params.calibration_version = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_hint_xyz":
                    updated_params.gravity_hint_xyz = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_mode":
                    updated_params.gravity_mode = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.bag_impl":
                    updated_params.pipeline.bag_impl = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.fruit_impl":
                    updated_params.pipeline.fruit_impl = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.min_depth_m":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.pipeline.min_depth_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.max_depth_m":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.pipeline.max_depth_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.min_points":
                    updated_params.pipeline.min_points = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pipeline.locked_only_segmentation":
                    updated_params.pipeline.locked_only_segmentation = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.D_inner":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.D_inner = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.L_insert":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.L_insert = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.L_blade":
                    updated_params.tool.L_blade = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.entry_d_tool":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.entry_d_tool = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.entry_d_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.entry_d_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.clearance_min":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.clearance_min = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.margin_neck":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.tool.margin_neck = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tool.version":
                    updated_params.tool.version = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.enable":
                    updated_params.target_memory.enable = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.match_radius_m":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.match_radius_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.max_targets":
                    updated_params.target_memory.max_targets = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.position_ema":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.position_ema = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.recovery_scale":
                    validation_result = ParameterValidators.gt_eq(param, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.recovery_scale = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.confirm_frames":
                    updated_params.target_memory.confirm_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.tentative_ttl_frames":
                    updated_params.target_memory.tentative_ttl_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.anchor_max_age_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.anchor_max_age_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.anchor_drop_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.anchor_drop_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_memory.max_age_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.target_memory.max_age_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "wind.swing_threshold_m":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.wind.swing_threshold_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "wind.swing_frames":
                    updated_params.wind.swing_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "lighting.min_depth_ratio":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.lt_eq(param, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.lighting.min_depth_ratio = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "lighting.min_conf_mean":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.lt_eq(param, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.lighting.min_conf_mean = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "lighting.bad_frames":
                    updated_params.lighting.bad_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "harvest.min_collect_frames":
                    updated_params.harvest.min_collect_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "harvest.lock_settle_frames":
                    updated_params.harvest.lock_settle_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "harvest.max_collect_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.harvest.max_collect_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "harvest.priority_prefer_lower_first":
                    updated_params.harvest.priority_prefer_lower_first = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "color_topic"):
                descriptor = ParameterDescriptor(description=r"color_topic", read_only = False)
                parameter = updated_params.color_topic
                self.node_.declare_parameter(self.prefix_ + "color_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "depth_topic"):
                descriptor = ParameterDescriptor(description=r"depth_topic", read_only = False)
                parameter = updated_params.depth_topic
                self.node_.declare_parameter(self.prefix_ + "depth_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "camera_info_topic"):
                descriptor = ParameterDescriptor(description=r"camera_info_topic", read_only = False)
                parameter = updated_params.camera_info_topic
                self.node_.declare_parameter(self.prefix_ + "camera_info_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "camera_optical_frame"):
                descriptor = ParameterDescriptor(description=r"camera_optical_frame", read_only = False)
                parameter = updated_params.camera_optical_frame
                self.node_.declare_parameter(self.prefix_ + "camera_optical_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "output_frame"):
                descriptor = ParameterDescriptor(description=r"output_frame", read_only = False)
                parameter = updated_params.output_frame
                self.node_.declare_parameter(self.prefix_ + "output_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_timeout_sec"):
                descriptor = ParameterDescriptor(description=r"tf_timeout_sec", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tf_timeout_sec
                self.node_.declare_parameter(self.prefix_ + "tf_timeout_sec", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_fallback_timeout_sec"):
                descriptor = ParameterDescriptor(description=r"tf_fallback_timeout_sec", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tf_fallback_timeout_sec
                self.node_.declare_parameter(self.prefix_ + "tf_fallback_timeout_sec", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "depth_scale_unit"):
                descriptor = ParameterDescriptor(description=r"depth_scale_unit", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.depth_scale_unit
                self.node_.declare_parameter(self.prefix_ + "depth_scale_unit", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sync_slop_s"):
                descriptor = ParameterDescriptor(description=r"RGB-D sync tolerance; strictly positive, restart required", read_only = True)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.sync_slop_s
                self.node_.declare_parameter(self.prefix_ + "sync_slop_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sync_queue_size"):
                descriptor = ParameterDescriptor(description=r"sync_queue_size", read_only = False)
                parameter = updated_params.sync_queue_size
                self.node_.declare_parameter(self.prefix_ + "sync_queue_size", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "max_frame_age_s"):
                descriptor = ParameterDescriptor(description=r"max_frame_age_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.max_frame_age_s
                self.node_.declare_parameter(self.prefix_ + "max_frame_age_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "min_detection_conf"):
                descriptor = ParameterDescriptor(description=r"min_detection_conf", read_only = False)
                parameter = updated_params.min_detection_conf
                self.node_.declare_parameter(self.prefix_ + "min_detection_conf", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "yolo_conf"):
                descriptor = ParameterDescriptor(description=r"yolo_conf", read_only = False)
                parameter = updated_params.yolo_conf
                self.node_.declare_parameter(self.prefix_ + "yolo_conf", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "yolo_nms_iou"):
                descriptor = ParameterDescriptor(description=r"yolo_nms_iou", read_only = False)
                parameter = updated_params.yolo_nms_iou
                self.node_.declare_parameter(self.prefix_ + "yolo_nms_iou", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "yolo_imgsz"):
                descriptor = ParameterDescriptor(description=r"yolo_imgsz", read_only = False)
                parameter = updated_params.yolo_imgsz
                self.node_.declare_parameter(self.prefix_ + "yolo_imgsz", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "yolo_half"):
                descriptor = ParameterDescriptor(description=r"yolo_half", read_only = False)
                parameter = updated_params.yolo_half
                self.node_.declare_parameter(self.prefix_ + "yolo_half", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sam_max_bboxes"):
                descriptor = ParameterDescriptor(description=r"sam_max_bboxes", read_only = False)
                parameter = updated_params.sam_max_bboxes
                self.node_.declare_parameter(self.prefix_ + "sam_max_bboxes", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sam_min_area"):
                descriptor = ParameterDescriptor(description=r"sam_min_area", read_only = False)
                parameter = updated_params.sam_min_area
                self.node_.declare_parameter(self.prefix_ + "sam_min_area", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "min_mask_points"):
                descriptor = ParameterDescriptor(description=r"min_mask_points", read_only = False)
                parameter = updated_params.min_mask_points
                self.node_.declare_parameter(self.prefix_ + "min_mask_points", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detection_dedup_ios"):
                descriptor = ParameterDescriptor(description=r"detection_dedup_ios", read_only = False)
                parameter = updated_params.detection_dedup_ios
                self.node_.declare_parameter(self.prefix_ + "detection_dedup_ios", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detection_dedup_area_ratio"):
                descriptor = ParameterDescriptor(description=r"detection_dedup_area_ratio", read_only = False)
                parameter = updated_params.detection_dedup_area_ratio
                self.node_.declare_parameter(self.prefix_ + "detection_dedup_area_ratio", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "publish_debug_image"):
                descriptor = ParameterDescriptor(description=r"publish_debug_image", read_only = False)
                parameter = updated_params.publish_debug_image
                self.node_.declare_parameter(self.prefix_ + "publish_debug_image", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug_downscale"):
                descriptor = ParameterDescriptor(description=r"debug_downscale", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.debug_downscale
                self.node_.declare_parameter(self.prefix_ + "debug_downscale", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "publish_masks"):
                descriptor = ParameterDescriptor(description=r"publish_masks", read_only = False)
                parameter = updated_params.publish_masks
                self.node_.declare_parameter(self.prefix_ + "publish_masks", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "publish_detection_cloud"):
                descriptor = ParameterDescriptor(description=r"publish_detection_cloud", read_only = False)
                parameter = updated_params.publish_detection_cloud
                self.node_.declare_parameter(self.prefix_ + "publish_detection_cloud", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detection_cloud_stride"):
                descriptor = ParameterDescriptor(description=r"detection_cloud_stride", read_only = False)
                parameter = updated_params.detection_cloud_stride
                self.node_.declare_parameter(self.prefix_ + "detection_cloud_stride", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "yolo_model_path"):
                descriptor = ParameterDescriptor(description=r"yolo_model_path", read_only = False)
                parameter = updated_params.yolo_model_path
                self.node_.declare_parameter(self.prefix_ + "yolo_model_path", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sam_model_path"):
                descriptor = ParameterDescriptor(description=r"sam_model_path", read_only = False)
                parameter = updated_params.sam_model_path
                self.node_.declare_parameter(self.prefix_ + "sam_model_path", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "model_version"):
                descriptor = ParameterDescriptor(description=r"model_version", read_only = False)
                parameter = updated_params.model_version
                self.node_.declare_parameter(self.prefix_ + "model_version", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "calibration_version"):
                descriptor = ParameterDescriptor(description=r"calibration_version", read_only = False)
                parameter = updated_params.calibration_version
                self.node_.declare_parameter(self.prefix_ + "calibration_version", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_hint_xyz"):
                descriptor = ParameterDescriptor(description=r"gravity_hint_xyz", read_only = False)
                parameter = updated_params.gravity_hint_xyz
                self.node_.declare_parameter(self.prefix_ + "gravity_hint_xyz", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_mode"):
                descriptor = ParameterDescriptor(description=r"gravity_mode", read_only = False)
                parameter = updated_params.gravity_mode
                self.node_.declare_parameter(self.prefix_ + "gravity_mode", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.bag_impl"):
                descriptor = ParameterDescriptor(description=r"pipeline.bag_impl", read_only = False)
                parameter = updated_params.pipeline.bag_impl
                self.node_.declare_parameter(self.prefix_ + "pipeline.bag_impl", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.fruit_impl"):
                descriptor = ParameterDescriptor(description=r"pipeline.fruit_impl", read_only = False)
                parameter = updated_params.pipeline.fruit_impl
                self.node_.declare_parameter(self.prefix_ + "pipeline.fruit_impl", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.min_depth_m"):
                descriptor = ParameterDescriptor(description=r"pipeline.min_depth_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.pipeline.min_depth_m
                self.node_.declare_parameter(self.prefix_ + "pipeline.min_depth_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.max_depth_m"):
                descriptor = ParameterDescriptor(description=r"pipeline.max_depth_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.pipeline.max_depth_m
                self.node_.declare_parameter(self.prefix_ + "pipeline.max_depth_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.min_points"):
                descriptor = ParameterDescriptor(description=r"pipeline.min_points", read_only = False)
                parameter = updated_params.pipeline.min_points
                self.node_.declare_parameter(self.prefix_ + "pipeline.min_points", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pipeline.locked_only_segmentation"):
                descriptor = ParameterDescriptor(description=r"pipeline.locked_only_segmentation", read_only = False)
                parameter = updated_params.pipeline.locked_only_segmentation
                self.node_.declare_parameter(self.prefix_ + "pipeline.locked_only_segmentation", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.D_inner"):
                descriptor = ParameterDescriptor(description=r"tool.D_inner", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.D_inner
                self.node_.declare_parameter(self.prefix_ + "tool.D_inner", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.L_insert"):
                descriptor = ParameterDescriptor(description=r"tool.L_insert", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.L_insert
                self.node_.declare_parameter(self.prefix_ + "tool.L_insert", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.L_blade"):
                descriptor = ParameterDescriptor(description=r"tool.L_blade", read_only = False)
                parameter = updated_params.tool.L_blade
                self.node_.declare_parameter(self.prefix_ + "tool.L_blade", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.entry_d_tool"):
                descriptor = ParameterDescriptor(description=r"tool.entry_d_tool", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.entry_d_tool
                self.node_.declare_parameter(self.prefix_ + "tool.entry_d_tool", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.entry_d_s"):
                descriptor = ParameterDescriptor(description=r"tool.entry_d_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.entry_d_s
                self.node_.declare_parameter(self.prefix_ + "tool.entry_d_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.clearance_min"):
                descriptor = ParameterDescriptor(description=r"tool.clearance_min", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.clearance_min
                self.node_.declare_parameter(self.prefix_ + "tool.clearance_min", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.margin_neck"):
                descriptor = ParameterDescriptor(description=r"tool.margin_neck", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.tool.margin_neck
                self.node_.declare_parameter(self.prefix_ + "tool.margin_neck", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tool.version"):
                descriptor = ParameterDescriptor(description=r"tool.version", read_only = False)
                parameter = updated_params.tool.version
                self.node_.declare_parameter(self.prefix_ + "tool.version", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.enable"):
                descriptor = ParameterDescriptor(description=r"target_memory.enable", read_only = False)
                parameter = updated_params.target_memory.enable
                self.node_.declare_parameter(self.prefix_ + "target_memory.enable", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.match_radius_m"):
                descriptor = ParameterDescriptor(description=r"target_memory.match_radius_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.match_radius_m
                self.node_.declare_parameter(self.prefix_ + "target_memory.match_radius_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.max_targets"):
                descriptor = ParameterDescriptor(description=r"target_memory.max_targets", read_only = False)
                parameter = updated_params.target_memory.max_targets
                self.node_.declare_parameter(self.prefix_ + "target_memory.max_targets", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.position_ema"):
                descriptor = ParameterDescriptor(description=r"target_memory.position_ema", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.position_ema
                self.node_.declare_parameter(self.prefix_ + "target_memory.position_ema", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.recovery_scale"):
                descriptor = ParameterDescriptor(description=r"target_memory.recovery_scale", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.recovery_scale
                self.node_.declare_parameter(self.prefix_ + "target_memory.recovery_scale", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.confirm_frames"):
                descriptor = ParameterDescriptor(description=r"target_memory.confirm_frames", read_only = False)
                parameter = updated_params.target_memory.confirm_frames
                self.node_.declare_parameter(self.prefix_ + "target_memory.confirm_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.tentative_ttl_frames"):
                descriptor = ParameterDescriptor(description=r"target_memory.tentative_ttl_frames", read_only = False)
                parameter = updated_params.target_memory.tentative_ttl_frames
                self.node_.declare_parameter(self.prefix_ + "target_memory.tentative_ttl_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.anchor_max_age_s"):
                descriptor = ParameterDescriptor(description=r"target_memory.anchor_max_age_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.anchor_max_age_s
                self.node_.declare_parameter(self.prefix_ + "target_memory.anchor_max_age_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.anchor_drop_s"):
                descriptor = ParameterDescriptor(description=r"target_memory.anchor_drop_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.anchor_drop_s
                self.node_.declare_parameter(self.prefix_ + "target_memory.anchor_drop_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_memory.max_age_s"):
                descriptor = ParameterDescriptor(description=r"target_memory.max_age_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.target_memory.max_age_s
                self.node_.declare_parameter(self.prefix_ + "target_memory.max_age_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "wind.swing_threshold_m"):
                descriptor = ParameterDescriptor(description=r"wind.swing_threshold_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.wind.swing_threshold_m
                self.node_.declare_parameter(self.prefix_ + "wind.swing_threshold_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "wind.swing_frames"):
                descriptor = ParameterDescriptor(description=r"wind.swing_frames", read_only = False)
                parameter = updated_params.wind.swing_frames
                self.node_.declare_parameter(self.prefix_ + "wind.swing_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "lighting.min_depth_ratio"):
                descriptor = ParameterDescriptor(description=r"lighting.min_depth_ratio", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = 1.0
                parameter = updated_params.lighting.min_depth_ratio
                self.node_.declare_parameter(self.prefix_ + "lighting.min_depth_ratio", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "lighting.min_conf_mean"):
                descriptor = ParameterDescriptor(description=r"lighting.min_conf_mean", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = 1.0
                parameter = updated_params.lighting.min_conf_mean
                self.node_.declare_parameter(self.prefix_ + "lighting.min_conf_mean", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "lighting.bad_frames"):
                descriptor = ParameterDescriptor(description=r"lighting.bad_frames", read_only = False)
                parameter = updated_params.lighting.bad_frames
                self.node_.declare_parameter(self.prefix_ + "lighting.bad_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "harvest.min_collect_frames"):
                descriptor = ParameterDescriptor(description=r"harvest.min_collect_frames", read_only = False)
                parameter = updated_params.harvest.min_collect_frames
                self.node_.declare_parameter(self.prefix_ + "harvest.min_collect_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "harvest.lock_settle_frames"):
                descriptor = ParameterDescriptor(description=r"harvest.lock_settle_frames", read_only = False)
                parameter = updated_params.harvest.lock_settle_frames
                self.node_.declare_parameter(self.prefix_ + "harvest.lock_settle_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "harvest.max_collect_s"):
                descriptor = ParameterDescriptor(description=r"harvest.max_collect_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.harvest.max_collect_s
                self.node_.declare_parameter(self.prefix_ + "harvest.max_collect_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "harvest.priority_prefer_lower_first"):
                descriptor = ParameterDescriptor(description=r"harvest.priority_prefer_lower_first", read_only = False)
                parameter = updated_params.harvest.priority_prefer_lower_first
                self.node_.declare_parameter(self.prefix_ + "harvest.priority_prefer_lower_first", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "color_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.color_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "depth_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.depth_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "camera_info_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.camera_info_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "camera_optical_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.camera_optical_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "output_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.output_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_timeout_sec")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tf_timeout_sec',param.value, 'Invalid value set during initialization for parameter tf_timeout_sec: ' + validation_result)
            updated_params.tf_timeout_sec = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_fallback_timeout_sec")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tf_fallback_timeout_sec',param.value, 'Invalid value set during initialization for parameter tf_fallback_timeout_sec: ' + validation_result)
            updated_params.tf_fallback_timeout_sec = param.value
            param = self.node_.get_parameter(self.prefix_ + "depth_scale_unit")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('depth_scale_unit',param.value, 'Invalid value set during initialization for parameter depth_scale_unit: ' + validation_result)
            updated_params.depth_scale_unit = param.value
            param = self.node_.get_parameter(self.prefix_ + "sync_slop_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('sync_slop_s',param.value, 'Invalid value set during initialization for parameter sync_slop_s: ' + validation_result)
            updated_params.sync_slop_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "sync_queue_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sync_queue_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "max_frame_age_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('max_frame_age_s',param.value, 'Invalid value set during initialization for parameter max_frame_age_s: ' + validation_result)
            updated_params.max_frame_age_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "min_detection_conf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.min_detection_conf = param.value
            param = self.node_.get_parameter(self.prefix_ + "yolo_conf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.yolo_conf = param.value
            param = self.node_.get_parameter(self.prefix_ + "yolo_nms_iou")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.yolo_nms_iou = param.value
            param = self.node_.get_parameter(self.prefix_ + "yolo_imgsz")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.yolo_imgsz = param.value
            param = self.node_.get_parameter(self.prefix_ + "yolo_half")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.yolo_half = param.value
            param = self.node_.get_parameter(self.prefix_ + "sam_max_bboxes")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sam_max_bboxes = param.value
            param = self.node_.get_parameter(self.prefix_ + "sam_min_area")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sam_min_area = param.value
            param = self.node_.get_parameter(self.prefix_ + "min_mask_points")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.min_mask_points = param.value
            param = self.node_.get_parameter(self.prefix_ + "detection_dedup_ios")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detection_dedup_ios = param.value
            param = self.node_.get_parameter(self.prefix_ + "detection_dedup_area_ratio")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detection_dedup_area_ratio = param.value
            param = self.node_.get_parameter(self.prefix_ + "publish_debug_image")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.publish_debug_image = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug_downscale")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('debug_downscale',param.value, 'Invalid value set during initialization for parameter debug_downscale: ' + validation_result)
            updated_params.debug_downscale = param.value
            param = self.node_.get_parameter(self.prefix_ + "publish_masks")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.publish_masks = param.value
            param = self.node_.get_parameter(self.prefix_ + "publish_detection_cloud")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.publish_detection_cloud = param.value
            param = self.node_.get_parameter(self.prefix_ + "detection_cloud_stride")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detection_cloud_stride = param.value
            param = self.node_.get_parameter(self.prefix_ + "yolo_model_path")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.yolo_model_path = param.value
            param = self.node_.get_parameter(self.prefix_ + "sam_model_path")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sam_model_path = param.value
            param = self.node_.get_parameter(self.prefix_ + "model_version")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.model_version = param.value
            param = self.node_.get_parameter(self.prefix_ + "calibration_version")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.calibration_version = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_hint_xyz")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gravity_hint_xyz = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_mode")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gravity_mode = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.bag_impl")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pipeline.bag_impl = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.fruit_impl")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pipeline.fruit_impl = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.min_depth_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('pipeline.min_depth_m',param.value, 'Invalid value set during initialization for parameter pipeline.min_depth_m: ' + validation_result)
            updated_params.pipeline.min_depth_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.max_depth_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('pipeline.max_depth_m',param.value, 'Invalid value set during initialization for parameter pipeline.max_depth_m: ' + validation_result)
            updated_params.pipeline.max_depth_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.min_points")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pipeline.min_points = param.value
            param = self.node_.get_parameter(self.prefix_ + "pipeline.locked_only_segmentation")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pipeline.locked_only_segmentation = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.D_inner")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.D_inner',param.value, 'Invalid value set during initialization for parameter tool.D_inner: ' + validation_result)
            updated_params.tool.D_inner = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.L_insert")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.L_insert',param.value, 'Invalid value set during initialization for parameter tool.L_insert: ' + validation_result)
            updated_params.tool.L_insert = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.L_blade")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tool.L_blade = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.entry_d_tool")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.entry_d_tool',param.value, 'Invalid value set during initialization for parameter tool.entry_d_tool: ' + validation_result)
            updated_params.tool.entry_d_tool = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.entry_d_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.entry_d_s',param.value, 'Invalid value set during initialization for parameter tool.entry_d_s: ' + validation_result)
            updated_params.tool.entry_d_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.clearance_min")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.clearance_min',param.value, 'Invalid value set during initialization for parameter tool.clearance_min: ' + validation_result)
            updated_params.tool.clearance_min = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.margin_neck")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('tool.margin_neck',param.value, 'Invalid value set during initialization for parameter tool.margin_neck: ' + validation_result)
            updated_params.tool.margin_neck = param.value
            param = self.node_.get_parameter(self.prefix_ + "tool.version")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tool.version = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.enable")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.target_memory.enable = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.match_radius_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.match_radius_m',param.value, 'Invalid value set during initialization for parameter target_memory.match_radius_m: ' + validation_result)
            updated_params.target_memory.match_radius_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.max_targets")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.target_memory.max_targets = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.position_ema")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.position_ema',param.value, 'Invalid value set during initialization for parameter target_memory.position_ema: ' + validation_result)
            updated_params.target_memory.position_ema = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.recovery_scale")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 1.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.recovery_scale',param.value, 'Invalid value set during initialization for parameter target_memory.recovery_scale: ' + validation_result)
            updated_params.target_memory.recovery_scale = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.confirm_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.target_memory.confirm_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.tentative_ttl_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.target_memory.tentative_ttl_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.anchor_max_age_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.anchor_max_age_s',param.value, 'Invalid value set during initialization for parameter target_memory.anchor_max_age_s: ' + validation_result)
            updated_params.target_memory.anchor_max_age_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.anchor_drop_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.anchor_drop_s',param.value, 'Invalid value set during initialization for parameter target_memory.anchor_drop_s: ' + validation_result)
            updated_params.target_memory.anchor_drop_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_memory.max_age_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('target_memory.max_age_s',param.value, 'Invalid value set during initialization for parameter target_memory.max_age_s: ' + validation_result)
            updated_params.target_memory.max_age_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "wind.swing_threshold_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('wind.swing_threshold_m',param.value, 'Invalid value set during initialization for parameter wind.swing_threshold_m: ' + validation_result)
            updated_params.wind.swing_threshold_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "wind.swing_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.wind.swing_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "lighting.min_depth_ratio")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('lighting.min_depth_ratio',param.value, 'Invalid value set during initialization for parameter lighting.min_depth_ratio: ' + validation_result)
            validation_result = ParameterValidators.lt_eq(param, 1.0)
            if validation_result:
                raise InvalidParameterValueException('lighting.min_depth_ratio',param.value, 'Invalid value set during initialization for parameter lighting.min_depth_ratio: ' + validation_result)
            updated_params.lighting.min_depth_ratio = param.value
            param = self.node_.get_parameter(self.prefix_ + "lighting.min_conf_mean")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('lighting.min_conf_mean',param.value, 'Invalid value set during initialization for parameter lighting.min_conf_mean: ' + validation_result)
            validation_result = ParameterValidators.lt_eq(param, 1.0)
            if validation_result:
                raise InvalidParameterValueException('lighting.min_conf_mean',param.value, 'Invalid value set during initialization for parameter lighting.min_conf_mean: ' + validation_result)
            updated_params.lighting.min_conf_mean = param.value
            param = self.node_.get_parameter(self.prefix_ + "lighting.bad_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.lighting.bad_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "harvest.min_collect_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.harvest.min_collect_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "harvest.lock_settle_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.harvest.lock_settle_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "harvest.max_collect_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('harvest.max_collect_s',param.value, 'Invalid value set during initialization for parameter harvest.max_collect_s: ' + validation_result)
            updated_params.harvest.max_collect_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "harvest.priority_prefer_lower_first")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.harvest.priority_prefer_lower_first = param.value


            self.update_internal_params(updated_params)

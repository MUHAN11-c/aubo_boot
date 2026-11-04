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



class peach_observability:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        host = "127.0.0.1"
        port = 8090
        startup_facts = ""
        robot_status_probe_enabled = False
        param_poll_period_s = 3.0
        target_observations_topic = "/peach/perception/target_observations"
        harvest_state_topic = "/peach/perception/harvest_state"
        reconstruction_status_topic = "/peach/reconstruction/status"
        reconstruction_diagnostics_topic = "/peach/reconstruction/diagnostics"
        reconstruction_diagnostics_debug_topic = "/peach/reconstruction/diagnostics_debug"
        grasp_decision_topic = "/peach/reconstruction/grasp_decision"
        refined_pose_topic = "/peach/reconstruction/refined_pose"
        refined_axis_topic = "/peach/reconstruction/refined_axis"
        refined_diagnostics_topic = "/peach/reconstruction/refined_diagnostics"
        manipulation_status_topic = "/peach_arm/status"
        grasp_hypothesis_topic = "/peach/manipulation/grasp_hypothesis"
        task_executor_state_topic = "/peach_supervisor/state"
        task_executor_events_topic = "/peach_supervisor/events"
        robot_status_topic = "/aubo_io_controller/robot_status"
        joint_states_topic = "/joint_states"
        joint_status_topic = "/aubo_io_controller/joint_status"
        tf_topic = "/tf"
        tf_static_topic = "/tf_static"
        scene_snapshot_topic = "/peach_supervisor/scene_snapshot"
        job_topic = "/peach/observability/job"
        metrics_topic = "/peach/observability/metrics"
        event_buffer_size = 100
        log_buffer_size = 200
        metrics_period_s = 1.0
        metrics_process_patterns = ["peach_scene_perception_node", "peach_target_reconstruction_node", "peach_arm", "peach_supervisor", "ros2_control_node", "percipio"]
        debug_image_topic = "/peach/perception/debug_image"
        debug_image_raw_topic = "/peach/perception/debug_image_raw"
        tsdf_cloud_topic = "/peach/reconstruction/tsdf_cloud"
        tcp_path_topic = "/peach/observability/tcp_path"
        tcp_markers_topic = "/peach/observability/markers"
        class __Selfcheck:
            enabled = True
            period_s = 30.0
            initial_settle_s = 10.0
            initial_timeout_s = 90.0
            rate_window_s = 5.0
            camera_probe_enabled = False
            camera_min_hz = 2.0
            disk_min_gb = 10.0
            expected_joints = ["shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"]
            expected_tcp_norm_m = -1.0
            moveit_expected = True
            force_expected = True
            model_paths = ""
            color_image_topic = "/camera/color/image_raw"
            depth_image_topic = "/camera/depth/image_raw"
        selfcheck = __Selfcheck()
        class __Record:
            enabled = True
            root_dir = ""
            save_images = True
            save_clouds = True
            rosout = True
            level = "std"
            max_total_bag_gb = 100.0
            queue_depth = 512
        record = __Record()
        class __Trajectory:
            enabled = True
            base_frame = "base_link"
            tip_frame = "tcp"
            period_s = 0.05
            min_step_m = 0.003
            max_points = 8000
        trajectory = __Trajectory()
        class __Debug:
            enabled = True
            motion_enabled = False
            action_timeout_s = 180.0
            audit_enabled = True
            class __Endpoints:
                run_harvest_action = "/peach_supervisor/run_harvest"
                control_service = "/peach_supervisor/control"
                begin_scene_service = "/peach_scene_perception_node/begin_scene"
                survey_action = "/peach_arm/survey_scene"
                execute_action = "/peach_arm/execute_target"
                build_action = "/peach_target_reconstruction_node/build_target_model"
                check_reachability_service = "/peach_arm/check_reachability"
                photo_pose_service = "/peach_arm/go_to_photo_pose"
                preview_approach_service = "/peach_arm/preview_approach_insert"
                preview_full_service = "/peach_arm/preview_full_contact"
                ack_recovery_service = "/peach_arm/acknowledge_recovery"
                arm_service = "/peach_arm/set_execution_armed"
                skill_cancel_service = "/peach_arm/cancel_cycle"
                recon_save_session_service = "/peach_target_reconstruction_node/save_session"
                recon_reset_service = "/peach_target_reconstruction_node/reset_reconstruction"
                recon_finalize_service = "/peach_target_reconstruction_node/finalize_reconstruction"
                recon_query_service = "/peach_target_reconstruction_node/query_reconstruction_state"
                manage_nodes_service = "/peach_lifecycle_manager/manage_nodes"
                set_enables_service = "/peach_supervisor/set_enables"
            endpoints = __Endpoints()
        debug = __Debug()



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = peach_observability.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("peach_observability." + prefix)

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
                if param.name == self.prefix_ + "host":
                    updated_params.host = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "port":
                    updated_params.port = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "startup_facts":
                    updated_params.startup_facts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "robot_status_probe_enabled":
                    updated_params.robot_status_probe_enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.enabled":
                    updated_params.selfcheck.enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.period_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.period_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.initial_settle_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.initial_settle_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.initial_timeout_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.initial_timeout_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.rate_window_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.rate_window_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.camera_probe_enabled":
                    updated_params.selfcheck.camera_probe_enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.camera_min_hz":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.camera_min_hz = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.disk_min_gb":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.disk_min_gb = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.expected_joints":
                    updated_params.selfcheck.expected_joints = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.expected_tcp_norm_m":
                    validation_result = ParameterValidators.gt_eq(param, -1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.selfcheck.expected_tcp_norm_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.moveit_expected":
                    updated_params.selfcheck.moveit_expected = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.force_expected":
                    updated_params.selfcheck.force_expected = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.model_paths":
                    updated_params.selfcheck.model_paths = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.color_image_topic":
                    updated_params.selfcheck.color_image_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "selfcheck.depth_image_topic":
                    updated_params.selfcheck.depth_image_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "param_poll_period_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.param_poll_period_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "target_observations_topic":
                    updated_params.target_observations_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "harvest_state_topic":
                    updated_params.harvest_state_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "reconstruction_status_topic":
                    updated_params.reconstruction_status_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "reconstruction_diagnostics_topic":
                    updated_params.reconstruction_diagnostics_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "reconstruction_diagnostics_debug_topic":
                    updated_params.reconstruction_diagnostics_debug_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "grasp_decision_topic":
                    updated_params.grasp_decision_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "refined_pose_topic":
                    updated_params.refined_pose_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "refined_axis_topic":
                    updated_params.refined_axis_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "refined_diagnostics_topic":
                    updated_params.refined_diagnostics_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "manipulation_status_topic":
                    updated_params.manipulation_status_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "grasp_hypothesis_topic":
                    updated_params.grasp_hypothesis_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "task_executor_state_topic":
                    updated_params.task_executor_state_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "task_executor_events_topic":
                    updated_params.task_executor_events_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "robot_status_topic":
                    updated_params.robot_status_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "joint_states_topic":
                    updated_params.joint_states_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "joint_status_topic":
                    updated_params.joint_status_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_topic":
                    updated_params.tf_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_static_topic":
                    updated_params.tf_static_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "scene_snapshot_topic":
                    updated_params.scene_snapshot_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "job_topic":
                    updated_params.job_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "metrics_topic":
                    updated_params.metrics_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "event_buffer_size":
                    updated_params.event_buffer_size = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "log_buffer_size":
                    updated_params.log_buffer_size = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "metrics_period_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.metrics_period_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "metrics_process_patterns":
                    updated_params.metrics_process_patterns = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.enabled":
                    updated_params.record.enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.root_dir":
                    updated_params.record.root_dir = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.save_images":
                    updated_params.record.save_images = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.save_clouds":
                    updated_params.record.save_clouds = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.rosout":
                    updated_params.record.rosout = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.level":
                    updated_params.record.level = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.max_total_bag_gb":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.record.max_total_bag_gb = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "record.queue_depth":
                    updated_params.record.queue_depth = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.enabled":
                    updated_params.trajectory.enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.base_frame":
                    updated_params.trajectory.base_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.tip_frame":
                    updated_params.trajectory.tip_frame = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.period_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory.period_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.min_step_m":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory.min_step_m = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "trajectory.max_points":
                    updated_params.trajectory.max_points = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug_image_topic":
                    updated_params.debug_image_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug_image_raw_topic":
                    updated_params.debug_image_raw_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tsdf_cloud_topic":
                    updated_params.tsdf_cloud_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tcp_path_topic":
                    updated_params.tcp_path_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tcp_markers_topic":
                    updated_params.tcp_markers_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.enabled":
                    updated_params.debug.enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.motion_enabled":
                    updated_params.debug.motion_enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.action_timeout_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.debug.action_timeout_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.audit_enabled":
                    updated_params.debug.audit_enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.run_harvest_action":
                    updated_params.debug.endpoints.run_harvest_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.control_service":
                    updated_params.debug.endpoints.control_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.begin_scene_service":
                    updated_params.debug.endpoints.begin_scene_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.survey_action":
                    updated_params.debug.endpoints.survey_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.execute_action":
                    updated_params.debug.endpoints.execute_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.build_action":
                    updated_params.debug.endpoints.build_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.check_reachability_service":
                    updated_params.debug.endpoints.check_reachability_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.photo_pose_service":
                    updated_params.debug.endpoints.photo_pose_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.preview_approach_service":
                    updated_params.debug.endpoints.preview_approach_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.preview_full_service":
                    updated_params.debug.endpoints.preview_full_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.ack_recovery_service":
                    updated_params.debug.endpoints.ack_recovery_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.arm_service":
                    updated_params.debug.endpoints.arm_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.skill_cancel_service":
                    updated_params.debug.endpoints.skill_cancel_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.recon_save_session_service":
                    updated_params.debug.endpoints.recon_save_session_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.recon_reset_service":
                    updated_params.debug.endpoints.recon_reset_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.recon_finalize_service":
                    updated_params.debug.endpoints.recon_finalize_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.recon_query_service":
                    updated_params.debug.endpoints.recon_query_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.manage_nodes_service":
                    updated_params.debug.endpoints.manage_nodes_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "debug.endpoints.set_enables_service":
                    updated_params.debug.endpoints.set_enables_service = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "host"):
                descriptor = ParameterDescriptor(description=r"host", read_only = False)
                parameter = updated_params.host
                self.node_.declare_parameter(self.prefix_ + "host", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "port"):
                descriptor = ParameterDescriptor(description=r"port", read_only = False)
                parameter = updated_params.port
                self.node_.declare_parameter(self.prefix_ + "port", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "startup_facts"):
                descriptor = ParameterDescriptor(description=r"startup_facts", read_only = False)
                parameter = updated_params.startup_facts
                self.node_.declare_parameter(self.prefix_ + "startup_facts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "robot_status_probe_enabled"):
                descriptor = ParameterDescriptor(description=r"robot_status_probe_enabled", read_only = False)
                parameter = updated_params.robot_status_probe_enabled
                self.node_.declare_parameter(self.prefix_ + "robot_status_probe_enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.enabled"):
                descriptor = ParameterDescriptor(description=r"selfcheck.enabled", read_only = False)
                parameter = updated_params.selfcheck.enabled
                self.node_.declare_parameter(self.prefix_ + "selfcheck.enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.period_s"):
                descriptor = ParameterDescriptor(description=r"selfcheck.period_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.period_s
                self.node_.declare_parameter(self.prefix_ + "selfcheck.period_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.initial_settle_s"):
                descriptor = ParameterDescriptor(description=r"selfcheck.initial_settle_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.initial_settle_s
                self.node_.declare_parameter(self.prefix_ + "selfcheck.initial_settle_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.initial_timeout_s"):
                descriptor = ParameterDescriptor(description=r"selfcheck.initial_timeout_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.initial_timeout_s
                self.node_.declare_parameter(self.prefix_ + "selfcheck.initial_timeout_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.rate_window_s"):
                descriptor = ParameterDescriptor(description=r"selfcheck.rate_window_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.rate_window_s
                self.node_.declare_parameter(self.prefix_ + "selfcheck.rate_window_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.camera_probe_enabled"):
                descriptor = ParameterDescriptor(description=r"selfcheck.camera_probe_enabled", read_only = False)
                parameter = updated_params.selfcheck.camera_probe_enabled
                self.node_.declare_parameter(self.prefix_ + "selfcheck.camera_probe_enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.camera_min_hz"):
                descriptor = ParameterDescriptor(description=r"selfcheck.camera_min_hz", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.camera_min_hz
                self.node_.declare_parameter(self.prefix_ + "selfcheck.camera_min_hz", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.disk_min_gb"):
                descriptor = ParameterDescriptor(description=r"selfcheck.disk_min_gb", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.disk_min_gb
                self.node_.declare_parameter(self.prefix_ + "selfcheck.disk_min_gb", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.expected_joints"):
                descriptor = ParameterDescriptor(description=r"selfcheck.expected_joints", read_only = False)
                parameter = updated_params.selfcheck.expected_joints
                self.node_.declare_parameter(self.prefix_ + "selfcheck.expected_joints", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.expected_tcp_norm_m"):
                descriptor = ParameterDescriptor(description=r"selfcheck.expected_tcp_norm_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = -1.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.selfcheck.expected_tcp_norm_m
                self.node_.declare_parameter(self.prefix_ + "selfcheck.expected_tcp_norm_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.moveit_expected"):
                descriptor = ParameterDescriptor(description=r"selfcheck.moveit_expected", read_only = False)
                parameter = updated_params.selfcheck.moveit_expected
                self.node_.declare_parameter(self.prefix_ + "selfcheck.moveit_expected", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.force_expected"):
                descriptor = ParameterDescriptor(description=r"selfcheck.force_expected", read_only = False)
                parameter = updated_params.selfcheck.force_expected
                self.node_.declare_parameter(self.prefix_ + "selfcheck.force_expected", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.model_paths"):
                descriptor = ParameterDescriptor(description=r"selfcheck.model_paths", read_only = False)
                parameter = updated_params.selfcheck.model_paths
                self.node_.declare_parameter(self.prefix_ + "selfcheck.model_paths", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.color_image_topic"):
                descriptor = ParameterDescriptor(description=r"selfcheck.color_image_topic", read_only = False)
                parameter = updated_params.selfcheck.color_image_topic
                self.node_.declare_parameter(self.prefix_ + "selfcheck.color_image_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "selfcheck.depth_image_topic"):
                descriptor = ParameterDescriptor(description=r"selfcheck.depth_image_topic", read_only = False)
                parameter = updated_params.selfcheck.depth_image_topic
                self.node_.declare_parameter(self.prefix_ + "selfcheck.depth_image_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "param_poll_period_s"):
                descriptor = ParameterDescriptor(description=r"param_poll_period_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.param_poll_period_s
                self.node_.declare_parameter(self.prefix_ + "param_poll_period_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "target_observations_topic"):
                descriptor = ParameterDescriptor(description=r"target_observations_topic", read_only = False)
                parameter = updated_params.target_observations_topic
                self.node_.declare_parameter(self.prefix_ + "target_observations_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "harvest_state_topic"):
                descriptor = ParameterDescriptor(description=r"harvest_state_topic", read_only = False)
                parameter = updated_params.harvest_state_topic
                self.node_.declare_parameter(self.prefix_ + "harvest_state_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "reconstruction_status_topic"):
                descriptor = ParameterDescriptor(description=r"reconstruction_status_topic", read_only = False)
                parameter = updated_params.reconstruction_status_topic
                self.node_.declare_parameter(self.prefix_ + "reconstruction_status_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "reconstruction_diagnostics_topic"):
                descriptor = ParameterDescriptor(description=r"reconstruction_diagnostics_topic", read_only = False)
                parameter = updated_params.reconstruction_diagnostics_topic
                self.node_.declare_parameter(self.prefix_ + "reconstruction_diagnostics_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "reconstruction_diagnostics_debug_topic"):
                descriptor = ParameterDescriptor(description=r"reconstruction_diagnostics_debug_topic", read_only = False)
                parameter = updated_params.reconstruction_diagnostics_debug_topic
                self.node_.declare_parameter(self.prefix_ + "reconstruction_diagnostics_debug_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "grasp_decision_topic"):
                descriptor = ParameterDescriptor(description=r"grasp_decision_topic", read_only = False)
                parameter = updated_params.grasp_decision_topic
                self.node_.declare_parameter(self.prefix_ + "grasp_decision_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "refined_pose_topic"):
                descriptor = ParameterDescriptor(description=r"refined_pose_topic", read_only = False)
                parameter = updated_params.refined_pose_topic
                self.node_.declare_parameter(self.prefix_ + "refined_pose_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "refined_axis_topic"):
                descriptor = ParameterDescriptor(description=r"refined_axis_topic", read_only = False)
                parameter = updated_params.refined_axis_topic
                self.node_.declare_parameter(self.prefix_ + "refined_axis_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "refined_diagnostics_topic"):
                descriptor = ParameterDescriptor(description=r"refined_diagnostics_topic", read_only = False)
                parameter = updated_params.refined_diagnostics_topic
                self.node_.declare_parameter(self.prefix_ + "refined_diagnostics_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "manipulation_status_topic"):
                descriptor = ParameterDescriptor(description=r"manipulation_status_topic", read_only = False)
                parameter = updated_params.manipulation_status_topic
                self.node_.declare_parameter(self.prefix_ + "manipulation_status_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "grasp_hypothesis_topic"):
                descriptor = ParameterDescriptor(description=r"grasp_hypothesis_topic", read_only = False)
                parameter = updated_params.grasp_hypothesis_topic
                self.node_.declare_parameter(self.prefix_ + "grasp_hypothesis_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "task_executor_state_topic"):
                descriptor = ParameterDescriptor(description=r"task_executor_state_topic", read_only = False)
                parameter = updated_params.task_executor_state_topic
                self.node_.declare_parameter(self.prefix_ + "task_executor_state_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "task_executor_events_topic"):
                descriptor = ParameterDescriptor(description=r"task_executor_events_topic", read_only = False)
                parameter = updated_params.task_executor_events_topic
                self.node_.declare_parameter(self.prefix_ + "task_executor_events_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "robot_status_topic"):
                descriptor = ParameterDescriptor(description=r"robot_status_topic", read_only = False)
                parameter = updated_params.robot_status_topic
                self.node_.declare_parameter(self.prefix_ + "robot_status_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "joint_states_topic"):
                descriptor = ParameterDescriptor(description=r"joint_states_topic", read_only = False)
                parameter = updated_params.joint_states_topic
                self.node_.declare_parameter(self.prefix_ + "joint_states_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "joint_status_topic"):
                descriptor = ParameterDescriptor(description=r"joint_status_topic", read_only = False)
                parameter = updated_params.joint_status_topic
                self.node_.declare_parameter(self.prefix_ + "joint_status_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_topic"):
                descriptor = ParameterDescriptor(description=r"tf_topic", read_only = False)
                parameter = updated_params.tf_topic
                self.node_.declare_parameter(self.prefix_ + "tf_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_static_topic"):
                descriptor = ParameterDescriptor(description=r"tf_static_topic", read_only = False)
                parameter = updated_params.tf_static_topic
                self.node_.declare_parameter(self.prefix_ + "tf_static_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "scene_snapshot_topic"):
                descriptor = ParameterDescriptor(description=r"scene_snapshot_topic", read_only = False)
                parameter = updated_params.scene_snapshot_topic
                self.node_.declare_parameter(self.prefix_ + "scene_snapshot_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "job_topic"):
                descriptor = ParameterDescriptor(description=r"job_topic", read_only = False)
                parameter = updated_params.job_topic
                self.node_.declare_parameter(self.prefix_ + "job_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "metrics_topic"):
                descriptor = ParameterDescriptor(description=r"metrics_topic", read_only = False)
                parameter = updated_params.metrics_topic
                self.node_.declare_parameter(self.prefix_ + "metrics_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "event_buffer_size"):
                descriptor = ParameterDescriptor(description=r"event_buffer_size", read_only = False)
                parameter = updated_params.event_buffer_size
                self.node_.declare_parameter(self.prefix_ + "event_buffer_size", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "log_buffer_size"):
                descriptor = ParameterDescriptor(description=r"log_buffer_size", read_only = False)
                parameter = updated_params.log_buffer_size
                self.node_.declare_parameter(self.prefix_ + "log_buffer_size", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "metrics_period_s"):
                descriptor = ParameterDescriptor(description=r"metrics_period_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.metrics_period_s
                self.node_.declare_parameter(self.prefix_ + "metrics_period_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "metrics_process_patterns"):
                descriptor = ParameterDescriptor(description=r"metrics_process_patterns", read_only = False)
                parameter = updated_params.metrics_process_patterns
                self.node_.declare_parameter(self.prefix_ + "metrics_process_patterns", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.enabled"):
                descriptor = ParameterDescriptor(description=r"record.enabled", read_only = False)
                parameter = updated_params.record.enabled
                self.node_.declare_parameter(self.prefix_ + "record.enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.root_dir"):
                descriptor = ParameterDescriptor(description=r"record.root_dir", read_only = False)
                parameter = updated_params.record.root_dir
                self.node_.declare_parameter(self.prefix_ + "record.root_dir", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.save_images"):
                descriptor = ParameterDescriptor(description=r"record.save_images", read_only = False)
                parameter = updated_params.record.save_images
                self.node_.declare_parameter(self.prefix_ + "record.save_images", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.save_clouds"):
                descriptor = ParameterDescriptor(description=r"record.save_clouds", read_only = False)
                parameter = updated_params.record.save_clouds
                self.node_.declare_parameter(self.prefix_ + "record.save_clouds", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.rosout"):
                descriptor = ParameterDescriptor(description=r"record.rosout", read_only = False)
                parameter = updated_params.record.rosout
                self.node_.declare_parameter(self.prefix_ + "record.rosout", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.level"):
                descriptor = ParameterDescriptor(description=r"record.level", read_only = False)
                parameter = updated_params.record.level
                self.node_.declare_parameter(self.prefix_ + "record.level", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.max_total_bag_gb"):
                descriptor = ParameterDescriptor(description=r"record.max_total_bag_gb", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.record.max_total_bag_gb
                self.node_.declare_parameter(self.prefix_ + "record.max_total_bag_gb", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "record.queue_depth"):
                descriptor = ParameterDescriptor(description=r"record.queue_depth", read_only = False)
                parameter = updated_params.record.queue_depth
                self.node_.declare_parameter(self.prefix_ + "record.queue_depth", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.enabled"):
                descriptor = ParameterDescriptor(description=r"trajectory.enabled", read_only = False)
                parameter = updated_params.trajectory.enabled
                self.node_.declare_parameter(self.prefix_ + "trajectory.enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.base_frame"):
                descriptor = ParameterDescriptor(description=r"trajectory.base_frame", read_only = False)
                parameter = updated_params.trajectory.base_frame
                self.node_.declare_parameter(self.prefix_ + "trajectory.base_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.tip_frame"):
                descriptor = ParameterDescriptor(description=r"trajectory.tip_frame", read_only = False)
                parameter = updated_params.trajectory.tip_frame
                self.node_.declare_parameter(self.prefix_ + "trajectory.tip_frame", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.period_s"):
                descriptor = ParameterDescriptor(description=r"trajectory.period_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.trajectory.period_s
                self.node_.declare_parameter(self.prefix_ + "trajectory.period_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.min_step_m"):
                descriptor = ParameterDescriptor(description=r"trajectory.min_step_m", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.trajectory.min_step_m
                self.node_.declare_parameter(self.prefix_ + "trajectory.min_step_m", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "trajectory.max_points"):
                descriptor = ParameterDescriptor(description=r"trajectory.max_points", read_only = False)
                parameter = updated_params.trajectory.max_points
                self.node_.declare_parameter(self.prefix_ + "trajectory.max_points", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug_image_topic"):
                descriptor = ParameterDescriptor(description=r"debug_image_topic", read_only = False)
                parameter = updated_params.debug_image_topic
                self.node_.declare_parameter(self.prefix_ + "debug_image_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug_image_raw_topic"):
                descriptor = ParameterDescriptor(description=r"debug_image_raw_topic", read_only = False)
                parameter = updated_params.debug_image_raw_topic
                self.node_.declare_parameter(self.prefix_ + "debug_image_raw_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tsdf_cloud_topic"):
                descriptor = ParameterDescriptor(description=r"tsdf_cloud_topic", read_only = False)
                parameter = updated_params.tsdf_cloud_topic
                self.node_.declare_parameter(self.prefix_ + "tsdf_cloud_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tcp_path_topic"):
                descriptor = ParameterDescriptor(description=r"tcp_path_topic", read_only = False)
                parameter = updated_params.tcp_path_topic
                self.node_.declare_parameter(self.prefix_ + "tcp_path_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tcp_markers_topic"):
                descriptor = ParameterDescriptor(description=r"tcp_markers_topic", read_only = False)
                parameter = updated_params.tcp_markers_topic
                self.node_.declare_parameter(self.prefix_ + "tcp_markers_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.enabled"):
                descriptor = ParameterDescriptor(description=r"debug.enabled", read_only = False)
                parameter = updated_params.debug.enabled
                self.node_.declare_parameter(self.prefix_ + "debug.enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.motion_enabled"):
                descriptor = ParameterDescriptor(description=r"debug.motion_enabled", read_only = False)
                parameter = updated_params.debug.motion_enabled
                self.node_.declare_parameter(self.prefix_ + "debug.motion_enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.action_timeout_s"):
                descriptor = ParameterDescriptor(description=r"debug.action_timeout_s", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.debug.action_timeout_s
                self.node_.declare_parameter(self.prefix_ + "debug.action_timeout_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.audit_enabled"):
                descriptor = ParameterDescriptor(description=r"debug.audit_enabled", read_only = False)
                parameter = updated_params.debug.audit_enabled
                self.node_.declare_parameter(self.prefix_ + "debug.audit_enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.run_harvest_action"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.run_harvest_action", read_only = False)
                parameter = updated_params.debug.endpoints.run_harvest_action
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.run_harvest_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.control_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.control_service", read_only = False)
                parameter = updated_params.debug.endpoints.control_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.control_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.begin_scene_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.begin_scene_service", read_only = False)
                parameter = updated_params.debug.endpoints.begin_scene_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.begin_scene_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.survey_action"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.survey_action", read_only = False)
                parameter = updated_params.debug.endpoints.survey_action
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.survey_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.execute_action"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.execute_action", read_only = False)
                parameter = updated_params.debug.endpoints.execute_action
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.execute_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.build_action"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.build_action", read_only = False)
                parameter = updated_params.debug.endpoints.build_action
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.build_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.check_reachability_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.check_reachability_service", read_only = False)
                parameter = updated_params.debug.endpoints.check_reachability_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.check_reachability_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.photo_pose_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.photo_pose_service", read_only = False)
                parameter = updated_params.debug.endpoints.photo_pose_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.photo_pose_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.preview_approach_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.preview_approach_service", read_only = False)
                parameter = updated_params.debug.endpoints.preview_approach_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.preview_approach_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.preview_full_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.preview_full_service", read_only = False)
                parameter = updated_params.debug.endpoints.preview_full_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.preview_full_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.ack_recovery_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.ack_recovery_service", read_only = False)
                parameter = updated_params.debug.endpoints.ack_recovery_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.ack_recovery_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.arm_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.arm_service", read_only = False)
                parameter = updated_params.debug.endpoints.arm_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.arm_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.skill_cancel_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.skill_cancel_service", read_only = False)
                parameter = updated_params.debug.endpoints.skill_cancel_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.skill_cancel_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.recon_save_session_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.recon_save_session_service", read_only = False)
                parameter = updated_params.debug.endpoints.recon_save_session_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.recon_save_session_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.recon_reset_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.recon_reset_service", read_only = False)
                parameter = updated_params.debug.endpoints.recon_reset_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.recon_reset_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.recon_finalize_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.recon_finalize_service", read_only = False)
                parameter = updated_params.debug.endpoints.recon_finalize_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.recon_finalize_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.recon_query_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.recon_query_service", read_only = False)
                parameter = updated_params.debug.endpoints.recon_query_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.recon_query_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.manage_nodes_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.manage_nodes_service", read_only = False)
                parameter = updated_params.debug.endpoints.manage_nodes_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.manage_nodes_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "debug.endpoints.set_enables_service"):
                descriptor = ParameterDescriptor(description=r"debug.endpoints.set_enables_service", read_only = False)
                parameter = updated_params.debug.endpoints.set_enables_service
                self.node_.declare_parameter(self.prefix_ + "debug.endpoints.set_enables_service", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "host")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.host = param.value
            param = self.node_.get_parameter(self.prefix_ + "port")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.port = param.value
            param = self.node_.get_parameter(self.prefix_ + "startup_facts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.startup_facts = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_status_probe_enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.robot_status_probe_enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.period_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.period_s',param.value, 'Invalid value set during initialization for parameter selfcheck.period_s: ' + validation_result)
            updated_params.selfcheck.period_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.initial_settle_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.initial_settle_s',param.value, 'Invalid value set during initialization for parameter selfcheck.initial_settle_s: ' + validation_result)
            updated_params.selfcheck.initial_settle_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.initial_timeout_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.initial_timeout_s',param.value, 'Invalid value set during initialization for parameter selfcheck.initial_timeout_s: ' + validation_result)
            updated_params.selfcheck.initial_timeout_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.rate_window_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.rate_window_s',param.value, 'Invalid value set during initialization for parameter selfcheck.rate_window_s: ' + validation_result)
            updated_params.selfcheck.rate_window_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.camera_probe_enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.camera_probe_enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.camera_min_hz")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.camera_min_hz',param.value, 'Invalid value set during initialization for parameter selfcheck.camera_min_hz: ' + validation_result)
            updated_params.selfcheck.camera_min_hz = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.disk_min_gb")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.disk_min_gb',param.value, 'Invalid value set during initialization for parameter selfcheck.disk_min_gb: ' + validation_result)
            updated_params.selfcheck.disk_min_gb = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.expected_joints")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.expected_joints = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.expected_tcp_norm_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, -1.0)
            if validation_result:
                raise InvalidParameterValueException('selfcheck.expected_tcp_norm_m',param.value, 'Invalid value set during initialization for parameter selfcheck.expected_tcp_norm_m: ' + validation_result)
            updated_params.selfcheck.expected_tcp_norm_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.moveit_expected")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.moveit_expected = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.force_expected")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.force_expected = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.model_paths")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.model_paths = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.color_image_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.color_image_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "selfcheck.depth_image_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.selfcheck.depth_image_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "param_poll_period_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('param_poll_period_s',param.value, 'Invalid value set during initialization for parameter param_poll_period_s: ' + validation_result)
            updated_params.param_poll_period_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "target_observations_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.target_observations_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "harvest_state_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.harvest_state_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "reconstruction_status_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.reconstruction_status_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "reconstruction_diagnostics_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.reconstruction_diagnostics_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "reconstruction_diagnostics_debug_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.reconstruction_diagnostics_debug_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "grasp_decision_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.grasp_decision_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "refined_pose_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.refined_pose_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "refined_axis_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.refined_axis_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "refined_diagnostics_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.refined_diagnostics_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "manipulation_status_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.manipulation_status_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "grasp_hypothesis_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.grasp_hypothesis_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "task_executor_state_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.task_executor_state_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "task_executor_events_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.task_executor_events_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_status_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.robot_status_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "joint_states_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.joint_states_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "joint_status_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.joint_status_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_static_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_static_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "scene_snapshot_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.scene_snapshot_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "job_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.job_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "metrics_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.metrics_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "event_buffer_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.event_buffer_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "log_buffer_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.log_buffer_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "metrics_period_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('metrics_period_s',param.value, 'Invalid value set during initialization for parameter metrics_period_s: ' + validation_result)
            updated_params.metrics_period_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "metrics_process_patterns")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.metrics_process_patterns = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.root_dir")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.root_dir = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.save_images")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.save_images = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.save_clouds")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.save_clouds = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.rosout")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.rosout = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.level")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.level = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.max_total_bag_gb")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('record.max_total_bag_gb',param.value, 'Invalid value set during initialization for parameter record.max_total_bag_gb: ' + validation_result)
            updated_params.record.max_total_bag_gb = param.value
            param = self.node_.get_parameter(self.prefix_ + "record.queue_depth")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.record.queue_depth = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.trajectory.enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.base_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.trajectory.base_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.tip_frame")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.trajectory.tip_frame = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.period_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('trajectory.period_s',param.value, 'Invalid value set during initialization for parameter trajectory.period_s: ' + validation_result)
            updated_params.trajectory.period_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.min_step_m")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('trajectory.min_step_m',param.value, 'Invalid value set during initialization for parameter trajectory.min_step_m: ' + validation_result)
            updated_params.trajectory.min_step_m = param.value
            param = self.node_.get_parameter(self.prefix_ + "trajectory.max_points")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.trajectory.max_points = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug_image_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug_image_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug_image_raw_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug_image_raw_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tsdf_cloud_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tsdf_cloud_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tcp_path_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tcp_path_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tcp_markers_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tcp_markers_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.motion_enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.motion_enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.action_timeout_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('debug.action_timeout_s',param.value, 'Invalid value set during initialization for parameter debug.action_timeout_s: ' + validation_result)
            updated_params.debug.action_timeout_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.audit_enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.audit_enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.run_harvest_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.run_harvest_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.control_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.control_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.begin_scene_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.begin_scene_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.survey_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.survey_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.execute_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.execute_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.build_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.build_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.check_reachability_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.check_reachability_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.photo_pose_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.photo_pose_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.preview_approach_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.preview_approach_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.preview_full_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.preview_full_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.ack_recovery_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.ack_recovery_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.arm_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.arm_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.skill_cancel_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.skill_cancel_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.recon_save_session_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.recon_save_session_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.recon_reset_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.recon_reset_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.recon_finalize_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.recon_finalize_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.recon_query_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.recon_query_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.manage_nodes_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.manage_nodes_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "debug.endpoints.set_enables_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.debug.endpoints.set_enables_service = param.value


            self.update_internal_params(updated_params)

# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
import rclpy.parameter
from generate_parameter_library_py.python_validators import ParameterValidators



class executor_node:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        execution_enabled = False
        survey_wait_s = 8.0
        service_timeout_s = 30.0
        action_timeout_s = 180.0
        empty_survey_limit = 2
        persist_ledger = True
        reconstruction_min_views = 4
        build_start_timeout_s = 2.0
        observe_build_grace_s = 3.0
        begin_scene_service = "/peach_scene_perception_node/begin_scene"
        survey_scene_action = "/peach_manipulation_skills_node/survey_scene"
        execute_target_action = "/peach_manipulation_skills_node/execute_target"
        build_target_model_action = "/peach_target_reconstruction_node/build_target_model"
        require_managed_stack = False



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = executor_node.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("executor_node." + prefix)

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
                if param.name == self.prefix_ + "execution_enabled":
                    updated_params.execution_enabled = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "survey_wait_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.survey_wait_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "service_timeout_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.service_timeout_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "action_timeout_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.action_timeout_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "empty_survey_limit":
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.empty_survey_limit = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "persist_ledger":
                    updated_params.persist_ledger = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "reconstruction_min_views":
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.reconstruction_min_views = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "build_start_timeout_s":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.build_start_timeout_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "observe_build_grace_s":
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.observe_build_grace_s = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "begin_scene_service":
                    updated_params.begin_scene_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "survey_scene_action":
                    updated_params.survey_scene_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "execute_target_action":
                    updated_params.execute_target_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "build_target_model_action":
                    updated_params.build_target_model_action = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "require_managed_stack":
                    updated_params.require_managed_stack = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "execution_enabled"):
                descriptor = ParameterDescriptor(description=r"false 时 Survey 后直接结算，不选目标、不派 ExecuteTarget。", read_only = False)
                parameter = updated_params.execution_enabled
                self.node_.declare_parameter(self.prefix_ + "execution_enabled", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "survey_wait_s"):
                descriptor = ParameterDescriptor(description=r"SurveyScene 后等待目标集锁定的秒数。", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.survey_wait_s
                self.node_.declare_parameter(self.prefix_ + "survey_wait_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "service_timeout_s"):
                descriptor = ParameterDescriptor(description=r"BeginScene 等服务等待上限。", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.service_timeout_s
                self.node_.declare_parameter(self.prefix_ + "service_timeout_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "action_timeout_s"):
                descriptor = ParameterDescriptor(description=r"Survey/Build/Execute 动作等待上限。", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.action_timeout_s
                self.node_.declare_parameter(self.prefix_ + "action_timeout_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "empty_survey_limit"):
                descriptor = ParameterDescriptor(description=r"连续空扫次数上限，达到则结算批次。", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31-1
                parameter = updated_params.empty_survey_limit
                self.node_.declare_parameter(self.prefix_ + "empty_survey_limit", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "persist_ledger"):
                descriptor = ParameterDescriptor(description=r"是否把 TargetOutcome 写入 harvest_runs 账本。", read_only = False)
                parameter = updated_params.persist_ledger
                self.node_.declare_parameter(self.prefix_ + "persist_ledger", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "reconstruction_min_views"):
                descriptor = ParameterDescriptor(description=r"OBSERVE 成功后 Build 至少应有的视角数，不足则 observe_build_view_race。", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31-1
                parameter = updated_params.reconstruction_min_views
                self.node_.declare_parameter(self.prefix_ + "reconstruction_min_views", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "build_start_timeout_s"):
                descriptor = ParameterDescriptor(description=r"Build 目标绑定并进入 COLLECTING 的等待上限；超时禁止观察运动。", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.build_start_timeout_s
                self.node_.declare_parameter(self.prefix_ + "build_start_timeout_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "observe_build_grace_s"):
                descriptor = ParameterDescriptor(description=r"OBSERVE 结束后等待重建视角追上的宽限秒数。", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.observe_build_grace_s
                self.node_.declare_parameter(self.prefix_ + "observe_build_grace_s", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "begin_scene_service"):
                descriptor = ParameterDescriptor(description=r"BeginScene 服务名。", read_only = False)
                parameter = updated_params.begin_scene_service
                self.node_.declare_parameter(self.prefix_ + "begin_scene_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "survey_scene_action"):
                descriptor = ParameterDescriptor(description=r"SurveyScene 动作名。", read_only = False)
                parameter = updated_params.survey_scene_action
                self.node_.declare_parameter(self.prefix_ + "survey_scene_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "execute_target_action"):
                descriptor = ParameterDescriptor(description=r"ExecuteTarget 动作名。", read_only = False)
                parameter = updated_params.execute_target_action
                self.node_.declare_parameter(self.prefix_ + "execute_target_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "build_target_model_action"):
                descriptor = ParameterDescriptor(description=r"BuildTargetModel 动作名。", read_only = False)
                parameter = updated_params.build_target_model_action
                self.node_.declare_parameter(self.prefix_ + "build_target_model_action", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "require_managed_stack"):
                descriptor = ParameterDescriptor(description=r"true 时须等 /peach/lifecycle/managed_nodes_activated 才接受 RunHarvest。", read_only = False)
                parameter = updated_params.require_managed_stack
                self.node_.declare_parameter(self.prefix_ + "require_managed_stack", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "execution_enabled")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.execution_enabled = param.value
            param = self.node_.get_parameter(self.prefix_ + "survey_wait_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('survey_wait_s',param.value, 'Invalid value set during initialization for parameter survey_wait_s: ' + validation_result)
            updated_params.survey_wait_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "service_timeout_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('service_timeout_s',param.value, 'Invalid value set during initialization for parameter service_timeout_s: ' + validation_result)
            updated_params.service_timeout_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "action_timeout_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('action_timeout_s',param.value, 'Invalid value set during initialization for parameter action_timeout_s: ' + validation_result)
            updated_params.action_timeout_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "empty_survey_limit")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException('empty_survey_limit',param.value, 'Invalid value set during initialization for parameter empty_survey_limit: ' + validation_result)
            updated_params.empty_survey_limit = param.value
            param = self.node_.get_parameter(self.prefix_ + "persist_ledger")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.persist_ledger = param.value
            param = self.node_.get_parameter(self.prefix_ + "reconstruction_min_views")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException('reconstruction_min_views',param.value, 'Invalid value set during initialization for parameter reconstruction_min_views: ' + validation_result)
            updated_params.reconstruction_min_views = param.value
            param = self.node_.get_parameter(self.prefix_ + "build_start_timeout_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('build_start_timeout_s',param.value, 'Invalid value set during initialization for parameter build_start_timeout_s: ' + validation_result)
            updated_params.build_start_timeout_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "observe_build_grace_s")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('observe_build_grace_s',param.value, 'Invalid value set during initialization for parameter observe_build_grace_s: ' + validation_result)
            updated_params.observe_build_grace_s = param.value
            param = self.node_.get_parameter(self.prefix_ + "begin_scene_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.begin_scene_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "survey_scene_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.survey_scene_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "execute_target_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.execute_target_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "build_target_model_action")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.build_target_model_action = param.value
            param = self.node_.get_parameter(self.prefix_ + "require_managed_stack")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.require_managed_stack = param.value


            self.update_internal_params(updated_params)

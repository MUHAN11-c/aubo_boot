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



class peach_lifecycle_flag_bridge:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        is_active_service = "/peach_lifecycle_manager/is_active"
        poll_hz = 1.0



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = peach_lifecycle_flag_bridge.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("peach_lifecycle_flag_bridge." + prefix)

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
                if param.name == self.prefix_ + "is_active_service":
                    updated_params.is_active_service = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "poll_hz":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.poll_hz = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "is_active_service"):
                descriptor = ParameterDescriptor(description=r"Nav2 lifecycle manager is_active service name.", read_only = False)
                parameter = updated_params.is_active_service
                self.node_.declare_parameter(self.prefix_ + "is_active_service", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "poll_hz"):
                descriptor = ParameterDescriptor(description=r"Polling frequency in hertz.", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = (0.0 + sys.float_info.epsilon)
                descriptor.floating_point_range[-1].to_value = float('inf')
                parameter = updated_params.poll_hz
                self.node_.declare_parameter(self.prefix_ + "poll_hz", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "is_active_service")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.is_active_service = param.value
            param = self.node_.get_parameter(self.prefix_ + "poll_hz")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException('poll_hz',param.value, 'Invalid value set during initialization for parameter poll_hz: ' + validation_result)
            updated_params.poll_hz = param.value


            self.update_internal_params(updated_params)

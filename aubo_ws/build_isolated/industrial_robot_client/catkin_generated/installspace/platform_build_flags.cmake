
# industrial_robot_client doesn't have anything to add, but we must re-export
# the definitions from simple_message, so consumers of industrial_robot_client
# also get them.

set(industrial_robot_client_DEFINITIONS SIMPLE_MESSAGE_USE_ROS;SIMPLE_MESSAGE_LINUX)
set(industrial_robot_client_bswap_DEFINITIONS SIMPLE_MESSAGE_USE_ROS;SIMPLE_MESSAGE_LINUX;BYTE_SWAPPING)

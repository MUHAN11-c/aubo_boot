#include "peach2_end_effector/end_effector.hpp"
#include "peach2_end_effector/plugins.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(peach2_end_effector::ShearV1, peach2_end_effector::EndEffector)
PLUGINLIB_EXPORT_CLASS(peach2_end_effector::BiteShearV1, peach2_end_effector::EndEffector)
PLUGINLIB_EXPORT_CLASS(peach2_end_effector::AdaptiveShearV1, peach2_end_effector::EndEffector)

// Copyright 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file feedback_bridge_node.cpp
 * @brief 消息转发节点：将 aubo_msgs/msg/JointTrajectoryFeedback 转换为 
 *        control_msgs/action/FollowJointTrajectory_Feedback
 * 
 * 该节点用于解决 ros1_bridge 无法直接桥接以下类型对的问题：
 * - ROS 1: control_msgs/FollowJointTrajectoryFeedback (独立消息类型)
 * - ROS 2: control_msgs/action/FollowJointTrajectory_Feedback (action 反馈类型)
 * 
 * 解决方案：使用自定义消息 aubo_msgs/JointTrajectoryFeedback（格式相同）
 * 通过 ros1_bridge 桥接到 ROS 2，然后转换为 action 反馈类型
 * 
 * 工作流程：
 * 1. 订阅 ROS 1 桥接过来的 aubo_msgs/msg/JointTrajectoryFeedback 消息
 * 2. 将其转换为 control_msgs/action/FollowJointTrajectory_Feedback 类型
 * 3. 发布到 /feedback_states 话题供 ROS 2 节点使用
 */

#include <rclcpp/rclcpp.hpp>
#include <aubo_msgs/msg/joint_trajectory_feedback.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using namespace std::chrono_literals;

class FeedbackBridgeNode : public rclcpp::Node
{
public:
  FeedbackBridgeNode()
    : Node("feedback_bridge_node")
  {
    // 声明参数：输入话题名称（从 ROS 1 桥接过来的话题）
    this->declare_parameter<std::string>("input_topic", "feedback_states");
    // 声明参数：输出话题名称（ROS 2 节点期望的话题）
    this->declare_parameter<std::string>("output_topic", "feedback_states");
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    
    // 订阅 ROS 1 桥接过来的自定义消息类型
    // ros1_bridge 会将 aubo_msgs/JointTrajectoryFeedback 桥接为
    // aubo_msgs/msg/JointTrajectoryFeedback
    msg_feedback_sub_ = this->create_subscription<aubo_msgs::msg::JointTrajectoryFeedback>(
      input_topic,
      10,
      std::bind(&FeedbackBridgeNode::msgFeedbackCallback, this, std::placeholders::_1)
    );

    // 发布 action 反馈类型到输出话题（供 ROS 2 节点使用）
    action_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>(
      output_topic,
      10
    );

    RCLCPP_INFO(this->get_logger(), "Feedback bridge node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
  }

private:
  /**
   * @brief 消息反馈回调函数
   * 
   * 将 aubo_msgs/msg/JointTrajectoryFeedback 转换为
   * control_msgs/action/FollowJointTrajectory_Feedback 并发布
   */
  void msgFeedbackCallback(const aubo_msgs::msg::JointTrajectoryFeedback::ConstSharedPtr msg)
  {
    // 创建 action 反馈消息
    auto action_feedback = std::make_shared<control_msgs::action::FollowJointTrajectory_Feedback>();
    
    // 复制所有字段
    action_feedback->header = msg->header;
    action_feedback->joint_names = msg->joint_names;
    action_feedback->actual = msg->actual;
    action_feedback->desired = msg->desired;
    action_feedback->error = msg->error;
    
    // 发布转换后的消息
    action_feedback_pub_->publish(*action_feedback);
    
    RCLCPP_DEBUG(this->get_logger(), "Converted and published feedback for %zu joints", 
                 msg->joint_names.size());
  }

  rclcpp::Subscription<aubo_msgs::msg::JointTrajectoryFeedback>::SharedPtr msg_feedback_sub_;
  rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr action_feedback_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeedbackBridgeNode>());
  rclcpp::shutdown();
  return 0;
}


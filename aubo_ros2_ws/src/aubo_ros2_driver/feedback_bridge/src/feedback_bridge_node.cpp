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
 * @brief 消息转发节点：桥接 ROS1 和 ROS2 之间的消息
 * 
 * 功能1：将 aubo_msgs/msg/JointTrajectoryFeedback 转换为 
 *        control_msgs/action/FollowJointTrajectory_Feedback
 * 
 * 功能2：转发 joint_states 消息（从 ROS1 桥接话题到 ROS2 话题）
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
 * 4. 订阅 ROS 1 桥接过来的 joint_states 消息
 * 5. 转发到 ROS 2 的 joint_states 话题（可配置话题名称）
 */

#include <rclcpp/rclcpp.hpp>
#include <aubo_msgs/msg/joint_trajectory_feedback.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class FeedbackBridgeNode : public rclcpp::Node
{
public:
  FeedbackBridgeNode()
    : Node("feedback_bridge_node")
  {
    // ========== Feedback 消息桥接参数 ==========
    // 声明参数：输入话题名称（从 ROS 1 桥接过来的话题）
    this->declare_parameter<std::string>("feedback_input_topic", "feedback_states");
    // 声明参数：输出话题名称（ROS 2 节点期望的话题）
    this->declare_parameter<std::string>("feedback_output_topic", "feedback_states");
    
    std::string feedback_input_topic = this->get_parameter("feedback_input_topic").as_string();
    std::string feedback_output_topic = this->get_parameter("feedback_output_topic").as_string();
    
    // ========== Joint States 消息桥接参数 ==========
    // 声明参数：joint_states 输入话题（从 ROS 1 桥接过来的话题）
    // 默认使用 ROS1 侧改名后的 joint_states_ros1（通过 ros1_bridge 桥接到 ROS2）
    this->declare_parameter<std::string>("joint_states_input_topic", "joint_states_ros1");
    // 声明参数：joint_states 输出话题（ROS 2 节点期望的话题）
    this->declare_parameter<std::string>("joint_states_output_topic", "joint_states");
    // 声明参数：是否启用 joint_states 桥接
    this->declare_parameter<bool>("enable_joint_states_bridge", true);
    
    std::string joint_states_input_topic = this->get_parameter("joint_states_input_topic").as_string();
    std::string joint_states_output_topic = this->get_parameter("joint_states_output_topic").as_string();
    bool enable_joint_states = this->get_parameter("enable_joint_states_bridge").as_bool();

    // 防止 joint_states 自回环：输入/输出同名会导致订阅到自己发布的消息并无限转发
    if (enable_joint_states && joint_states_input_topic == joint_states_output_topic) {
      RCLCPP_WARN(this->get_logger(),
                  "joint_states bridge disabled to prevent loop: input_topic == output_topic == '%s'. "
                  "Please set different 'joint_states_input_topic'/'joint_states_output_topic' or disable the bridge.",
                  joint_states_input_topic.c_str());
      enable_joint_states = false;
    }
    
    // ========== Feedback 消息订阅和发布 ==========
    // 订阅 ROS 1 桥接过来的自定义消息类型
    // ros1_bridge 会将 aubo_msgs/JointTrajectoryFeedback 桥接为
    // aubo_msgs/msg/JointTrajectoryFeedback
    msg_feedback_sub_ = this->create_subscription<aubo_msgs::msg::JointTrajectoryFeedback>(
      feedback_input_topic,
      10,
      std::bind(&FeedbackBridgeNode::msgFeedbackCallback, this, std::placeholders::_1)
    );

    // 发布 action 反馈类型到输出话题（供 ROS 2 节点使用）
    action_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>(
      feedback_output_topic,
      10
    );

    // ========== Joint States 消息订阅和发布 ==========
    if (enable_joint_states) {
      // 订阅 ROS 1 桥接过来的 joint_states 消息
      // ros1_bridge 会将 sensor_msgs/JointState 桥接为 sensor_msgs/msg/JointState
      joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_input_topic,
        10,
        std::bind(&FeedbackBridgeNode::jointStatesCallback, this, std::placeholders::_1)
      );

      // 发布 joint_states 到输出话题（供 ROS 2 节点使用）
      joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_states_output_topic,
        10
      );

      RCLCPP_INFO(this->get_logger(), "Joint states bridge enabled");
      RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", joint_states_input_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", joint_states_output_topic.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Joint states bridge disabled");
    }

    RCLCPP_INFO(this->get_logger(), "Feedback bridge node started");
    RCLCPP_INFO(this->get_logger(), "Feedback bridge:");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", feedback_input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", feedback_output_topic.c_str());
  }

private:
  /**
   * @brief 消息反馈回调函数
   * 
   * 将 aubo_msgs/msg/JointTrajectoryFeedback 转换为
   * control_msgs/action/FollowJointTrajectory_Feedback 并发布
   * 时间戳会被更新为ROS2的当前时间，避免时钟不同步问题
   */
  void msgFeedbackCallback(const aubo_msgs::msg::JointTrajectoryFeedback::ConstSharedPtr msg)
  {
    // 创建 action 反馈消息
    auto action_feedback = std::make_shared<control_msgs::action::FollowJointTrajectory_Feedback>();
    
    // 复制消息内容
    action_feedback->joint_names = msg->joint_names;
    action_feedback->actual = msg->actual;
    action_feedback->desired = msg->desired;
    action_feedback->error = msg->error;
    
    // ✨ 用ROS2当前时间覆盖时间戳
    action_feedback->header.stamp = this->now();
    action_feedback->header.frame_id = msg->header.frame_id;
    
    // 发布转换后的消息
    action_feedback_pub_->publish(*action_feedback);
    
    RCLCPP_DEBUG(this->get_logger(), "Converted and published feedback for %zu joints", 
                 msg->joint_names.size());
  }

  /**
   * @brief Joint States 回调函数
   * 
   * 接收从 ROS1 桥接过来的 joint_states 消息，并转发到 ROS2 话题
   * 时间戳会被更新为ROS2的当前时间，避免时钟不同步问题
   */
  void jointStatesCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    // 创建新的 joint_states 消息
    auto joint_states_msg = std::make_shared<sensor_msgs::msg::JointState>();
    
    // 复制消息内容
    joint_states_msg->name = msg->name;
    joint_states_msg->position = msg->position;
    joint_states_msg->velocity = msg->velocity;
    joint_states_msg->effort = msg->effort;
    
    // ✨ 关键修改：用ROS2当前时间覆盖时间戳，解决ROS1/ROS2时钟不同步问题
    joint_states_msg->header.stamp = this->now();
    joint_states_msg->header.frame_id = msg->header.frame_id;  // 保留原始frame_id
    
    // 发布转发后的消息
    joint_states_pub_->publish(*joint_states_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Forwarded joint_states with %zu joints (timestamp updated to ROS2 time)", 
                 msg->name.size());
  }

  // Feedback 消息订阅和发布
  rclcpp::Subscription<aubo_msgs::msg::JointTrajectoryFeedback>::SharedPtr msg_feedback_sub_;
  rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr action_feedback_pub_;
  
  // Joint States 消息订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeedbackBridgeNode>());
  rclcpp::shutdown();
  return 0;
}


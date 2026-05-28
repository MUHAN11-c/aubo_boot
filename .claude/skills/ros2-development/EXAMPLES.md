# ROS 2 Progressive Examples

All examples in both C++ and Python. Each is a self-contained, buildable
package following the [fishros/ros2bookcode](https://github.com/fishros/ros2bookcode)
progressive structure.

## 1. Hello World

Minimal node that prints a message.

**C++** — `hello_node.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hello_node");
    RCLCPP_INFO(node->get_logger(), "Hello ROS 2!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.8)
project(hello_cpp)
find_package(rclcpp REQUIRED)
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)
install(TARGETS hello_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

**Python** — `hello_node.py`
```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("hello_node")
    node.get_logger().info("Hello ROS 2!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
setup.py entry_points:
```python
entry_points={"console_scripts": ["hello_node = hello_pkg.hello_node:main"]}
```

Build & run:
```bash
colcon build --packages-select hello_cpp hello_pkg
source install/setup.bash
ros2 run hello_cpp hello_node
ros2 run hello_pkg hello_node
```

---

## 2. OOP Node

Inherit from Node, use member variables. Demonstrates the pattern used in
real ROS 2 projects.

**C++** — `person_node.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include <string>

class PersonNode : public rclcpp::Node {
public:
    PersonNode(const std::string &name, int age)
        : Node("person_node"), name_(name), age_(age) {
        timer_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&PersonNode::onTimer, this));
    }

private:
    void onTimer() {
        RCLCPP_INFO(get_logger(), "I'm %s, %d years old", name_.c_str(), age_);
    }

    std::string name_;
    int age_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("Tom", 25);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**Python** — `person_node.py`
```python
import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, name: str, age: int):
        super().__init__("person_node")
        self.name = name
        self.age = age
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        self.get_logger().info(f"I'm {self.name}, {self.age} years old")

def main():
    rclpy.init()
    node = PersonNode("Tom", 25)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 3. Topic — Publisher + Subscriber

Practical example: a novel reader publishes paragraphs, a listener counts words.

**Custom message** — `msg/NovelChunk.msg`
```
string content
int32 chunk_number
```

**C++ Publisher** — `novel_pub.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include "topic_demo/msg/novel_chunk.hpp"
#include <fstream>
#include <sstream>
#include <thread>

class NovelPublisher : public rclcpp::Node {
public:
    NovelPublisher() : Node("novel_publisher") {
        pub_ = create_publisher<topic_demo::msg::NovelChunk>("novel", 10);
        timer_ = create_wall_timer(std::chrono::seconds(2),
            std::bind(&NovelPublisher::publishNext, this));
        loadNovel();
    }

private:
    void loadNovel() {
        // In practice: read from file. Here we use hardcoded paragraphs.
        paragraphs_ = {
            "ROS 2 is a distributed communication framework.",
            "It uses DDS as its middleware layer.",
            "Nodes communicate via topics, services, and actions."};
    }

    void publishNext() {
        if (chunk_num_ >= paragraphs_.size()) {
            RCLCPP_INFO(get_logger(), "Novel finished. Closing.");
            rclcpp::shutdown();
            return;
        }
        auto msg = topic_demo::msg::NovelChunk();
        msg.chunk_number = chunk_num_;
        msg.content = paragraphs_[chunk_num_++];
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published chunk %d", msg.chunk_number);
    }

    rclcpp::Publisher<topic_demo::msg::NovelChunk>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> paragraphs_;
    size_t chunk_num_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NovelPublisher>());
    rclcpp::shutdown();
}
```

**Python Subscriber** — `novel_sub.py`
```python
import rclpy
from rclpy.node import Node
from topic_demo.msg import NovelChunk

class NovelSubscriber(Node):
    def __init__(self):
        super().__init__("novel_subscriber")
        self.sub = self.create_subscription(
            NovelChunk, "novel", self.on_chunk, 10)
        self.word_count = 0

    def on_chunk(self, msg: NovelChunk):
        words = len(msg.content.split())
        self.word_count += words
        self.get_logger().info(
            f"Chunk {msg.chunk_number}: {words} words (total: {self.word_count})")

def main():
    rclpy.init()
    rclpy.spin(NovelSubscriber())
    rclpy.shutdown()
```

---

## 4. Custom Interfaces

Define `.msg`, `.srv`, `.action` files in an interfaces-only package.

```
my_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── SystemStatus.msg    # float32 cpu_usage, float32 memory_usage, string status
├── srv/
│   └── Calibrate.srv       # string sensor_name --- bool success, string message
└── action/
    └── Patrol.action       # float32 radius, int32 laps --- int32 completed_laps --- float32 progress
```

**CMakeLists.txt** (interfaces package):
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SystemStatus.msg"
    "srv/Calibrate.srv"
    "action/Patrol.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

**package.xml**:
```xml
<package format="3">
    <name>my_interfaces</name>
    <version>0.1.0</version>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

---

## 5. Service — Server + Client

Practical example: face detection service. Client sends an image path, server
returns detection results.

**Service definition** — `srv/FaceDetect.srv`:
```
string image_path
---
bool detected
int32 face_count
float32 confidence
```

**C++ Server** — `face_detect_server.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include "face_demo/srv/face_detect.hpp"

class FaceDetectServer : public rclcpp::Node {
public:
    FaceDetectServer() : Node("face_detect_server") {
        srv_ = create_service<face_demo::srv::FaceDetect>(
            "face_detect",
            std::bind(&FaceDetectServer::onRequest, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

private:
    void onRequest(const face_demo::srv::FaceDetect::Request::SharedPtr req,
                   face_demo::srv::FaceDetect::Response::SharedPtr res) {
        RCLCPP_INFO(get_logger(), "Processing: %s", req->image_path.c_str());
        // In practice: run OpenCV face detection here
        res->detected = true;
        res->face_count = 3;
        res->confidence = 0.92;
    }

    rclcpp::Service<face_demo::srv::FaceDetect>::SharedPtr srv_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceDetectServer>());
    rclcpp::shutdown();
}
```

**Python Client** — `face_detect_client.py`
```python
import rclpy
from rclpy.node import Node
from face_demo.srv import FaceDetect

class FaceDetectClient(Node):
    def __init__(self):
        super().__init__("face_detect_client")
        self.cli = self.create_client(FaceDetect, "face_detect")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.send_request("/tmp/photo.jpg")

    def send_request(self, path: str):
        req = FaceDetect.Request()
        req.image_path = path
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(
                f"Detected: {res.detected}, faces: {res.face_count}, "
                f"confidence: {res.confidence:.2f}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = FaceDetectClient()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## 6. Parameters + Launch Files

Practical example: robot monitoring node with configurable thresholds, launched
with YAML parameters.

**YAML config** — `config/monitor_params.yaml`:
```yaml
/monitor_node:
  ros__parameters:
    cpu_warn_threshold: 80.0     # percent
    memory_warn_threshold: 90.0  # percent
    publish_rate: 1.0            # Hz
    robot_name: "fishbot"
    enabled_sensors: ["camera", "lidar", "imu"]
```

**C++ Node** — `monitor_node.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>

class MonitorNode : public rclcpp::Node {
public:
    MonitorNode() : Node("monitor_node") {
        // Guard auto-declare from launch
        if (!has_parameter("cpu_warn_threshold"))
            declare_parameter("cpu_warn_threshold", 90.0);
        if (!has_parameter("memory_warn_threshold"))
            declare_parameter("memory_warn_threshold", 95.0);
        if (!has_parameter("publish_rate"))
            declare_parameter("publish_rate", 1.0);

        double rate;
        get_parameter("publish_rate", rate);
        int ms = static_cast<int>(1000.0 / rate);

        timer_ = create_wall_timer(std::chrono::milliseconds(ms),
            std::bind(&MonitorNode::onTimer, this));

        // React to parameter changes at runtime
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                for (auto &p : params) {
                    if (p.get_name() == "publish_rate") {
                        timer_->cancel();
                        int ms = static_cast<int>(1000.0 / p.as_double());
                        timer_ = create_wall_timer(std::chrono::milliseconds(ms),
                            std::bind(&MonitorNode::onTimer, this));
                        RCLCPP_INFO(get_logger(), "Rate updated: %.1f Hz", p.as_double());
                    }
                }
                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                return res;
            });
    }

private:
    void onTimer() {
        double cpu, mem;
        get_parameter("cpu_warn_threshold", cpu);
        get_parameter("memory_warn_threshold", mem);
        RCLCPP_INFO(get_logger(), "Monitor: cpu<%.0f%%, mem<%.0f%%", cpu, mem);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
```

**Python launch file** — `launch/monitor.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="fishbot"),
        Node(
            package="monitor_pkg",
            executable="monitor_node",
            name="monitor_node",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("monitor_pkg"), "config", "monitor_params.yaml"
                ]),
                {"robot_name": LaunchConfiguration("robot_name")}
            ],
            output="screen"
        )
    ])
```

CLI usage:
```bash
ros2 param list /monitor_node
ros2 param get /monitor_node cpu_warn_threshold
ros2 param set /monitor_node publish_rate 2.0
ros2 param dump /monitor_node  # export current params

# Set another node's parameters programmatically (C++):
# auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "target_node");
# client->set_parameters({{"param", 42}});
```

---

## 7. TF2 — Broadcaster + Listener

Practical example: a moving robot broadcasts its pose, a follower listens and
maintains a relative offset.

**C++ Broadcaster** — `dynamic_tf_broadcaster.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DynamicTFBroadcaster : public rclcpp::Node {
public:
    DynamicTFBroadcaster() : Node("tf_broadcaster") {
        tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&DynamicTFBroadcaster::onTimer, this));
    }

private:
    void onTimer() {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "robot_base";
        ts.transform.translation.x = std::sin(count_ * 0.1);
        ts.transform.translation.y = std::cos(count_ * 0.1);
        ts.transform.translation.z = 0.0;
        ts.transform.rotation.w = 1.0;
        tf_bc_->sendTransform(ts);
        count_++;
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;
};
```

**Python Listener** — `tf_listener.py`
```python
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

class TFListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        try:
            ts = self.tf_buffer.lookup_transform(
                "world", "robot_base", rclpy.time.Time())
            self.get_logger().info(
                f"Robot at ({ts.transform.translation.x:.2f}, "
                f"{ts.transform.translation.y:.2f})")
        except TransformException as e:
            self.get_logger().warn(f"TF not ready: {e}")
```

---

## 8. Action — Server + Client

Practical example: patrol action — robot circles at given radius, reporting
progress and supporting cancellation.

**C++ Server** — `patrol_server.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "patrol_interfaces/action/patrol.hpp"
#include <thread>
#include <cmath>

using Patrol = patrol_interfaces::action::Patrol;
using GoalHandle = rclcpp_action::ServerGoalHandle<Patrol>;

class PatrolServer : public rclcpp::Node {
public:
    PatrolServer() : Node("patrol_server") {
        server_ = rclcpp_action::create_server<Patrol>(
            this, "patrol",
            // Goal callback
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Patrol::Goal> goal) {
                if (goal->radius <= 0 || goal->laps <= 0)
                    return rclcpp_action::GoalResponse::REJECT;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // Cancel callback
            [](std::shared_ptr<GoalHandle> handle) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            // Execute callback
            [this](std::shared_ptr<GoalHandle> handle) {
                execute(handle);
            });
    }

private:
    void execute(std::shared_ptr<GoalHandle> handle) {
        auto goal = handle->get_goal();
        auto feedback = std::make_shared<Patrol::Feedback>();
        auto result = std::make_shared<Patrol::Result>();
        int total_steps = goal->laps * 36;  // 10° per step
        double radius = goal->radius;

        for (int i = 0; i <= total_steps; i++) {
            if (handle->is_canceling()) {
                result->completed_laps = i / 36;
                handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Patrol canceled");
                return;
            }
            feedback->progress = (float)i / total_steps;
            handle->publish_feedback(feedback);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        result->completed_laps = goal->laps;
        handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Patrol complete: %d laps", goal->laps);
    }

    rclcpp_action::Server<Patrol>::SharedPtr server_;
};
```

**Python Client** — `patrol_client.py`
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from patrol_interfaces.action import Patrol

class PatrolClient(Node):
    def __init__(self):
        super().__init__("patrol_client")
        self.cli = ActionClient(self, Patrol, "patrol")
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        self.timer.cancel()
        if not self.cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return

        goal = Patrol.Goal()
        goal.radius = 2.0
        goal.laps = 3

        self.get_logger().info("Sending patrol goal...")
        future = self.cli.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        self.get_logger().info("Goal accepted, patrolling...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        pct = feedback_msg.feedback.progress * 100
        self.get_logger().info(f"Progress: {pct:.0f}%")

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"Done! {result.completed_laps} laps completed")
        rclpy.shutdown()
```

---

## 9. Lifecycle Node

Practical example: camera driver that manages hardware resource lifecycle.
Services only respond in Active state, sensors only stream when activated.

**C++ Lifecycle Node** — `camera_lifecycle.cpp`
```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    CameraLifecycleNode() : LifecycleNode("camera_driver") {}

    CallbackReturn on_configure(const State &) override {
        // Create publishers (work in Inactive too)
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        RCLCPP_INFO(get_logger(), "Configured: camera ready");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State &) override {
        // Lightweight: start streaming
        image_pub_->on_activate();
        timer_ = create_wall_timer(std::chrono::milliseconds(33),  // 30Hz
            std::bind(&CameraLifecycleNode::captureAndPublish, this));
        RCLCPP_INFO(get_logger(), "Activated: streaming at 30Hz");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State &) override {
        timer_.reset();
        image_pub_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Deactivated: streaming stopped");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const State &) override {
        image_pub_.reset();
        RCLCPP_INFO(get_logger(), "Cleaned up: resources released");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const State &) override {
        RCLCPP_INFO(get_logger(), "Shutdown");
        return CallbackReturn::SUCCESS;
    }

private:
    void captureAndPublish() { /* capture frame, publish */ }

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

**Python Lifecycle Node** — `camera_lifecycle.py`
```python
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image

class CameraLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("camera_driver")

    def on_configure(self, state):
        self.image_pub = self.create_lifecycle_publisher(Image, "image_raw", 10)
        self.get_logger().info("Configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.timer = self.create_timer(0.033, self.capture)
        self.get_logger().info("Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.destroy_timer(self.timer)
        self.get_logger().info("Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.destroy_publisher(self.image_pub)
        return TransitionCallbackReturn.SUCCESS

    def capture(self):
        pass  # capture + publish
```

Manage lifecycle:
```bash
ros2 lifecycle set /camera_driver configure
ros2 lifecycle set /camera_driver activate
ros2 lifecycle set /camera_driver deactivate
ros2 lifecycle list /camera_driver
```

---

## 10. Advanced Topics

### QoS Reliability Test

**C++** — publisher with configurable QoS + subscriber measuring loss
```cpp
// Publisher: publish 1000 messages at 100Hz
auto qos = rclcpp::QoS(10);
qos.best_effort();  // or .reliable()

auto pub = node->create_publisher<std_msgs::msg::Int32>("test_topic", qos);
int sent = 0;
auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
    auto msg = std_msgs::msg::Int32();
    msg.data = sent++;
    pub->publish(msg);
});

// Subscriber: count received messages, compare with sequence numbers
int received = 0, gaps = 0;
auto sub = node->create_subscription<std_msgs::msg::Int32>(
    "test_topic", qos,
    [&](std_msgs::msg::Int32::SharedPtr msg) {
        received++;
        // Detect gaps from sequence number
    });
```

### Intra-process Communication (zero-copy)
```cpp
// Talker
auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
auto talker = std::make_shared<rclcpp::Node>("talker", options);
auto pub = talker->create_publisher<Msg>("topic", 10);

// Publish with unique_ptr for zero-copy transfer
auto msg = std::make_unique<Msg>();
msg->data = 42;
pub->publish(std::move(msg));

// Listener — same process
auto listener = std::make_shared<rclcpp::Node>("listener", options);
auto sub = listener->create_subscription<Msg>("topic", 10, callback);
// Callback receives const Msg& (not SharedPtr) when intra-process
```

### Node Composition
```cpp
// talker.hpp — component header
#include <rclcpp/rclcpp.hpp>

namespace my_components {
class Talker : public rclcpp::Node {
public:
    explicit Talker(const rclcpp::NodeOptions &opts) : Node("talker", opts) {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
} // namespace my_components

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::Talker)
```

```bash
# Standalone container (configurable at runtime)
ros2 run rclcpp_components component_container

# Load component
ros2 component load /ComponentManager my_pkg my_components::Talker

# Or all-in-one process
ros2 component standalone my_pkg my_components::Talker
```

### MultiThreadedExecutor + Reentrant Callback Group
```cpp
auto node = std::make_shared<rclcpp::Node>("parallel_node");
auto reentrant = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

// Service that can handle concurrent calls
rclcpp::SubscriptionOptions sub_opts;
sub_opts.callback_group = reentrant;

// All service/subscription callbacks in this group run concurrently
// across the executor's threads
rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4);  // 4 threads
exec.add_node(node);
exec.spin();
```

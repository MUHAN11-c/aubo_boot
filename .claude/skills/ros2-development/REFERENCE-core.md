# ROS 2 Core Concepts

## Topics

### Publisher (C++)
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

auto pub = node->create_publisher<std_msgs::msg::String>(
    "topic_name", rclcpp::QoS(10).reliable().transient_local());

// Publish
auto msg = std_msgs::msg::String();
msg.data = "hello";
pub->publish(msg);
```

### Publisher (Python)
```python
from std_msgs.msg import String

self.pub = self.create_publisher(String, "topic_name", 10)
self.pub.publish(String(data="hello"))
```

### Subscriber (C++)
```cpp
auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic_name", 10,
    [](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("sub"), "%s", msg->data.c_str());
    });
```

### Subscriber (Python)
```python
self.sub = self.create_subscription(String, "topic_name", self.callback, 10)

def callback(self, msg):
    self.get_logger().info(f"Received: {msg.data}")
```

### Intra-process communication
```cpp
// Must be in same process, same context
auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
// Publisher must use SharedPtr publish variant for zero-copy
auto msg = std::make_unique<Msg>();
pub->publish(std::move(msg));
```

### Loaned messages (zero-copy, C++ only)
```cpp
// Publisher borrows memory from middleware
auto loaned = pub->borrow_loaned_message();
loaned.get().field = value;
pub->publish(std::move(loaned));
```

### Topic statistics
```cpp
auto options = rclcpp::SubscriptionOptions();
options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
options.topic_stats_options.publish_period = std::chrono::seconds(1);
// Publishes statistics to /statistics topic
```

---

## Services

### Server (C++)
```cpp
#include <example_interfaces/srv/add_two_ints.hpp>

auto srv = node->create_service<example_interfaces::srv::AddTwoInts>(
    "add_two_ints",
    [](const Request::SharedPtr req, Response::SharedPtr res) {
        res->sum = req->a + req->b;
    });
```

### Client — async (C++)
```cpp
auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

// Wait for service (with timeout)
if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger, "Service not available");
    return;
}

auto req = std::make_shared<Request>();
req->a = 2; req->b = 3;

auto future = client->async_send_request(req);
// Process result in callback or spin
if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
    RCLCPP_INFO(logger, "Sum: %ld", future.get()->sum);
}
```

### Client — callback-based (C++)
```cpp
// SAFE: uses callback instead of future.wait_for()
auto future = client->async_send_request(req,
    [](rclcpp::Client<AddTwoInts>::SharedFuture future) {
        RCLCPP_INFO(logger, "Sum: %ld", future.get()->sum);
    });
```

### Server (Python)
```python
from example_interfaces.srv import AddTwoInts

self.srv = self.create_service(AddTwoInts, "add_two_ints", self.add_cb)

def add_cb(self, request, response):
    response.sum = request.a + request.b
    return response
```

### Client (Python)
```python
from example_interfaces.srv import AddTwoInts

cli = self.create_client(AddTwoInts, "add_two_ints")
if not cli.wait_for_service(timeout_sec=5.0):
    self.get_logger().error("Service not available")
    return

future = cli.call_async(AddTwoInts.Request(a=2, b=3))
# rclpy.spin_until_future_complete(self, future) or use callback
```

### Critical: Callback group deadlock
```
In a MutuallyExclusive callback group callback:
  async_send_request() → future.wait_for() → DEADLOCK

Why: wait_for() blocks the callback, but the service response handler
runs in the SAME MutuallyExclusive group. Can't execute → deadlock.

Fix: Put client + server in separate MutuallyExclusive groups, OR
use Reentrant group, OR use callback-based async_send_request.
```
See [REFERENCE-patterns.md](REFERENCE-patterns.md) for details.

---

## Parameters

### Declare (C++)
```cpp
// Simple declare with default
node->declare_parameter<int>("my_param", 42);
node->declare_parameter<std::string>("name", "default");

// With descriptor (constraints)
auto desc = rcl_interfaces::msg::ParameterDescriptor{};
desc.description = "Speed scaling factor between 0 and 1";
auto range = rcl_interfaces::msg::FloatingPointRange{};
range.set__from_value(0.0).set__to_value(1.0).set__step(0.01);
desc.floating_point_range = {range};

node->declare_parameter("speed_factor", 1.0, desc);

// Guard against auto-declare from launch
if (!node->has_parameter("param")) {
    node->declare_parameter("param", default_val);
}
```

### Get/Set (C++)
```cpp
// Get
int val;
node->get_parameter("my_param", val);                // copy out
auto param = node->get_parameter("my_param");         // Parameter object
int v = node->get_parameter("my_param").as_int();     // fluent

// Set
node->set_parameter(rclcpp::Parameter("my_param", 99));
// Set multiple (non-atomic: partial on failure)
node->set_parameters({p1, p2, p3});
// Atomic set (all-or-nothing)
node->set_parameters_atomically({p1, p2, p3});
```

### Parameter callback (C++)
```cpp
auto cb = [](const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    for (auto& p : params) {
        if (p.get_name() == "speed" && p.as_double() < 0) {
            res.successful = false;
            res.reason = "speed must be >= 0";
        }
    }
    return res;
};
node->add_on_set_parameters_callback(cb);
```

### Parameter callback (Python)
```python
def param_cb(self, params):
    for p in params:
        if p.name == "speed" and p.value < 0:
            return SetParametersResult(successful=False, reason="speed >= 0")
    return SetParametersResult(successful=True)

self.add_on_set_parameters_callback(param_cb)
```

### Set parameters on another node
```cpp
#include <rclcpp_components/component_manager.hpp>
// In C++, use AsyncParametersClient:
auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "target_node");
client->set_parameters({{"param", 42}});
// Or through component_manager:
// ros2 param set /target_node param 42
```

### YAML parameter file structure
```yaml
# ROS 2: parameters are namespaced by node full name
/node_full_name:
  ros__parameters:
    param1: 42
    param2: "hello"
    nested:
      key: value
```

---

## Actions

### Server (C++)
```cpp
#include <rclcpp_action/rclcpp_action.hpp>
#include <example_interfaces/action/fibonacci.hpp>

using Fibonacci = example_interfaces::action::Fibonacci;

auto server = rclcpp_action::create_server<Fibonacci>(
    node, "fibonacci",
    // Goal callback
    [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Fibonacci::Goal> goal) {
        if (goal->order <= 0) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // Cancel callback
    [](std::shared_ptr<GoalHandleFibonacci> handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
    },
    // Accepted callback (start execution)
    [](std::shared_ptr<GoalHandleFibonacci> handle) {
        // Execute in a separate thread
        std::thread{[handle]() {
            auto feedback = std::make_shared<Fibonacci::Feedback>();
            auto result = std::make_shared<Fibonacci::Result>();
            // ... compute, send feedback, check cancellation ...
            for (int i = 0; i < handle->get_goal()->order; i++) {
                if (handle->is_canceling()) {
                    handle->canceled(result);
                    return;
                }
                feedback->partial_sequence.push_back(i);
                handle->publish_feedback(feedback);
            }
            handle->succeed(result);
        }}.detach();
    });
```

### Client (C++)
```cpp
auto client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    return; // not available
}

auto goal = Fibonacci::Goal();
goal.order = 10;

auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
options.goal_response_callback = [](auto future) {
    if (future.get() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) { /* ... */ }
};
options.feedback_callback = [](auto, auto fb) {
    RCLCPP_INFO(logger, "Feedback: %zu", fb->partial_sequence.size());
};
options.result_callback = [](auto future) {
    RCLCPP_INFO(logger, "Result: ...");
};

client->async_send_goal(goal, options);
```

### Action (Python)
```python
from example_interfaces.action import Fibonacci

# Server
self.action_srv = ActionServer(self, Fibonacci, "fibonacci",
    execute_callback=self.execute, goal_callback=self.goal_cb)

# Client
self.action_cli = ActionClient(self, Fibonacci, "fibonacci")
goal = Fibonacci.Goal(order=10)
future = self.action_cli.send_goal_async(goal, feedback_callback=self.fb_cb)
```

---

## Plugins (pluginlib)

### Define a plugin interface (base class)
```cpp
// polygon_base.h
namespace polygon_base {
class RegularPolygon {
public:
    virtual ~RegularPolygon() = default;
    virtual void initialize(double side_length) = 0;
    virtual double area() = 0;
};
} // namespace polygon_base
```

### Implement a plugin
```cpp
// square_plugin.cpp
#include <pluginlib/class_list_macros.hpp>
#include "polygon_base.h"

class Square : public polygon_base::RegularPolygon {
public:
    void initialize(double side_length) override { side_length_ = side_length; }
    double area() override { return side_length_ * side_length_; }
private:
    double side_length_;
};
PLUGINLIB_EXPORT_CLASS(Square, polygon_base::RegularPolygon)
```

### Plugin XML descriptor
```xml
<!-- plugins.xml -->
<library path="square_plugin">
    <class type="Square" base_class_type="polygon_base::RegularPolygon">
        <description>Square plugin</description>
    </class>
</library>
```

### Load plugins
```cpp
#include <pluginlib/class_loader.hpp>

auto loader = pluginlib::ClassLoader<polygon_base::RegularPolygon>(
    "polygon_base", "polygon_base::RegularPolygon");

auto plugin = loader.createSharedInstance("Square");
plugin->initialize(5.0);
double area = plugin->area();

// List available plugins
for (auto& name : loader.getDeclaredClasses()) { /* ... */ }
```

### Plugin XML in package.xml
```xml
<export>
    <polygon_base plugin="${prefix}/plugins.xml" />
</export>
```

---

## Launch System (Python)

### Minimal launch file
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="pkg", executable="node_name", name="renamed", output="screen",
             parameters=[{"param1": 42}],
             remappings=[("/old/topic", "/new/topic")])
    ])
```

### Substitutions
```python
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

Node(
    parameters=[PathJoinSubstitution([
        FindPackageShare("pkg"), "config", "params.yaml"
    ])]
)
```

### Conditional nodes
```python
from launch.conditions import IfCondition, UnlessCondition

Node(..., condition=IfCondition(LaunchConfiguration("use_sim")))
```

### Declare arguments
```python
from launch.actions import DeclareLaunchArgument

DeclareLaunchArgument("robot_ip", default_value="192.168.1.100",
                      description="Robot IP address")
```

### Include another launch file
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([FindPackageShare("pkg"), "/launch/other.launch.py"]))
```

### Lifecycle node in launch
```python
from launch_ros.actions import LifecycleNode

LifecycleNode(package="pkg", executable="lifecycle_node", name="node")
```

### OpaqueFunction (runtime evaluation)
```python
from launch.actions import OpaqueFunction

def evaluate(context, *args, **kwargs):
    ip = LaunchConfiguration("robot_ip").perform(context)
    # ... decision logic here ...
    return [Node(...)]  # or []

LaunchDescription([
    OpaqueFunction(function=evaluate)
])
```
> **Important**: OpaqueFunction runs at launch PARSE time, not runtime.
> Decisions made here are baked into the launch tree and don't re-evaluate.

### XML launch file (alternative)
```xml
<launch>
    <node pkg="pkg" exec="node" name="renamed">
        <param name="param1" value="42"/>
        <remap from="/old" to="/new"/>
    </node>
</launch>
```

---

## Lifecycle Nodes

### State machine
```
┌──────────────┐   on_configure()   ┌──────────┐   on_activate()   ┌────────┐
│ Unconfigured │ ──────────────────> │ Inactive │ ────────────────> │ Active │
└──────────────┘                     └──────────┘                   └────────┘
      ^                                   │                             │
      │        on_cleanup()               │    on_deactivate()          │
      └───────────────────────────────────┘ <──────────────────────────┘
                                                                        │
      ┌───────────┐   on_shutdown()                                    │
      │ Finalized │ <──────────────────────────────────────────────────┘
      └───────────┘   (from any state)
```

### Implementation (C++)
```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    MyLifecycleNode() : LifecycleNode("my_node") {}

    CallbackReturn on_configure(const State&) {
        // Create resources: services, publishers, subscriptions
        // These work in Inactive state too
        pub_ = create_publisher<Msg>("topic", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State&) {
        // Lightweight: activate publishers, start timers
        pub_->on_activate();
        timer_->reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State&) {
        // Pause: stop timers, deactivate publishers
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const State&) {
        // Full teardown
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const State&) {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const State&) {
        // Must fully clean up — no partial state
        return CallbackReturn::SUCCESS;
    }
};
```

### Check state before work
```cpp
// Service callbacks should check state:
if (get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    res->success = false;
    res->message = "Node not active";
    return;
}
```

### Transition via CLI
```bash
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
ros2 lifecycle set /node_name deactivate
ros2 lifecycle set /node_name cleanup
ros2 lifecycle set /node_name shutdown
ros2 lifecycle list /node_name     # show current state
ros2 lifecycle get /node_name      # show available transitions
```

---

## Executors and Callback Groups

### Executor types
```cpp
// SingleThreadedExecutor: all callbacks serialized on one thread
rclcpp::executors::SingleThreadedExecutor exec;
exec.add_node(node);
exec.spin();

// MultiThreadedExecutor: N threads, callbacks can run concurrently
rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4);  // 4 threads
exec.add_node(node);
exec.spin();

// StaticSingleThreadedExecutor: no dynamic add/remove
rclcpp::executors::StaticSingleThreadedExecutor exec;
```

### Callback groups
```cpp
// MutuallyExclusive: callbacks in group run serially (default per-node)
auto group1 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
auto group2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

// Reentrant: callbacks can run concurrently across threads
auto reentrant = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

// Assign subscription to a specific group
auto options = rclcpp::SubscriptionOptions();
options.callback_group = group1;
auto sub = node->create_subscription<Msg>("topic", 10, cb, options);
```

### Deadlock scenario
```
┌─────────────────────────────────────────────────────┐
│ MutuallyExclusive callback group "A"                 │
│                                                      │
│  callback1() {                                       │
│    future = client->async_send_request(req);         │
│    future.wait_for(5s);   // ← BLOCKING              │
│  }                                                    │
│                                                      │
│  // response callback ALSO in group A                │
│  // CANNOT run because callback1 is blocking group A │
│  // → DEADLOCK (timeout or forever)                  │
│ ──────────────────────────────────────────────────── │
```

### Correct patterns
```cpp
// Option 1: callback-based client (no wait_for)
client->async_send_request(req, [](auto future) {
    // Process result in response callback
});

// Option 2: separate callback groups
auto srv_group = node->create_callback_group(MutuallyExclusive);
auto cli_group = node->create_callback_group(MutuallyExclusive);
// Server callbacks in srv_group, client in cli_group

// Option 3: Reentrant group + MultiThreadedExecutor
auto reentrant = node->create_callback_group(Reentrant);
```

---

## QoS Profiles

### Predefined profiles
```
rmw_qos_profile_sensor_data    — sensor data (BEST_EFFORT, VOLATILE, depth 5)
rmw_qos_profile_parameters     — parameter events (RELIABLE, VOLATILE, depth 1000)
rmw_qos_profile_services_default — services
rmw_qos_profile_system_default   — topics (RELIABLE, VOLATILE, depth 10)
rmw_qos_profile_default          — same as system_default
```

### Custom QoS
```cpp
auto qos = rclcpp::QoS(10)                 // depth
    .reliable()                            // or .best_effort()
    .transient_local()                     // or .volatile()
    .keep_last(10)                         // or .keep_all()
    .durability_volatile()                 // or .durability_transient_local()
    .lifespan(std::chrono::seconds(5));    // message expiry
```

### Key compatibility rules
- RELIABLE publisher → can't talk to BEST_EFFORT subscriber (blocked, no error)
- BEST_EFFORT publisher → compatible with RELIABLE subscriber
- TRANSIENT_LOCAL publisher (latched) → can't talk to VOLATILE subscriber
- TRANSIENT_LOCAL requires `keep_all()` depth for full history

### `/robot_description` special case
```cpp
// robot_state_publisher subscribes via /parameter_events, NOT the topic!
// To trigger URDF rebuild, you MUST use set_parameters(), not publish():
auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "robot_state_publisher");
client->set_parameters({{"robot_description", urdf_string}});

// If you DO need to subscribe to /robot_description topic:
auto qos = rclcpp::QoS(1).transient_local().reliable();
auto sub = node->create_subscription<std_msgs::msg::String>(
    "/robot_description", qos, callback);
```

---

## TF2

### Broadcaster (C++)
```cpp
#include <tf2_ros/transform_broadcaster.h>

auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped ts;
ts.header.stamp = node->now();
ts.header.frame_id = "parent_frame";
ts.child_frame_id = "child_frame";
ts.transform.translation.x = 1.0;
ts.transform.rotation.w = 1.0; // identity quaternion

tf_broadcaster->sendTransform(ts);
```

### Static broadcaster (C++)
```cpp
#include <tf2_ros/static_transform_broadcaster.h>

auto static_bc = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
ts.header.stamp = node->now();
static_bc->sendTransform(ts); // latched: sent once, persisted
```

### Listener + buffer (C++)
```cpp
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

// Lookup
try {
    auto ts = buffer->lookupTransform("target", "source", tf2::TimePointZero);
    // use ts.transform
} catch (const tf2::TransformException& e) {
    RCLCPP_ERROR(logger, "%s", e.what());
}
```

### Debugging
```bash
ros2 run tf2_tools view_frames      # generates frames.pdf
ros2 run tf2_ros tf2_echo source target
ros2 topic echo /tf
ros2 topic echo /tf_static
```

---

## Node Composition

### Component (C++) — loadable into container
```cpp
#include <rclcpp_components/register_node_macro.hpp>

class MyComponent : public rclcpp::Node { /* ... */ };

RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

### CMakeLists.txt for component
```cmake
add_library(my_component SHARED src/my_component.cpp)
ament_target_dependencies(my_component rclcpp rclcpp_components)
rclcpp_components_register_node(my_component
    PLUGIN "my_namespace::MyComponent"
    EXECUTABLE my_node)
```

### Load component at runtime
```bash
# Start container
ros2 run rclcpp_components component_container

# Load component into running container
ros2 component load /ComponentManager my_pkg my_namespace::MyComponent

# Standalone mode (container + component in one process)
ros2 component standalone my_pkg my_namespace::MyComponent
```

### Composition launch
```python
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

LoadComposableNodes(
    target_container="container",
    composable_node_descriptions=[
        ComposableNode(package="pkg", plugin="ns::Component", name="node1")
    ])
```

---

## Timers

```cpp
// Wall timer
auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    []() { RCLCPP_INFO(logger, "100ms tick"); });

// One-shot timer
auto oneshot = node->create_wall_timer(
    std::chrono::seconds(5),
    [timer_ptr = oneshot.get()]() mutable {
        // Do work once
        timer_ptr->cancel();  // cancel self
    });
```

---

## ros2cli Quick Reference

```bash
ros2 node list / info / kill
ros2 topic list / info / echo / hz / bw / pub
ros2 service list / type / call / find
ros2 param list / get / set / dump / load / delete / describe
ros2 action list / info / send_goal
ros2 interface show / list / package
ros2 pkg list / executables / xml
ros2 launch pkg launch_file.launch.py
ros2 bag record / play / info
ros2 run --prefix 'gdb -ex run --args' pkg exe   # debug
```

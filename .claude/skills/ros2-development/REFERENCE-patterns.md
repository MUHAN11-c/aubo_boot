# ROS 2 Design Patterns & Anti-Patterns

## Design Patterns

### 1. Async init pattern

When a node needs `shared_from_this()` (for clients, parameter clients, etc.), delay init past the constructor.

```cpp
class MyNode : public rclcpp::Node {
public:
    explicit MyNode(const rclcpp::NodeOptions& opt) : Node("my_node", opt) {}

    void init() {
        // shared_from_this() valid here
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this->shared_from_this(), "target_node");
        service_client_ = this->create_client<Srv>("service");
    }

private:
    std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
    rclcpp::Client<Srv>::SharedPtr service_client_;
};

// Usage
auto node = std::make_shared<MyNode>(options);
node->init();  // MUST call after shared_ptr constructed
```

### 2. Wall timer deferred init

For truly autonomous init (no external `init()` call):

```cpp
MyNode() : Node("my_node") {
    auto timer = create_wall_timer(std::chrono::milliseconds(1),
        [this, timer_ptr = timer.get()]() {
            timer_ptr->cancel();
            // shared_from_this() valid here
            param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
                shared_from_this(), "target");
        });
}
```

### 3. Composition over inheritance

Avoid inheriting Node when you don't need to. Pass the Node as a dependency:

```cpp
class MyWorker {
public:
    explicit MyWorker(rclcpp::Node::SharedPtr node) : node_(node) {
        pub_ = node_->create_publisher<Msg>("topic", 10);
    }
    void doWork() { pub_->publish(msg); }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<Msg>::SharedPtr pub_;
};
```

### 4. Safe service client pattern (callback-based)

```cpp
// Always safe — never blocks the callback group
void callService() {
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(logger_, "Service unavailable");
        return;
    }
    auto req = std::make_shared<Srv::Request>();
    req->data = 42;

    client_->async_send_request(req,
        [this](rclcpp::Client<Srv>::SharedFuture future) {
            auto res = future.get();
            // Process response safely
        });
}
```

### 5. Reentrant group for concurrent service calls

```cpp
// Node that handles multiple concurrent service requests
class ParallelServiceNode : public rclcpp::Node {
public:
    ParallelServiceNode() : Node("parallel") {
        reentrant_ = create_callback_group(Reentrant);
        sub_opts_.callback_group = reentrant_;

        srv_ = create_service<Srv>("service",
            [this](const auto req, auto res) {
                // Safe to run concurrently with other calls
                res->result = doWork(req->input);
            },
            rmw_qos_profile_services_default, reentrant_);
    }
private:
    rclcpp::CallbackGroup::SharedPtr reentrant_;
};
// Run with MultiThreadedExecutor(>= num_concurrent_calls)
```

### 6. Worker thread for long-running tasks

```cpp
class ProcessingNode : public rclcpp::Node {
public:
    ProcessingNode() : Node("processor") {
        srv_ = create_service<Srv>("process",
            [this](const auto req, auto res) {
                // Queue work, respond immediately
                res->accepted = true;
                work_queue_.push(req->data);
                cv_.notify_one();
            });

        worker_ = std::thread([this]() {
            while (running_) {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { return !work_queue_.empty() || !running_; });
                // Process work outside ROS callbacks
            }
        });
    }

    ~ProcessingNode() {
        running_ = false;
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }
private:
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<Data> work_queue_;
    std::atomic<bool> running_{true};
};
```

### 7. Parameter-driven behavior

```cpp
// React to parameter changes automatically
auto cb = [this](const std::vector<rclcpp::Parameter>& params) {
    auto res = rcl_interfaces::msg::SetParametersResult{};
    res.successful = true;
    for (auto& p : params) {
        if (p.get_name() == "publish_rate") {
            timer_->cancel();
            timer_ = create_wall_timer(
                std::chrono::milliseconds((int)(1000.0 / p.as_double())),
                std::bind(&MyNode::timerCb, this));
        }
    }
    return res;
};
add_on_set_parameters_callback(cb);
```

### 8. Topic relay / republish pattern

```cpp
// Subscribe to one topic, republish on another (with optional transform)
auto sub = node->create_subscription<Msg>("input", 10,
    [pub](Msg::SharedPtr msg) {
        auto out = transform(*msg);
        pub->publish(std::move(out));
    });
```

---

## Anti-Patterns

### 1. sync service call in a callback
```cpp
// DON'T: deadlock in MutuallyExclusive group
void callback() {
    auto future = client->async_send_request(req);
    future.wait();  // ← BLOCKS callback group → response handler can't run
}

// DO: use callback-based API
void callback() {
    client->async_send_request(req, [](auto future) {
        auto res = future.get();  // runs as response callback
    });
}
```

### 2. shared_from_this() in constructor
```cpp
// DON'T: bad_weak_ptr
MyNode() : Node("bad") {
    auto self = shared_from_this();  // ← CRASH: weak_ptr not initialized
}

// DO: deferred init (see pattern #1)
```

### 3. Busy-waiting for service
```cpp
// DON'T: spin-lock burns CPU
while (!client->wait_for_service(std::chrono::seconds(1))) {
    // ...
}

// DO: check once with timeout, fail gracefully
if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger, "Service not available");
    return;  // or retry later via timer
}
```

### 4. Large messages on main callback
```cpp
// DON'T: blocking executor with heavy processing
void imageCallback(Image::SharedPtr img) {
    auto result = heavyComputation(img);  // 100ms+
    pub->publish(result);
}

// DO: move heavy work to separate thread/pool
void imageCallback(Image::SharedPtr img) {
    thread_pool_.post([this, img = std::move(img)]() {
        auto result = heavyComputation(img);
        pub->publish(result);
    });
}
```

### 5. Unbounded message queues
```cpp
// DON'T: deep queue hides performance issues
auto qos = rclcpp::QoS(rclcpp::KeepAll());  // infinite

// DO: bounded queue with sensible depth
auto qos = rclcpp::QoS(10).keep_last(10);
// Higher for high-rate sensors:
auto sensor_qos = rclcpp::QoS(100).best_effort().keep_last(100);
```

### 6. Ignoring lifecycle state
```cpp
// DON'T: service works even when inactive
CallbackReturn on_deactivate(const State&) {
    return CallbackReturn::SUCCESS;  // But service still callable!
}

// DO: Check state in service callbacks
void onService(const auto req, auto res) {
    if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        res->success = false;
        return;
    }
    // ... handle request ...
}
```

### 7. QoS mismatch on /robot_description
```cpp
// DON'T: VOLATILE subscriber never receives TRANSIENT_LOCAL (latched) data
auto sub = node->create_subscription<StringMsg>(
    "/robot_description", 1, callback);  // defaults to VOLATILE

// DO: Match TRANSIENT_LOCAL publisher
auto qos = rclcpp::QoS(1).transient_local().reliable();
auto sub = node->create_subscription<StringMsg>(
    "/robot_description", qos, callback);
```

---

## Debugging

### Standard diagnostic commands
```bash
# Node introspection
ros2 node info /node_name           # all pubs/subs/services/actions
ros2 node list -a                   # all nodes (including hidden)

# Topic debugging
ros2 topic info /topic              # type + QoS + publishers/subscribers
ros2 topic echo /topic              # print messages
ros2 topic hz /topic                # measure publish rate
ros2 topic bw /topic                # measure bandwidth
ros2 topic delay /topic             # measure latency

# Service debugging
ros2 service list -t                # list with types
ros2 service type /service          # get type
ros2 service call /service Type "{data: 42}"

# Parameter introspection
ros2 param list /node
ros2 param describe /node param     # see type + constraints + description
ros2 param get /node param

# Action debugging
ros2 action info /action            # active goals
ros2 action send_goal /action Type "{order: 10}"

# Full system graph
ros2 run rqt_graph rqt_graph
rqt  # plugin-based: Topic Monitor, Service Caller, etc.
```

### Logging
```cpp
// C++ logging macros
RCLCPP_DEBUG(logger, "debug: %d", val);
RCLCPP_INFO(logger, "info message");
RCLCPP_WARN(logger, "warning");
RCLCPP_ERROR(logger, "error: %s", msg.c_str());
RCLCPP_FATAL(logger, "fatal");

// Named logger
auto logger = rclcpp::get_logger("my_component");

// Throttled (only logs every N ms)
RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 5000, "throttled 5s");

// Per-node log level
node->get_logger().set_level(rclcpp::Logger::Level::Debug);
```
```bash
# Runtime log level change
ros2 service call /node_name/set_logger_levels \
    rcl_interfaces/srv/SetLoggerLevels \
    "{set: {name: '', level: 10}}"  # 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR
```

### TF debugging
```bash
ros2 run tf2_tools view_frames    # → frames.pdf (TF tree graph)
ros2 run tf2_ros tf2_echo source target  # real-time transform
ros2 topic echo /tf               # inspect raw TF messages
ros2 topic echo /tf_static        # static transforms
```

### ros2 doctor
```bash
ros2 doctor          # run all checks
ros2 doctor --report # detailed report
ros2 wtf             # alias for doctor
```

### GDB debugging
```bash
# Start node under GDB
ros2 run --prefix 'gdb -ex run --args' pkg executable

# Attach to running node
ros2 run --prefix 'gdb -ex attach $(pgrep executable) --args' pkg executable

# With valgrind
ros2 run --prefix 'valgrind --leak-check=full' pkg executable
```

### Recording data for offline analysis
```bash
ros2 bag record -o debug_session /topic1 /topic2 /tf /tf_static
# Reproduce issue, then:
ros2 bag play debug_session
# Analyze with same tooling (echo, hz, etc.)
```

### Checking ROS 2 logs
```bash
# System logs
ls ~/.ros/log/
# Latest session
cat ~/.ros/log/latest/*.log
```

---

## Common Errors and Solutions

| Error | Likely Cause | Fix |
|-------|-------------|-----|
| `bad_weak_ptr` | `shared_from_this()` in constructor or `AsyncParametersClient` in constructor | Defer to `init()` method or 1ms wall timer |
| `transform not found` | TF tree not fully connected or timing | `ros2 run tf2_tools view_frames`; check timestamps |
| `Failed to fetch current robot state` | CurrentStateMonitor timeout (1s), single-threaded executor blocking | Use `MultiThreadedExecutor` + Reentrant group |
| `Could not find parameter robot_description` | Parameter not set on this node | `ros2 param list /node`; check YAML namespacing |
| Service call times out | Server not available, wrong service name, or deadlock | `ros2 service list`; check callback groups |
| `Controller XXX failed to activate` | ros2_control type mismatch or missing command_interface | `ros2 control list_hardware_interfaces` |
| `planning failed` / `No valid plan` | Collision, target unreachable, or IK failure | Check `/get_planning_scene`; verify target pose |
| `declare_parameter` already declared | Launch `automatically_declare_parameters_from_overrides(true)` + manual declare | Guard with `has_parameter()` before `declare_parameter()` |
| `/rosout` topic empty | No nodes use `RCLCPP_*` logging or log level filtered | Set logger level to DEBUG; check `rcutils` logging config |
| `TypeError: types.UnionType` | pydantic 1.x + PEP 604 (`X \| Y`) syntax | Use `typing.Optional[X]` or `typing.Union[X, Y]` |
| `ModuleNotFoundError: No module named 'httpx'` | pip dep not installed (not colcon-managed) | `pip3 install httpx websockets` |
| Build: `undefined reference to rclcpp::*` | Missing `find_package` or `ament_target_dependencies` | Add to CMakeLists.txt |
| Build: `fatal error: rclcpp/rclcpp.hpp` | `ros-humble-rclcpp` not installed | `sudo apt install ros-humble-rclcpp` |
| Colcon build slow | Full rebuild every time | `--packages-select pkg1 pkg2` or `--symlink-install` |
| Python import fails after colcon build | `PYTHONPATH` not updated | `source install/setup.bash` |

---

## Package Organization

### Recommended structure
```
my_package/
├── package.xml              # Required
├── CMakeLists.txt           # C++ (or setup.py/setup.cfg for Python)
├── include/my_package/      # Public headers
├── src/                     # Source files
├── config/                  # YAML parameter files
├── launch/                  # Launch files (.launch.py)
├── rviz/                    # RViz configs
├── urdf/                    # Robot description
├── test/                    # Tests
├── plugins.xml              # Plugin export descriptor
└── README.md
```

### package.xml essentials
```xml
<?xml version="1.0"?>
<package format="3">
    <name>my_package</name>
    <version>0.1.0</version>
    <description>What this package does</description>
    <maintainer email="user@example.com">User Name</maintainer>
    <license>Apache-2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    <!-- or ament_cmake_python for Python packages -->

    <depend>rclcpp</depend>
    <depend>std_msgs</depend>
    <!-- test dependencies -->
    <test_depend>ament_cmake_gtest</test_depend>
    <test_depend>ament_lint_auto</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

### CMakeLists.txt template (C++)
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(my_test test/test_my_node.cpp)
    target_link_libraries(my_test ${PROJECT_NAME})
endif()

ament_package()
```

### setup.py template (Python)
```python
from setuptools import setup

package_name = "my_package"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name + "/launch", ["launch/my_launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="What this package does",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "my_node = my_package.my_module:main",
        ],
    },
)
```

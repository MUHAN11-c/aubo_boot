# ROS 2 Software Architecture Design

Architectural patterns distilled from the ROS 2 ecosystem's most popular and
well-engineered projects: Autoware, Nav2, micro-ROS, MoveIt 2, ros2_control,
ros2_lingua, osmAG-Nav, OrionNav, Open-RMF, dora-rs, and more.

---

## 1. Layered Architecture

**Pattern**: Separate concerns into distinct layers with well-defined interfaces.

```
┌─────────────────────────────────────────────────┐
│               Application Layer                  │  ← mission logic, task planning
├─────────────────────────────────────────────────┤
│               ROS 2 Interface Layer              │  ← nodes, topics, services, actions
├─────────────────────────────────────────────────┤
│               Core Algorithm Layer               │  ← pure C++/Python, zero ROS deps
├─────────────────────────────────────────────────┤
│               Hardware Abstraction Layer         │  ← drivers, sensors, actuators
└─────────────────────────────────────────────────┘
```

**Exemplars**:
- **Autoware** — Sensing → Localization → Perception → Planning → Control → Vehicle
- **ros2_control** — Controller ↔ Controller Manager ↔ Hardware Interface
- **ros2_lingua** — `ros2_lingua_core` (pure Python, no ROS) + `ros2_lingua` (ROS2 nodes)

**Why**: Layers isolate change. Swap hardware without touching algorithms. Test core
logic without launching ROS. Port to new middleware without rewriting business logic.

**Anti-pattern**: Mixing SDK calls, business logic, and ROS communication in one
node. When the SDK API changes, everything breaks.

---

## 2. Core / Universe Split (Maturity Pipeline)

**Pattern**: Separate stable code (`core`) from experimental code (`universe`). Code
graduates from universe → core after proving reliability and API stability.

**Exemplar**: **Autoware** (the canonical example):
```
autoware_universe → (matures) → autoware_core
     ↑ experimental               ↑ stable, API-guaranteed
autoware_launch/  —  config only, separate repo
autoware_msgs/    —  interface definitions, separate repo
```

**Repo structure**:
```
autoware/                  ← meta-repo (.repos file only, no code)
├── src/
│   ├── autoware_core/     ← stable packages
│   ├── autoware_universe/ ← experimental packages
│   ├── autoware_launch/   ← launch + config only
│   └── autoware_msgs/     ← message definitions
```

**Why**: Allows rapid innovation (universe) without breaking production (core).
Users can fork only the launch/config repo to customize parameters. Interface
packages change slowly; implementation packages churn freely.

**Application to your project**:
- `ivg_interfaces/` — your interfaces-only package (equivalent to autoware_msgs)
- `aubo_driver_ros2/` — stable SDK wrapper
- `demo_driver/` — experimental service layer
- `latte_imitation/` — application-specific

---

## 3. Plugin-Based Extensibility

**Pattern**: Define abstract interfaces in C++ (base class), implement algorithms as
dynamically-loaded plugins via `pluginlib`. Swap implementations without recompiling
consumers.

**Exemplar — Nav2**:
```
controller_server
  ├── DWB Controller         (pluginlib class)
  ├── Regulated Pure Pursuit (pluginlib class)
  ├── MPPI Controller        (pluginlib class)
  └── YourCustomController   (pluginlib class) ← add without touching Nav2 code
```

**Exemplar — ros2_control**:
```
<ros2_control name="MyRobot" type="system">
    <hardware>
        <plugin>my_hardware_pkg/MyHardwareInterface</plugin>
    </hardware>
</ros2_control>
```

**Interface definition** (the contract):
```cpp
// nav2_core/controller.hpp
namespace nav2_core {
class Controller {
public:
    virtual ~Controller() = default;
    virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr&,
                           std::string name, std::shared_ptr<tf2_ros::Buffer>) = 0;
    virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped&,
        const geometry_msgs::msg::Twist&,
        nav2_core::GoalChecker*) = 0;
};
}
```

**Why**: Open/Closed Principle — open for extension, closed for modification.
Third parties can add planners/controllers/hardware drivers without forking upstream.

**Plugin XML**: Must be exported in `package.xml`:
```xml
<export>
    <nav2_core plugin="${prefix}/plugins.xml" />
</export>
```

---

## 4. Federated Action Server Architecture

**Pattern**: Each navigation capability is an independent ROS 2 Action Server.
A central orchestrator (Behavior Tree) coordinates them.

**Exemplar — Nav2**:
```
                    ┌──────────────┐
                    │  BtNavigator  │  ← Action Server (NavigateToPose)
                    └──┬───┬───┬───┘
                       │   │   │
          ┌────────────┼───┼───┼────────────┐
          ▼            ▼   ▼   ▼            ▼
   ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐
   │ Planner  │ │Controller│ │Behavior  │ │ Smoother │
   │ Server   │ │ Server   │ │ Server   │ │ Server   │
   └──────────┘ └──────────┘ └──────────┘ └──────────┘
   Action:       Action:      Action:      Action:
   ComputePath   FollowPath   Spin/BackUp  SmoothPath
```

**Why**: Loose coupling. Each server can:
- Be developed and tested independently
- Use its own callback group and executor
- Have its own lifecycle (managed nodes)
- Be swapped for an alternative implementation

**How**: Each server = LifecycleNode + Action Server. Controller Server doesn't
import Planner Server. They communicate only through ROS 2 actions.

---

## 5. Behavior Tree Orchestration

**Pattern**: Express mission logic as a Behavior Tree (BT) — a hierarchical task
structure defined in XML. BT nodes call ROS 2 actions/services. No C++ changes
needed to create new behaviors.

**Exemplar — Nav2**:
```xml
<root>
  <BehaviorTree ID="Navigate">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="1" name="ComputePath">
          <ComputePathToPose goal="{goal}" path="{path}" />
          <ClearEntireCostmap name="ClearGlobalCostmap" />
        </RecoveryNode>
      </RateController>
      <RecoveryNode number_of_retries="1" name="FollowPath">
        <FollowPath path="{path}" />
        <Sequence name="Recoveries">
          <ClearEntireCostmap name="ClearLocalCostmap" />
          <Spin/>
          <BackUp/>
        </Sequence>
      </RecoveryNode>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

**BT Node Types**:
| Type | Purpose |
|------|---------|
| **Action** | Calls a ROS 2 action (long-running), returns SUCCESS/FAILURE/RUNNING |
| **Condition** | Checks state, returns SUCCESS/FAILURE |
| **Sequence** | Ticks children in order. First failure → abort |
| **Fallback** | Ticks children in order. First success → done (used for recovery) |
| **PipelineSequence** | Ticks children in parallel (re-plan while following!) |
| **RateController** | Throttles child tick rate |
| **RecoveryNode** | Retries child with a recovery sub-tree on failure |

**Blackboard** (shared data): BT nodes exchange data via key-value pairs.
`{goal}`, `{path}` — the BT framework injects these before ticking.

**Why**: Mission logic is **declarative** (XML), not imperative (C++). Non-programmers
can design behaviors in Groot2 (visual BT editor). Recovery logic is naturally
expressed as fallback branches.

---

## 6. Lifecycle-Managed System Startup

**Pattern**: Nodes are LifecycleNodes. A LifecycleManager orchestrates startup in
dependency order, monitors health via bonds, and respawns failed nodes.

**Exemplar — Nav2**:
```yaml
lifecycle_manager:
  ros__parameters:
    node_names: [map_server, amcl, controller_server,
                 planner_server, behavior_server, bt_navigator]
    service_name: lifecycle_manager/manage_nodes  # starts in dependency order
    bond_timeout: 4.0    # heartbeat interval
    attempt_respawn: true
```

**Startup sequence** (deterministic):
```
map_server     → configure → activate
amcl           → configure → activate
planner_server → configure → activate
controller_server → configure → activate
bt_navigator   → configure → activate
→ /bond established (heartbeat every 200ms)
→ NavigateToPose action ready
```

**Why**: No race conditions at startup. If `controller_server` needs `planner_server`
to be ready first, the LifecycleManager guarantees ordering. Bond-based
heartbeats detect crashes and trigger automatic respawn.

**Lifecycle states in launch**:
```python
from launch_ros.actions import LifecycleNode

LifecycleNode(package="nav2_planner", executable="planner_server",
              name="planner_server", output="screen")
```

---

## 7. Hierarchical Decomposition (System of Systems)

**Pattern**: Split into **global reasoning** (slow, semantic, topological) and
**local execution** (fast, metric, reactive). Each layer operates at its own
frequency with its own representation of the world.

**Exemplar — osmAG-Nav**:
```
Global Layer (0.1-1 Hz):
  OpenStreetMap topological graph → room-level waypoints
  LCA-anchored re-localization
  ↓ outputs: intermediate goals

Local Layer (1-10 Hz):
  Rolling window costmap (constant memory)
  Standard Nav2 controller
  ↓ outputs: velocity commands
```

**Exemplar — OrionNav**:
```
LLM Planner (high-level):    "go to the kitchen, avoid the hallway"
    → Nav2/MPPI (mid-level):  path + velocity profile
        → CBF Safety Filter (low-level): enforce collision-free velocity
```

**Key technique — Rolling Window Costmap**:
```cpp
// Memory footprint depends on window size, NOT total map area
costmap->setRollingWindowSize(5.0, 5.0);  // 5m x 5m around robot
costmap->setGlobalFrame("map");           // anchored in global frame
// Costmap updates as robot moves — O(1) memory
```

**Why**: Global reasoning on full maps is slow. Reactive execution must be fast.
Decoupling them lets each layer use the right representation and update rate.
Lifelong deployment needs constant memory (rolling window).

---

## 8. Safety Filter Wrapper

**Pattern**: Insert a provably-safe filter between the controller output and
hardware commands. The filter enforces constraints (collision avoidance, joint
limits) regardless of what the controller requests.

**Exemplar — OrionNav**:
```
Planner → Controller(MPPI) → [CBF-QP Safety Filter] → Velocity Tracker → Robot
                                  ↑
                            LiDAR obstacles
```

**C++ pattern**:
```cpp
class SafetyFilter : public rclcpp::Node {
public:
    SafetyFilter() : Node("safety_filter") {
        cmd_sub_ = create_subscription<Twist>("cmd_vel_unsafe", 10,
            [this](Twist::SharedPtr cmd) {
                auto safe_cmd = enforceCBF(*cmd);  // project to safe set
                safe_pub_->publish(safe_cmd);
            });
    }
private:
    Twist enforceCBF(const Twist& desired) {
        // Control Barrier Function: if near obstacle, reduce velocity
        // Guarantees: h(x) >= 0 ⇒ safe, h'(x) + α·h(x) >= 0
    }
};
```

**Why**: Safety is separate from performance. A planner can be experimental or
LLM-driven — the safety filter catches bad outputs. Auditable: the filter is
small and verifiable.

---

## 9. Decoupled Core + Interface

**Pattern**: Put algorithms in a ROS-independent library. ROS nodes are thin
wrappers that call library functions.

**Exemplar — ros2_lingua**:
```
ros2_lingua_core/          ← pip install ros2-lingua-core (no ROS deps!)
├── planner.py             ← pure Python: backward-chaining, capability matching
├── schema.py              ← JSON Schema: scene graph, capability registry
└── test/                  ← pytest (no launch files, no ROS)

ros2_lingua/               ← ROS 2 package
├── lingua_node.cpp        ← thin wrapper: creates ROS actions for each capability
└── launch/
```

**Exemplar — Your project can do this**:
```
ivg_utils/                 ← pure Python/C++ library (no ROS deps for math)
├── math.py                ← quaternions, rotations, Euler angles
└── test/                  ← pure pytest

latte_imitation/           ← ROS 2 package
├── retarget_node.py       ← thin wrapper: imports ivg_utils.math
```

**Why**: Core library can be:
- Unit tested without `colcon`, `launch`, or ROS 2 infrastructure
- Reused in non-ROS contexts (data analysis, simulation, web backend)
- Published to PyPI / Conan for external consumers
- CI runs in seconds, not minutes

---

## 10. Data-Driven Configuration

**Pattern**: Drive behavior with YAML config files, not hardcoded C++/Python.
New behaviors = new YAML entries, zero code changes.

**Exemplar — Nav2 BT XML**:
New mission behavior = new XML file. No C++ changes.
```
navigate_to_pose.xml     ← standard
navigate_through_poses.xml
waypoint_follower.xml
my_custom_behavior.xml   ← add this file, load via parameter
```

**Exemplar — Your project (tools.yaml)**:
```yaml
gripper2:
  dock_approach_joints: [0.0, -0.5, 1.2, 0.0, 1.5, 0.0]
  trajectory: {type: "slide", direction: "left", distance: 0.05}
# New tool = new YAML entry. No C++ changes.
```

**Why**: Domain experts (not developers) can tune parameters. Reduces release
cycles. YAML is diffable, reviewable, and can be loaded at runtime.

---

## 11. Meta-Repository Pattern

**Pattern**: A top-level repo contains no code, only `.repos` files defining which
repos to check out and where. Versioned dependency management.

**Exemplar — Autoware**:
```yaml
# autoware.repos
repositories:
  autoware_core:
    type: git
    url: https://github.com/autowarefoundation/autoware_core.git
    version: release/v1.0
  autoware_universe:
    type: git
    url: https://github.com/autowarefoundation/autoware_universe.git
    version: main
```

```bash
mkdir -p autoware/src && cd autoware
vcs import src < autoware.repos
rosdep install --from-paths src -r -y
colcon build
```

**Why**: A developer can clone one repo and get the exact dependency versions.
No "works on my machine" surprises. Version pins in `.repos` = reproducible builds.

---

## 12. Interface-First Design

**Pattern**: Define `.msg`, `.srv`, `.action` files FIRST in a dedicated
interfaces package. Packages depend on interfaces, never the other way around.

**Dependency direction** (must be acyclic):
```
interfaces/   ← defines RobotStatus.msg, RunGripperSwap.srv, Patrol.action
     ↑
     ├── driver/     ← publishes RobotStatus
     ├── planner/    ← calls RunGripperSwap service
     └── executor/   ← provides Patrol action
```

**Why**: Interfaces change slower than implementations. Separating them prevents
unnecessary rebuild cascades. Multiple implementations can satisfy the same
interface.

**Your project has this**: `ivg_interfaces/` (52 definitions) → all packages
depend on it.

---

## 13. Client-Agent Pattern (Distributed / Embedded)

**Pattern**: A lightweight **client** runs on a constrained device (MCU, embedded
Linux). A full-featured **agent** runs on a ROS 2 host and bridges the client
into the ROS 2 data space.

**Exemplar — micro-ROS**:
```
┌────────────────────┐         ┌────────────────────────┐
│   MCU (ESP32)      │ serial  │   Linux Host            │
│                    │ ←─────→ │                         │
│  micro-ROS Client  │  UART   │  micro-ROS Agent        │
│  (RCLC, C99 API)   │         │  (bridges to ROS 2)     │
│                    │         │                         │
│  Pub: /imu_data    │         │  Exposes /imu_data      │
│  Sub: /cmd_vel     │         │  Forwards /cmd_vel      │
└────────────────────┘         └────────────────────────┘
```

**Why**: MCU has no DDS. The Agent handles discovery, serialization, QoS.
Client stays tiny (~30 kB). The pattern generalizes: any non-ROS system can
be bridged this way.

---

## 14. Docker Containerization

**Pattern**: Multi-stage Docker builds. Dev images have build tools + headers.
Runtime images have only binaries. Separate GPU variants.

**Exemplar — Autoware**:
```dockerfile
# base: ROS 2 + CycloneDDS
FROM ros:humble-ros-base AS base

# core: autoware_core packages built
FROM base AS core
COPY --from=build /opt/autoware /opt/autoware

# universe: autoware_universe packages built
FROM core AS universe

# universe-cuda: GPU acceleration
FROM universe AS universe-cuda
RUN apt install cuda-toolkit-12-0

# runtime: stripped, no build tools
FROM universe AS runtime
RUN apt purge -y build-essential
```

**Why**: Reproducible environments. `docker compose up` = entire stack running.
CI/CD: build once, deploy anywhere. GPU variant = same code, different base image.

---

## 15. Zero-Copy Shared Memory (Performance)

**Pattern**: Use shared memory (Apache Arrow, iceoryx) for large data between
nodes in the same process or same machine. Avoid serialization overhead.

**Exemplar — dora-rs** (10-17x latency improvement over ROS 2 pub/sub):
```rust
// dora-rs: data stays in shared memory
let data = arrow::array::Float64Array::from(vec![1.0, 2.0, 3.0]);
sender.send(data);  // zero-copy: pointer to shared memory, no serialization
```

**ROS 2 built-in: Intra-process + loaned messages**:
```cpp
// C++: intra-process (same context, zero-copy)
auto opts = rclcpp::NodeOptions().use_intra_process_comms(true);
auto msg = std::make_unique<Msg>();  // unique_ptr → no copy
pub->publish(std::move(msg));        // ownership transfer, zero-copy

// Loaned message: borrow middleware buffer, write into it
auto loaned = pub->borrow_loaned_message();
loaned.get().field = value;  // write directly into DDS buffer
pub->publish(std::move(loaned));
```

**When**: Point clouds (50MB+), images (10MB+), or 1000+ Hz control loops.

---

## 16. Multi-Node Composition (vs Multi-Process)

**Decision matrix**:

| Approach | When to Use | Example |
|----------|------------|---------|
| **Separate process** | Independent lifecycle, different teams, different languages | Nav2 servers (each = separate process) |
| **Component in container** | Shared memory, low latency, same lifecycle | sensor driver + preprocessor |
| **Intra-process** | Zero-copy, both C++, same executor | publisher + subscriber chain |

**Launch component composition**:
```python
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

LoadComposableNodes(
    target_container="sensor_container",
    composable_node_descriptions=[
        ComposableNode(package="camera_driver", plugin="ns::CameraDriver"),
        ComposableNode(package="image_proc", plugin="ns::Rectify"),
        ComposableNode(package="image_proc", plugin="ns::Resize"),
    ])
```

**Why**: Composition saves IPC overhead. Separate processes enable independent
restart. Choose based on lifecycle coupling, not performance alone.

---

## 17. Distributed Consensus for Multi-Robot

**Pattern**: Zone-partitioned architecture. Each zone has an elected leader that
runs local coordination. Cross-zone conflicts resolved via consensus protocol.

**Exemplar — DiRAC**:
```
Zone A (rooms 1-3)       Zone B (rooms 4-6)       Zone C (hallways)
  Leader: Robot 1          Leader: Robot 3           Leader: Robot 5
  Follower: Robot 2        Follower: Robot 4         Follower: Robot 6

        │                       │                       │
        └───────────────────────┼───────────────────────┘
                                │
                    RAFT-inspired consensus
                    (tick-synchronized, deterministic)
```

**Why**: Centralized fleet management doesn't scale. Zone partitioning bounds
communication overhead. Leader election handles failures gracefully.

---

## 18. LLM Integration with Guardrails

**Pattern**: LLMs reason at the **task level** only. They select from a
pre-registered **capability catalog**. Every LLM action passes through
**precondition checks** and **postcondition validation**.

**Exemplar — ros2_lingua**:
```python
# Capability is registered with schema + pre/post conditions
class NavigateToRoom(LinguaMixin):
    capability_name = "navigate_to_room"
    schema = {
        "room": str,        # target room name
        "speed": float      # 0.0-1.0
    }
    preconditions = ["robot_is_stabilized", "map_is_loaded"]
    postconditions = ["robot_at_goal"]

    def execute(self, room: str, speed: float):
        # ROS 2 action call — only registered capabilities can be invoked
        return self.nav_client.send_goal(NavigateToPose.Goal(
            pose=get_room_pose(room),
            speed=speed
        ))
```

**Rules for LLM integration**:
1. LLM never generates ROS 2 API calls directly — only capability names
2. Backward-chaining planner injects missing prerequisites
3. Safety filter is always the last layer before hardware
4. Human-in-the-loop for irreversible actions (grasp, pour, dock)

**Why**: LLMs hallucinate. Explicit capability registration + guardrails prevent
the robot from executing nonsensical or dangerous commands.

---

## Architecture Decision Checklist

When designing a new ROS 2 system, answer these:

1. **Layers**: Where does each concern live? (algorithm → ROS wrapper → application)
2. **Interfaces first**: What `.msg/.srv/.action` do we need before writing nodes?
3. **Plugin or not**: Will this need multiple implementations? If yes → pluginlib.
4. **Lifecycle or not**: Does startup order matter? Need health monitoring? → LifecycleNode.
5. **Behavior Tree or imperative**: Complex mission with recovery? → BT. Simple sequential? → imperative.
6. **Component or process**: Shared memory needed? Lifecycle coupled? → component. Otherwise → process.
7. **Data-driven**: What needs to be configurable without recompilation? → YAML.
8. **Safety layer**: Where does the safety guarantee live? → separate node, last before hardware.
9. **Docker or bare metal**: Need reproducible CI? Multi-robot deployment? → Docker.
10. **LLM or not**: If LLM → explicit capability catalog + safety filter required.

---

## Project References

| Project | GitHub | Stars | Key Pattern |
|---------|--------|-------|-------------|
| Autoware | [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) | 10k+ | Core/Universe split, layered AD stack, meta-repo |
| Nav2 | [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | 3k+ | Federated servers, BT orchestration, pluginlib |
| MoveIt 2 | [moveit/moveit2](https://github.com/moveit/moveit2) | 1k+ | Plugin-based planners/kinematics, planning scene |
| micro-ROS | [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) | 500+ | Client-Agent, C99 API, RTOS abstraction |
| ros2_control | [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | 500+ | Hardware abstraction plugin, lifecycle control |
| Open-RMF | [open-rmf/rmf](https://github.com/open-rmf/rmf) | 500+ | Multi-robot fleet management, cross-vendor |
| ros2_lingua | [purahan/ros2_lingua](https://github.com/purahan/ros2_lingua) | new | Core/Interface decoupling, capability catalog |
| dora-rs | [dora-rs/dora](https://github.com/dora-rs/dora) | 3k+ | Zero-copy Arrow, Rust hot-path, WebSocket control |

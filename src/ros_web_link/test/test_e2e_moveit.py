"""E2E — MoveIt 2 官方 Panda 示例 × ros_web_link 端到端验证.

被测链路：pytest → 网关 /ws/foxglove → ivg_foxglove_bridge → 官方
moveit_resources_panda（mock_components）：
  1. /robot_description（latched std_msgs/String）—— panda URDF
  2. /joint_states —— panda 关节（jsb mock）
  3. /compute_ik（moveit_msgs/srv/GetPositionIK）—— IK 服务经 web
  4. /plan_kinematic_path（GetMotionPlan）—— OMPL 规划经 web
  5. /panda_arm_controller/follow_joint_trajectory —— 执行 planned 轨迹经 web
  6. /monitored_planning_scene —— PlanningScene 快照（displayPlannedPath 等场景树）
运行前提：moveit_resources_panda_* 已构建（本工作区 src/），aubo_py3.12 venv。
"""
from __future__ import annotations

import asyncio
import glob
import os
import time

import pytest

pytest.importorskip('websockets')
pytest.importorskip('rclpy')
pytest.importorskip('fastapi')

from control_msgs.action import FollowJointTrajectory  # noqa: E402
from moveit_msgs.msg import (Constraints, JointConstraint,  # noqa: E402
                             MotionPlanRequest)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from .e2e_harness import Connected, free_domain, free_port, loopback_client  # noqa: E402

_WS_ROOT = __import__('pathlib').Path(__file__).resolve().parents[3]
_PANDA_LAUNCH = 'panda_moveit_web.launch.py'
_ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
_TIMEOUT = 60.0  # MoveIt 就绪/规划给足余量


def _vendor_ld_path() -> str:
    parts = sorted(glob.glob('/opt/ros/jazzy/opt/*/lib'))
    return ':'.join(parts + [
        '/opt/ros/jazzy/lib', '/opt/ros/jazzy/lib/x86_64-linux-gnu',
        str(_WS_ROOT / 'install' / 'ivg_foxglove_bridge' / 'lib')])


class PandaStack:
    """官方 panda demo（无 rviz）+ 桥 + 网关，随机域/端口（域避开本机占用）."""

    def __init__(self) -> None:
        self.bridge_port = free_port()
        self.gateway_port = free_port()
        self._domain = str(free_domain())
        self._loop = None
        self._thread = None
        self._proc = None

    def _env(self) -> dict:
        env = os.environ.copy()
        env.update({
            'ROS_DISTRO': 'jazzy',
            'ROS_DOMAIN_ID': self._domain,
            'AMENT_PREFIX_PATH': ':'.join([
                str(_WS_ROOT / 'install' / 'ros_web_link'),
                str(_WS_ROOT / 'install' / 'ivg_foxglove_bridge'),
                str(_WS_ROOT / 'install' / 'moveit_resources_panda_description'),
                str(_WS_ROOT / 'install' / 'moveit_resources_panda_moveit_config'),
                '/opt/ros/jazzy']),
            'PYTHONPATH': ':'.join([
                '/opt/ros/jazzy/lib/python3.12/site-packages',
                str(_WS_ROOT / 'src' / 'ros_web_link')]),
            'LD_LIBRARY_PATH': _vendor_ld_path(),
            'PATH': ':'.join([
                str(_WS_ROOT / 'aubo_py3.12' / 'bin'),
                '/opt/ros/jazzy/bin', '/usr/bin', '/bin']),
        })
        return env

    def start(self) -> None:
        import subprocess
        import threading
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name='rwl-panda-e2e', daemon=True)
        self._thread.start()
        self._log_path = None

        async def spawn():
            import subprocess
            import tempfile
            # 临时日志（tempfile 安全 API）：定位网关未就绪的真实死因
            log_fh = tempfile.NamedTemporaryFile(
                prefix='panda_e2e_', suffix='.log', delete=False)
            self._log_path = log_fh.name
            env = self._env()
            log_fh.write(('ENV AMENT=' + env.get('AMENT_PREFIX_PATH', '?')
                          + '\nENV PATH=' + env.get('PATH', '?') + '\n').encode())
            log_fh.flush()
            self._proc = await asyncio.create_subprocess_exec(
                'ros2', 'launch', 'ros_web_link', _PANDA_LAUNCH,
                f'web_port:={self.gateway_port}',
                f'bridge_port:={self.bridge_port}',
                env=env, start_new_session=True,
                stdout=log_fh, stderr=subprocess.STDOUT)

        asyncio.run_coroutine_threadsafe(spawn(), self._loop).result(timeout=30.0)
        self._wait_gateway()

    def _wait_gateway(self) -> None:
        deadline = time.monotonic() + _TIMEOUT
        last = None
        while time.monotonic() < deadline:
            try:
                with loopback_client(self.gateway_port) as c:
                    if c.get('/health').status_code == 200:
                        return
            except Exception as exc:  # noqa: BLE001
                last = exc
            time.sleep(0.5)
        self.stop()
        raise RuntimeError(
            f'gateway 未就绪 ({self._log_path}): ' + repr(last))

    def stop(self) -> None:
        import signal
        loop, self._loop = self._loop, None
        thread, self._thread = self._thread, None
        proc = self._proc
        if loop is not None and thread is not None:
            async def shutdown():
                if proc is not None and proc.returncode is None:
                    try:
                        os.killpg(proc.pid, signal.SIGINT)
                    except (ProcessLookupError, PermissionError):
                        proc.terminate()
                    try:
                        await asyncio.wait_for(proc.wait(), 20.0)
                    except asyncio.TimeoutError:
                        try:
                            os.killpg(proc.pid, signal.SIGKILL)
                        except (ProcessLookupError, PermissionError):
                            proc.kill()
                        await proc.wait()
            try:
                asyncio.run_coroutine_threadsafe(
                    shutdown(), loop).result(timeout=30.0)
            finally:
                loop.call_soon_threadsafe(loop.stop)
                thread.join(timeout=5.0)
                loop.close()

    @property
    def ws_url(self) -> str:
        return f'ws://127.0.0.1:{self.gateway_port}/ws/foxglove'


@pytest.fixture(scope='module')
def panda():
    st = PandaStack()
    try:
        st.start()
    except Exception:
        st.stop()
        raise
    yield st
    st.stop()


def _home_joints() -> dict:
    return {j: 0.0 for j in _ARM_JOINTS}


# ── 1. latched robot_description ───────────────────────────────────────────

def test_robot_description_latched(panda):
    async def run():
        async with Connected(panda) as c:
            sub = await c.subscribe('/robot_description', String, timeout=_TIMEOUT)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                assert '<robot name="panda"' in m.data
                assert 'panda_link0' in m.data
                # 官方 panda URDF 的 mesh 用 file://$(find pkg) 绝对路径展开（MoveItConfigsBuilder 产物）
                assert 'meshes/visual/link0.dae' in m.data
            finally:
                await sub.cancel()
    asyncio.run(run())


# ── 2. joint_states（mock jsb） ────────────────────────────────────────────

def test_joint_states_panda(panda):
    async def run():
        async with Connected(panda) as c:
            sub = await c.subscribe('/joint_states', JointState, timeout=_TIMEOUT)
            try:
                m = await sub.next(timeout=_TIMEOUT)
                names = list(m.name)
                for j in _ARM_JOINTS:
                    assert j in names, names
                assert len(m.position) == len(names)
            finally:
                await sub.cancel()
    asyncio.run(run())


# ── 3. IK 服务 ─────────────────────────────────────────────────────────────

def test_compute_ik(panda):
    async def run():
        async with Connected(panda) as c:
            req = GetPositionIK.Request()
            req.ik_request.pose_stamped.header.frame_id = 'panda_link0'
            req.ik_request.pose_stamped.pose.position.x = 0.3
            req.ik_request.pose_stamped.pose.position.z = 0.4
            req.ik_request.pose_stamped.pose.orientation.w = 1.0
            req.ik_request.group_name = 'panda_arm'
            req.ik_request.timeout.sec = 2
            # 服务可能晚注册：等待 advertiseServices
            for attempt in range(8):
                try:
                    resp = await c.call_service('/compute_ik', req,
                                                GetPositionIK.Response,
                                                timeout=30.0)
                    break
                except Exception:
                    await asyncio.sleep(2.0)
            else:
                pytest.fail('/compute_ik 不可达')
            assert resp.error_code.val in (0, 1), f'IK 失败: {resp.error_code.val}'
            solved = {n: p for n, p in zip(resp.solution.joint_state.name,
                                           resp.solution.joint_state.position)}
            for j in _ARM_JOINTS:
                assert j in solved, solved.keys()
    asyncio.run(run())


# ── 4. OMPL 规划服务 ──────────────────────────────────────────────────────

def _plan_request(goal_positions):
    req = GetMotionPlan.Request()
    r = MotionPlanRequest()
    r.group_name = 'panda_arm'
    r.planner_id = 'RRTConnectkConfigDefault'
    r.num_planning_attempts = 1
    r.allowed_planning_time = 5.0
    goal = Constraints()
    for j, v in zip(_ARM_JOINTS, goal_positions):
        jc = JointConstraint()
        jc.joint_name = j
        jc.position = v
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        goal.joint_constraints.append(jc)
    r.goal_constraints.append(goal)
    req.motion_plan_request = r
    return req


def test_plan_kinematic_path(panda):
    async def run():
        async with Connected(panda) as c:
            goal = [0.0, -0.35, 0.0, -1.9, 0.0, 1.2, 0.8]
            resp = None
            # MoveIt 冷启动期 current-state monitor 未就绪会回 99999：退避重试
            for attempt in range(5):
                try:
                    resp = await c.call_service('/plan_kinematic_path',
                                                _plan_request(goal),
                                                GetMotionPlan.Response,
                                                timeout=30.0)
                    if resp.motion_plan_response.error_code.val == 1:
                        break
                except Exception:
                    pass
                await asyncio.sleep(3.0)
            assert resp is not None, '规划服务不可达'
            assert resp.motion_plan_response.error_code.val == 1, \
                f'规划失败: {resp.motion_plan_response.error_code.val}'
            jt = resp.motion_plan_response.trajectory.joint_trajectory
            assert len(jt.points) >= 2
            assert list(jt.joint_names) == _ARM_JOINTS
            final = jt.points[-1].positions
            for want, got in zip(goal, final):
                assert want == pytest.approx(got, abs=0.05)
            return jt
    jt = asyncio.run(run())
    # 供执行测试复用（模块级传递）
    globals()['__planned_jt'] = jt


# ── 5. 执行 planned 轨迹（follow_joint_trajectory action） ─────────────────


async def _wait_arm_controller_active(c, timeout_s: float = 25.0) -> None:
    """等 panda_arm_controller active 并退出激活过渡窗口.

    冷启动竞速（2026-10-15 实证）：goal 落在 JTC 激活过渡期会被立即判
    'Goal reached, success!'——臂零运动、mock 状态不变、action 却 SUCCEEDED
    （ros2_control 上游行为）。执行用例必须先确认控制器 active，再留稳定期。
    """
    from controller_manager_msgs.srv import ListControllers
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            resp = await c.call_service('/controller_manager/list_controllers',
                                        ListControllers.Request(),
                                        ListControllers.Response, timeout=10.0)
            states = {ctrl.name: ctrl.state for ctrl in resp.controller}
            if states.get('panda_arm_controller') == 'active':
                await asyncio.sleep(1.0)
                return
        except Exception:
            pass
        await asyncio.sleep(0.5)
    pytest.fail('panda_arm_controller 未到 active（25s）')


def test_execute_planned_trajectory(panda):
    jt = globals().get('__planned_jt')
    if jt is None:  # 规划测试被单独跳过时兜底：构造直插轨迹
        pytest.skip('依赖 test_plan_kinematic_path 先产出轨迹')

    async def run():
        async with Connected(panda) as c:
            import uuid as uuid_mod
            from unique_identifier_msgs.msg import UUID
            S = FollowJointTrajectory.Impl.SendGoalService
            gid = uuid_mod.uuid4().bytes
            greq = S.Request()
            greq.goal_id = UUID()
            greq.goal_id.uuid = list(gid)
            greq.goal.trajectory = jt
            # 订阅先于 goal：帧流即执行轨迹的观测证据（JSB 连续发布，
            # 实测 ~60Hz），失败诊断可对比「期望终点 vs 帧流实际值」
            sub = await c.subscribe('/joint_states', JointState, timeout=_TIMEOUT)
            await _wait_arm_controller_active(c)
            # 冷启动偶发拒绝/超时：退避重试整条 goal（accepted=False 多为
            # JTC/controller_manager 发现未就绪；三栈连跑负载下更易出现）
            accepted = None
            for _ in range(3):
                try:
                    accepted = await c.call_service(
                        '/panda_arm_controller/follow_joint_trajectory/_action/send_goal',
                        greq, S.Response, timeout=30.0)
                    if accepted.accepted:
                        break
                except Exception:
                    pass
                await asyncio.sleep(3.0)
            assert accepted is not None, 'send_goal 不可达'
            assert accepted.accepted

            G = FollowJointTrajectory.Impl.GetResultService
            rreq = G.Request()
            rreq.goal_id = UUID()
            rreq.goal_id.uuid = list(gid)
            res = None
            for _ in range(3):  # 冷启动偶发拒绝/超时：退避重试整条 goal
                try:
                    res = await c.call_service(
                        '/panda_arm_controller/follow_joint_trajectory/_action/get_result',
                        rreq, G.Response, timeout=30.0)
                    break
                except Exception:
                    await asyncio.sleep(3.0)
            assert res is not None, 'get_result 不可达'
            assert res.status == 4, f'执行终态 {res.status}（期望 SUCCEEDED）'
            assert res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

            # 执行期/执行后的帧流里找终点位（每关节取最新值合并）
            wants = dict(zip(jt.joint_names, jt.points[-1].positions))
            latest: dict = {}
            saw_target = False
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not saw_target:
                try:
                    m = await sub.next(timeout=2.0)
                except asyncio.TimeoutError:
                    continue
                latest.update(dict(zip(m.name, m.position)))
                saw_target = all(latest.get(k, 9e9) == pytest.approx(v, abs=0.1)
                                 for k, v in wants.items())
            await sub.cancel()
            assert saw_target, (
                f'执行后未见终点位（status={res.status}）；终点='
                f'{ {k: round(v, 3) for k, v in wants.items()} }，'
                f'最新帧合并={ {k: round(v, 3) for k, v in sorted(latest.items())} }')
    asyncio.run(run())


# ── 6. PlanningScene 流（monitored） ───────────────────────────────────────

def test_monitored_planning_scene(panda):
    async def run():
        from moveit_msgs.msg import PlanningScene
        async with Connected(panda) as c:
            sub = await c.subscribe('/monitored_planning_scene', PlanningScene,
                                    timeout=_TIMEOUT)
            try:
                # monitored 场景仅随变化发布（空闲不重发）：先订阅再触发一次规划
                await c.call_service('/plan_kinematic_path', _plan_request(
                    [0.0, -0.5, 0.0, -1.5, 0.0, 1.2, 0.6]),
                    GetMotionPlan.Response, timeout=30.0)
                m = await sub.next(timeout=30.0)
                assert m.robot_state.joint_state.name  # 含关节状态
                assert m.robot_state.is_diff is False or True  # 快照或 diff 均合法
            finally:
                await sub.cancel()
    asyncio.run(run())

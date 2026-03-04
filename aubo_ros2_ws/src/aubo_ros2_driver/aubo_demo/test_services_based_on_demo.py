#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from demo_interface.srv import (
    GetCurrentState, MoveToPose, PlanTrajectory, ExecuteTrajectory,
    SetSpeedFactor, SetRobotIO, ReadRobotIO
)
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import sys

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        
        self.test_pose = Pose()
        self.test_pose.position.x = -0.07401805371046066
        self.test_pose.position.y = -0.20905423164367676
        self.test_pose.position.z = 0.9532700777053833
        self.test_pose.orientation.x = 0.7025718092918396
        self.test_pose.orientation.y = -0.00014736213779542595
        self.test_pose.orientation.z = -0.0008002749527804554
        self.test_pose.orientation.w = 0.711612343788147
        
        self.get_logger().info("Service Tester initialized")
        self.get_logger().info(f"Test pose: x={self.test_pose.position.x:.6f}, "
                              f"y={self.test_pose.position.y:.6f}, "
                              f"z={self.test_pose.position.z:.6f}")
        
    def wait_for_services(self, timeout_sec=10.0):
        services = {
            "/get_current_state": GetCurrentState,
            "/set_speed_factor": SetSpeedFactor,
            "/plan_trajectory": PlanTrajectory,
            "/execute_trajectory": ExecuteTrajectory,
            "/move_to_pose": MoveToPose,
        }
        
        for service_name, service_type in services.items():
            client = self.create_client(service_type, service_name)
            self.get_logger().info(f"等待服务 {service_name} 可用...")
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f"服务 {service_name} 不可用")
                return False
            self.get_logger().info(f"服务 {service_name} 已就绪")
        
        return True
    
    def test_get_current_state(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("步骤1: 获取机器人当前状态")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(GetCurrentState, '/get_current_state')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("GetCurrentState service not available")
            return False
        
        request = GetCurrentState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ 成功获取状态: %s" % response.message)
                    pos = response.cartesian_position.position
                    self.get_logger().info(f"  当前位置: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
                    self.get_logger().info(f"  关节位置 (弧度): {response.joint_position_rad}")
                    return True
                else:
                    self.get_logger().error(f"✗ 获取状态失败: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ GetCurrentState: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ GetCurrentState: TIMEOUT")
            return False
    
    def test_set_speed_factor(self, factor=0.5):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"步骤2: 设置速度因子 (factor={factor})")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(SetSpeedFactor, '/set_speed_factor')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetSpeedFactor service not available")
            return False
        
        request = SetSpeedFactor.Request()
        request.velocity_factor = factor
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ %s" % response.message)
                    return True
                else:
                    self.get_logger().error(f"✗ 设置速度因子失败: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ SetSpeedFactor: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ SetSpeedFactor: TIMEOUT")
            return False
    
    def test_read_robot_io(self, io_type="digital_input", io_index=0):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"步骤3: 读取 {io_type} #{io_index}")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(ReadRobotIO, '/read_robot_io')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("ReadRobotIO service not available, skipping")
            return None
        
        request = ReadRobotIO.Request()
        request.io_type = io_type
        request.io_index = io_index
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    value_str = "HIGH" if response.value > 0.5 else "LOW"
                    self.get_logger().info(f"✓ {io_type} #{io_index} = {value_str} (value: {response.value})")
                    return True
                else:
                    self.get_logger().warn(f"✗ 读取IO失败: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().warn(f"✗ ReadRobotIO: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().warn("✗ ReadRobotIO: TIMEOUT")
            return False
    
    def test_set_robot_io(self, io_type="digital_output", io_index=0, value=1.0):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"步骤4: 设置 {io_type} #{io_index} 为 {value}")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(SetRobotIO, '/set_robot_io')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("SetRobotIO service not available, skipping")
            return None
        
        request = SetRobotIO.Request()
        request.io_type = io_type
        request.io_index = io_index
        request.value = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ {response.message}")
                    return True
                else:
                    self.get_logger().warn(f"✗ 设置IO失败: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().warn(f"✗ SetRobotIO: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().warn("✗ SetRobotIO: TIMEOUT")
            return False
    
    def test_plan_trajectory(self, use_joints=False):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"步骤5: 规划轨迹到目标位置 (use_joints={use_joints})")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(PlanTrajectory, '/plan_trajectory')
        
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("PlanTrajectory service not available")
            return None
        
        request = PlanTrajectory.Request()
        request.target_pose = self.test_pose
        request.use_joints = use_joints
        
        self.get_logger().info("发送规划请求: 目标位姿 x=%.3f, y=%.3f, z=%.3f" %
                              (self.test_pose.position.x, self.test_pose.position.y, self.test_pose.position.z))
        self.get_logger().info("使用关节空间: %s" % ("true" if use_joints else "false"))
        
        future = client.call_async(request)
        self.get_logger().info("等待规划完成 (timeout: 60s)...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ 规划成功: %s" % response.message)
                    self.get_logger().info(f"  规划时间: {response.planning_time:.3f} 秒")
                    self.get_logger().info(f"  轨迹点数: {len(response.trajectory.points)}")
                    return response.trajectory
                else:
                    self.get_logger().error(f"✗ 规划失败: {response.message}")
                    return None
            except Exception as e:
                self.get_logger().error(f"✗ PlanTrajectory: EXCEPTION - {str(e)}")
                return None
        else:
            self.get_logger().error("✗ PlanTrajectory: TIMEOUT")
            return None
    
    def test_execute_trajectory(self, trajectory):
        if trajectory is None:
            self.get_logger().error("Cannot execute: trajectory is None")
            return False
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("步骤6: 执行规划好的轨迹")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(ExecuteTrajectory, '/execute_trajectory')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory service not available")
            return False
        
        request = ExecuteTrajectory.Request()
        request.trajectory = trajectory
        
        self.get_logger().info("发送执行请求: 轨迹包含 %d 个点" % len(trajectory.points))
        joint_names_str = ", ".join(trajectory.joint_names)
        self.get_logger().info("关节名称: [%s]" % joint_names_str)
        
        future = client.call_async(request)
        self.get_logger().info("等待轨迹执行完成...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ 执行成功: %s" % response.message)
                    return True
                else:
                    self.get_logger().error(f"✗ 执行失败 (错误码: {response.error_code}): {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ ExecuteTrajectory: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ ExecuteTrajectory: TIMEOUT")
            return False
    
    def test_move_to_pose(self, use_joints=False, velocity_factor=0.3, acceleration_factor=0.3):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"步骤7: 移动到目标位姿 (use_joints={use_joints}, "
                              f"velocity_factor={velocity_factor}, acceleration_factor={acceleration_factor})")
        self.get_logger().info("=" * 60)
        
        client = self.create_client(MoveToPose, '/move_to_pose')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveToPose service not available")
            return False
        
        request = MoveToPose.Request()
        request.target_pose = self.test_pose
        request.use_joints = use_joints
        request.velocity_factor = velocity_factor
        request.acceleration_factor = acceleration_factor
        
        self.get_logger().info("发送请求: 目标位姿 x=%.3f, y=%.3f, z=%.3f" %
                              (self.test_pose.position.x, self.test_pose.position.y, self.test_pose.position.z))
        self.get_logger().info("使用关节空间: %s, 速度因子: %.2f, 加速度因子: %.2f" %
                              (("true" if use_joints else "false"), velocity_factor, acceleration_factor))
        
        future = client.call_async(request)
        self.get_logger().info("等待移动完成...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ 成功: %s" % response.message)
                    return True
                else:
                    self.get_logger().error(f"✗ 失败 (错误码: {response.error_code}): {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ MoveToPose: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ MoveToPose: TIMEOUT")
            return False
    
    def run_comprehensive_test(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("综合示例：完整工作流程演示")
        self.get_logger().info("=" * 60 + "\n")
        
        if not self.wait_for_services():
            self.get_logger().error("无法连接到所有服务，退出")
            return False
        
        results = {}
        
        results['get_current_state'] = self.test_get_current_state()
        time.sleep(1)
        
        results['read_robot_io'] = self.test_read_robot_io("digital_input", 0)
        time.sleep(1)
        
        results['set_robot_io'] = self.test_set_robot_io("digital_output", 0, 1.0)
        time.sleep(1)
        
        results['set_speed_factor'] = self.test_set_speed_factor(0.5)
        time.sleep(1)
        
        results['plan_trajectory_cartesian'] = self.test_plan_trajectory(use_joints=False)
        time.sleep(1)
        
        if results['plan_trajectory_cartesian']:
            results['execute_trajectory'] = self.test_execute_trajectory(results['plan_trajectory_cartesian'])
            time.sleep(2)
        else:
            results['execute_trajectory'] = False
        
        results['get_current_state_after'] = self.test_get_current_state()
        time.sleep(1)
        
        results['plan_trajectory_joints'] = self.test_plan_trajectory(use_joints=True)
        time.sleep(1)
        
        results['move_to_pose_cartesian'] = self.test_move_to_pose(use_joints=False, velocity_factor=0.3, acceleration_factor=0.3)
        time.sleep(2)
        
        results['move_to_pose_joints'] = self.test_move_to_pose(use_joints=True, velocity_factor=0.3, acceleration_factor=0.3)
        time.sleep(1)
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("测试结果摘要")
        self.get_logger().info("=" * 60)
        for test_name, result in results.items():
            if result is None:
                status = "SKIP"
            elif result:
                status = "✓ PASS"
            else:
                status = "✗ FAIL"
            self.get_logger().info(f"  {test_name:30s}: {status}")
        self.get_logger().info("=" * 60 + "\n")
        
        passed = sum(1 for r in results.values() if r is True)
        total = sum(1 for r in results.values() if r is not None)
        self.get_logger().info(f"总计: {passed}/{total} 测试通过")
        
        return results

def main(args=None):
    rclpy.init(args=args)
    
    tester = ServiceTester()
    results = {}
    
    try:
        results = tester.run_comprehensive_test()
    except KeyboardInterrupt:
        tester.get_logger().info("测试被用户中断")
        results = {}
    except Exception as e:
        tester.get_logger().error(f"测试失败，异常: {str(e)}")
        results = {}
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    if not results:
        return 1
    passed = sum(1 for r in results.values() if r is True)
    total = sum(1 for r in results.values() if r is not None)
    return 0 if passed == total else 1

if __name__ == '__main__':
    sys.exit(main())


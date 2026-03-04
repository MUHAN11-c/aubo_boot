#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from demo_interface.srv import GetCurrentState, MoveToPose, PlanTrajectory, ExecuteTrajectory, SetSpeedFactor
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import sys
import argparse

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        
        self.current_pose = Pose()
        self.current_pose.position.x = -0.07401805371046066
        self.current_pose.position.y = -0.20905423164367676
        self.current_pose.position.z = 0.9532700777053833
        self.current_pose.orientation.x = 0.7025718092918396
        self.current_pose.orientation.y = -0.00014736213779542595
        self.current_pose.orientation.z = -0.0008002749527804554
        self.current_pose.orientation.w = 0.711612343788147
        
        self.get_logger().info("Service Tester initialized")
        self.get_logger().info(f"Test pose: x={self.current_pose.position.x:.6f}, "
                              f"y={self.current_pose.position.y:.6f}, "
                              f"z={self.current_pose.position.z:.6f}")
        
    def test_get_current_state(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("Testing GetCurrentState service...")
        
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
                    self.get_logger().info("✓ GetCurrentState: SUCCESS")
                    self.get_logger().info(f"  Joint positions (rad): {response.joint_position_rad}")
                    self.get_logger().info(f"  Cartesian position: x={response.cartesian_position.position.x:.6f}, "
                                          f"y={response.cartesian_position.position.y:.6f}, "
                                          f"z={response.cartesian_position.position.z:.6f}")
                    self.get_logger().info(f"  Message: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"✗ GetCurrentState: FAILED - {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ GetCurrentState: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ GetCurrentState: TIMEOUT")
            return False
    
    def test_set_speed_factor(self, factor=0.5):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Testing SetSpeedFactor service (factor={factor})...")
        
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
                    self.get_logger().info("✓ SetSpeedFactor: SUCCESS")
                    self.get_logger().info(f"  Message: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"✗ SetSpeedFactor: FAILED - {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ SetSpeedFactor: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ SetSpeedFactor: TIMEOUT")
            return False
    
    def test_plan_trajectory(self, use_joints=False):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Testing PlanTrajectory service (use_joints={use_joints})...")
        
        client = self.create_client(PlanTrajectory, '/plan_trajectory')
        
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("PlanTrajectory service not available")
            return None
        
        request = PlanTrajectory.Request()
        request.target_pose = self.current_pose
        request.use_joints = use_joints
        
        self.get_logger().info("  Sending planning request...")
        future = client.call_async(request)
        self.get_logger().info("  Waiting for planning to complete (timeout: 60s)...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ PlanTrajectory: SUCCESS")
                    self.get_logger().info(f"  Planning time: {response.planning_time:.4f} seconds")
                    self.get_logger().info(f"  Trajectory points: {len(response.trajectory.points)}")
                    self.get_logger().info(f"  Message: {response.message}")
                    return response.trajectory
                else:
                    self.get_logger().error(f"✗ PlanTrajectory: FAILED - {response.message}")
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
        self.get_logger().info("Testing ExecuteTrajectory service...")
        
        client = self.create_client(ExecuteTrajectory, '/execute_trajectory')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory service not available")
            return False
        
        request = ExecuteTrajectory.Request()
        request.trajectory = trajectory
        
        future = client.call_async(request)
        self.get_logger().info("  Waiting for trajectory execution to complete...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ ExecuteTrajectory: SUCCESS")
                    self.get_logger().info(f"  Message: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"✗ ExecuteTrajectory: FAILED - Error code: {response.error_code}, Message: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ ExecuteTrajectory: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ ExecuteTrajectory: TIMEOUT")
            return False
    
    def test_move_to_pose(self, use_joints=False, velocity_factor=0.5, acceleration_factor=0.5):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Testing MoveToPose service (use_joints={use_joints}, "
                              f"velocity_factor={velocity_factor}, acceleration_factor={acceleration_factor})...")
        
        client = self.create_client(MoveToPose, '/move_to_pose')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveToPose service not available")
            return False
        
        request = MoveToPose.Request()
        request.target_pose = self.current_pose
        request.use_joints = use_joints
        request.velocity_factor = velocity_factor
        request.acceleration_factor = acceleration_factor
        
        future = client.call_async(request)
        self.get_logger().info("  Waiting for movement to complete...")
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info("✓ MoveToPose: SUCCESS")
                    self.get_logger().info(f"  Message: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"✗ MoveToPose: FAILED - Error code: {response.error_code}, Message: {response.message}")
                    return False
            except Exception as e:
                self.get_logger().error(f"✗ MoveToPose: EXCEPTION - {str(e)}")
                return False
        else:
            self.get_logger().error("✗ MoveToPose: TIMEOUT")
            return False
    
    def run_all_tests(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("Starting Service Interface Tests")
        self.get_logger().info("=" * 60 + "\n")
        
        results = {}
        
        results['get_current_state'] = self.test_get_current_state()
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
        
        results['plan_trajectory_joints'] = self.test_plan_trajectory(use_joints=True)
        time.sleep(1)
        
        results['move_to_pose_cartesian'] = self.test_move_to_pose(use_joints=False, velocity_factor=0.3, acceleration_factor=0.3)
        time.sleep(2)
        
        results['move_to_pose_joints'] = self.test_move_to_pose(use_joints=True, velocity_factor=0.3, acceleration_factor=0.3)
        time.sleep(1)
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("Test Results Summary")
        self.get_logger().info("=" * 60)
        for test_name, result in results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            self.get_logger().info(f"  {test_name:30s}: {status}")
        self.get_logger().info("=" * 60 + "\n")
        
        passed = sum(1 for r in results.values() if r)
        total = len(results)
        self.get_logger().info(f"Total: {passed}/{total} tests passed")
        
        return results

def main(args=None):
    parser = argparse.ArgumentParser(
        description='Test ROS2 demo_driver services',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all tests
  python3 test_services.py

  # Run a single test
  python3 test_services.py --test get_current_state
  python3 test_services.py --test plan_trajectory --use-joints
  python3 test_services.py --test execute_trajectory
  python3 test_services.py --test move_to_pose --use-joints --velocity-factor 0.3

  # List all available tests
  python3 test_services.py --list
        """
    )
    parser.add_argument(
        '--test', '-t',
        type=str,
        choices=['get_current_state', 'set_speed_factor', 'plan_trajectory', 
                 'execute_trajectory', 'move_to_pose', 'all'],
        default='all',
        help='Test to run (default: all)'
    )
    parser.add_argument(
        '--use-joints', '-j',
        action='store_true',
        help='Use joint space planning (for plan_trajectory and move_to_pose)'
    )
    parser.add_argument(
        '--velocity-factor', '-v',
        type=float,
        default=0.3,
        help='Velocity factor (0.0-1.0, default: 0.3)'
    )
    parser.add_argument(
        '--acceleration-factor', '-a',
        type=float,
        default=0.3,
        help='Acceleration factor (0.0-1.0, default: 0.3)'
    )
    parser.add_argument(
        '--speed-factor', '-s',
        type=float,
        default=0.5,
        help='Speed factor for set_speed_factor test (0.0-1.0, default: 0.5)'
    )
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='List all available tests and exit'
    )
    
    # Parse known args to avoid conflicts with ROS2
    ros_args = None
    if args is None:
        import sys
        args = sys.argv[1:]
    
    parsed_args, unknown = parser.parse_known_args(args)
    
    if parsed_args.list:
        print("\nAvailable tests:")
        print("  get_current_state    - Test GetCurrentState service")
        print("  set_speed_factor     - Test SetSpeedFactor service")
        print("  plan_trajectory      - Test PlanTrajectory service")
        print("  execute_trajectory    - Test ExecuteTrajectory service (requires plan_trajectory first)")
        print("  move_to_pose         - Test MoveToPose service")
        print("  all                  - Run all tests (default)")
        print("\nOptions:")
        print("  --use-joints         - Use joint space planning")
        print("  --velocity-factor    - Velocity scaling factor (0.0-1.0)")
        print("  --acceleration-factor - Acceleration scaling factor (0.0-1.0)")
        print("  --speed-factor       - Speed factor for set_speed_factor test")
        return 0
    
    # Initialize ROS2 with unknown args
    rclpy.init(args=unknown)
    
    tester = ServiceTester()
    results = {}
    success = False
    
    try:
        if parsed_args.test == 'all':
            results = tester.run_all_tests()
            success = all(results.values()) if results else False
        elif parsed_args.test == 'get_current_state':
            success = tester.test_get_current_state()
        elif parsed_args.test == 'set_speed_factor':
            success = tester.test_set_speed_factor(parsed_args.speed_factor)
        elif parsed_args.test == 'plan_trajectory':
            trajectory = tester.test_plan_trajectory(use_joints=parsed_args.use_joints)
            success = trajectory is not None
            if success:
                tester.get_logger().info(f"\n✓ PlanTrajectory test completed successfully")
                tester.get_logger().info(f"  Trajectory saved, you can use it with execute_trajectory test")
            else:
                tester.get_logger().error(f"\n✗ PlanTrajectory test failed")
        elif parsed_args.test == 'execute_trajectory':
            tester.get_logger().info("ExecuteTrajectory test: planning trajectory first...")
            trajectory = tester.test_plan_trajectory(use_joints=parsed_args.use_joints)
            if trajectory:
                time.sleep(1)
                success = tester.test_execute_trajectory(trajectory)
            else:
                tester.get_logger().error("Failed to plan trajectory, cannot execute")
                success = False
        elif parsed_args.test == 'move_to_pose':
            success = tester.test_move_to_pose(
                use_joints=parsed_args.use_joints,
                velocity_factor=parsed_args.velocity_factor,
                acceleration_factor=parsed_args.acceleration_factor
            )
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
        success = False
    except Exception as e:
        tester.get_logger().error(f"Test failed with exception: {str(e)}")
        success = False
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main())


"""
入口编排 — mission_node：同一进程组合 MissionManager 与 PickExecutor.

console_scripts 入口（peach_motion.mission_node:main）；用多线程执行器：
PickExecutor 的 goal 流程与 MoveIt 客户端等待是阻塞式轮询，需并发线程
处理订阅/action 结果回调。
"""
from peach_motion.mission_manager import MissionManager
from peach_motion.pick_executor import PickExecutor
import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    """
    入口：rclpy 初始化 → MissionManager + PickExecutor 同进程 spin.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；KeyboardInterrupt 干净收尾.

    """
    rclpy.init(args=args)
    manager = MissionManager()
    executor_node = PickExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager)
    executor.add_node(executor_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        executor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

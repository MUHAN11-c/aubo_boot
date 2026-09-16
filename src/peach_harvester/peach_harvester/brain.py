"""peach_harvester 大脑进程：vision（看+建）与 supervisor（批）一进程三节点。

清洁重写轮 3b：ScenePerception / TargetReconstruction / TaskExecutor 三节点
对象进同一 MultiThreadedExecutor——进程合并（故障域统一、部署一件化），
图名零变化（节点名/话题名不动，lifecycle_manager 按名照常管理）。

并发语义与独立进程等价：各节点默认回调组互斥（节点内串行），跨节点可
并行；重活本就在各节点 worker 线程。节点保持 Unconfigured 启动，由
lifecycle_manager 按名单（场景→重建→调度）configure/activate。
"""
from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from peach_harvester.supervisor.executor_node import TaskExecutorNode
from peach_harvester.vision.scene_perception.scene_perception_node import (
    ScenePerceptionNode,
)
from peach_harvester.vision.target_reconstruction.target_reconstruction_node import (
    TargetReconstructionNode,
)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    nodes = []
    executor = MultiThreadedExecutor()
    try:
        nodes.append(ScenePerceptionNode())
        nodes.append(TargetReconstructionNode())
        nodes.append(TaskExecutorNode())
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    finally:
        for node in nodes:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

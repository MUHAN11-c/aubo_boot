"""
peach_core.ros — 唯一允许 import rclpy 的子包（纯核 guard 的显式豁免区）.

只放「纯核抽象 → ROS 运行时」的适配器（当前为时钟适配器）；算法与
数据层永远不得落在这里。
"""

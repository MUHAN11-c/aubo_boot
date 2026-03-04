#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GraspNet ROS 节点：接收点云话题，预测抓取位姿并发布。
功能：订阅 PointCloud2 → 预测抓取 → 碰撞检测 → NMS → 发布 Pose + MarkerArray
"""

import rospy  # ROS Python 客户端库
import ros_numpy  # ROS 消息与 numpy 数组转换工具

from utils.grasp import Grasp, GraspGroup  # 抓取数据结构：单条抓取与抓取组

from sensor_msgs.msg import PointCloud2  # ROS 点云消息类型
from visualization_msgs.msg import Marker, MarkerArray  # RViz 可视化标记消息
from geometry_msgs.msg import Pose, Vector3  # ROS 位姿与向量消息
from std_msgs.msg import ColorRGBA  # ROS 颜色消息（RGBA）

import time  # 时间相关（用于延迟）
import torch  # PyTorch 深度学习框架
import numpy as np  # 数值计算库
import open3d as o3d  # 3D 点云处理库
from scipy.spatial.transform import Rotation  # 旋转矩阵与四元数转换

from models.graspnet import GraspNet, pred_decode  # GraspNet 模型与预测解码函数
from utils.collision_detector import ModelFreeCollisionDetector  # 无模型碰撞检测器


class GNBServer:
    """GraspNet ROS 服务节点类：处理点云输入，预测并发布抓取位姿"""
    
    def __init__(self):
        """初始化：加载 ROS 参数、创建模型、订阅/发布话题"""
        # 从 ROS 参数服务器读取配置参数
        self.get_ros_param()
        
        # 创建 GraspNet 模型实例（与训练时配置一致）
        self.net = GraspNet(
            input_feature_dim=0,  # 点云特征维度：0 表示仅使用 xyz 坐标
            num_view=300,  # 多视角编码的视角数量（approach 方向采样数）
            num_angle=12,  # 平面内旋转角度离散化档位数（12 类对应 0°~165°，每 15° 一档）
            num_depth=4,  # 抓取深度档位数（4 个深度候选）
            cylinder_radius=0.05,  # 圆柱裁剪半径（米）：以种子点为轴、沿 approach 方向的圆柱半径
            hmin=-0.02,  # 圆柱下底面在局部 x 轴上的坐标（米）：抓取点后方 2 cm
            hmax_list=[0.01, 0.02, 0.03, 0.04],  # 4 个深度档位对应的圆柱上底面坐标（米）
            is_training=False,  # 推理模式：False 表示使用预测结果而非标签
        )
        
        # 设置计算设备：优先使用 GPU（cuda:0），否则使用 CPU
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # 将模型移动到指定设备（GPU 或 CPU）
        self.net.to(self.device)
        
        # 加载模型权重：从 checkpoint 文件读取训练好的参数
        checkpoint = torch.load(self.model_path)
        # 将权重加载到模型中
        self.net.load_state_dict(checkpoint["model_state_dict"])
        # 设置为评估模式（关闭 dropout、batch normalization 的训练行为）
        self.net.eval()

        # 订阅点云话题：当收到 PointCloud2 消息时，调用 self.predict_grasps 回调函数
        rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.predict_grasps)

        # 发布抓取位姿话题：发布 geometry_msgs/Pose 消息，队列大小为 1
        self.grasp_pub = rospy.Publisher(self.grasp_topic, Pose, queue_size=1)

        # 发布可视化标记话题：发布 MarkerArray 用于在 RViz 中显示夹爪
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        # 标记 ID 计数器：每次绘制新抓取时递增，避免 ID 冲突
        self.draw_grasp_id = 0

    def get_ros_param(self):
        """从 ROS 参数服务器读取配置参数（带默认值）"""
        # 模型权重文件路径（~ 表示私有命名空间，即 /gnb_server/model_path）
        self.model_path = rospy.get_param("~model_path", "../models/checkpoint-rs.tar")
        # 送入网络的最大点数：超过此数量会随机采样
        self.max_point_num = rospy.get_param("~max_point_num", 20000)
        # 碰撞检测阈值（米）：全局碰撞 IoU 超过此值则判定为碰撞
        self.collision_thresh = rospy.get_param("~collision_thresh", 0.01)
        # 碰撞检测前体素下采样的体素边长（米）：用于加速碰撞检测
        self.voxel_size = rospy.get_param("~voxel_size", 0.01)
        # 发布的最大抓取数量：NMS 和排序后取前 n 个
        self.max_grasps_num = rospy.get_param("~max_grasps_num", 20)

        # 订阅的点云话题名称
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "pointcloud")
        # 发布的抓取位姿话题名称
        self.grasp_topic = rospy.get_param("~grasp_topic", "grasps")
        # 发布的可视化标记话题名称
        self.marker_topic = rospy.get_param("~marker_topic", "marker")

    def get_points(
        self, data
    ) -> tuple[dict[str, torch.Tensor], o3d.utility.Vector3dVector]:
        """
        将 ROS PointCloud2 消息转换为模型输入格式。
        
        参数:
            data: sensor_msgs/PointCloud2，ROS 点云消息
            
        返回:
            end_points: dict，包含 'point_clouds' (torch.Tensor, shape=(1, N, 3))，在 DEVICE 上
            points_o3d: o3d.utility.Vector3dVector，完整点云（未采样），用于碰撞检测
        """
        # 将 ROS PointCloud2 消息转换为结构化 numpy 数组（字典形式，包含 x, y, z 等字段）
        pc = ros_numpy.numpify(data)
        
        # 提取 x, y, z 坐标并拼接成 (N, 3) 数组
        # reshape(-1, 1) 将一维数组转为列向量，axis=1 表示沿列方向拼接
        pc_np = np.concatenate(
            (pc["x"].reshape(-1, 1), pc["y"].reshape(-1, 1), pc["z"].reshape(-1, 1)),
            axis=1,
        ).astype(
            np.float32
        )  # 形状为 (N, 3)，N 为点云中的点数
        
        # 如果点数超过最大限制，进行随机采样
        if len(pc_np) > self.max_point_num:
            rospy.logdebug(f"点云数量{len(pc_np)}，随机采样{self.max_point_num}个点")
            # 无放回随机采样：从所有点中随机选择 max_point_num 个
            idxs = np.random.choice(len(pc_np), self.max_point_num, replace=False)
        else:
            # 点数不足时，使用全部点（不补足）
            rospy.logdebug(f"点云数量{len(pc_np)}")
            idxs = np.arange(len(pc_np))
        
        # 根据采样索引提取对应的点
        pc_np_sampled = pc_np[idxs]
        
        # 将采样后的点云转为 PyTorch tensor，并添加 batch 维度
        # np.newaxis 添加一个维度：(N, 3) -> (1, N, 3)
        # to(self.device) 将 tensor 移动到 GPU 或 CPU
        points_torch = torch.from_numpy(pc_np_sampled[np.newaxis]).to(
            self.device
        )  # 形状为 (1, N, 3)
        
        # 组装模型输入字典：point_clouds 是必需的输入
        end_points = {"point_clouds": points_torch}
        
        # 将完整点云（未采样）转为 Open3D 格式，用于后续碰撞检测
        # 注意：碰撞检测使用完整点云，网络输入使用采样后的点云
        points_o3d = o3d.utility.Vector3dVector(pc_np)
        
        return end_points, points_o3d

    def get_grasps(self, end_points):
        """
        执行 GraspNet 前向推理，将输出解码为 GraspGroup。
        
        参数:
            end_points: dict，模型输入（含 point_clouds）
            
        返回:
            gg: GraspGroup，预测的抓取组，每个抓取包含位姿、宽度、分数等
        """
        # 禁用梯度计算：推理时不需要反向传播，节省内存和计算
        with torch.no_grad():
            # 前向传播：模型输出原始预测（各角度/深度/宽度的分数等）
            end_points = self.net(end_points)
            # 解码：将原始输出转换为 6 自由度抓取位姿 + 宽度等，组成 (M, 17) 数组
            grasp_preds = pred_decode(end_points)
        
        # 取第一个 batch 的结果（通常 batch_size=1）
        # detach() 断开梯度连接，cpu() 转到 CPU，numpy() 转为 numpy 数组
        gg_array = grasp_preds[0].detach().cpu().numpy()
        
        # 封装为 GraspGroup 对象：gg_array 形状为 (M, 17)，M 为预测抓取数量
        gg = GraspGroup(gg_array)
        
        return gg

    def collision_detection(self, gg, cloud) -> GraspGroup:
        """
        无模型碰撞检测：过滤与场景点云碰撞的抓取。
        
        参数:
            gg: GraspGroup，待检测的抓取组
            cloud: np.ndarray，形状为 (N, 3)，场景点云坐标（单位：米）
            
        返回:
            GraspGroup，过滤掉碰撞抓取后的新抓取组
        """
        # 创建碰撞检测器：对场景点云进行体素下采样以加速检测
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        
        # 执行碰撞检测：
        # approach_dist=0.05：沿 approach 方向检查 5 cm 的距离
        # collision_thresh：全局碰撞 IoU 阈值，超过则判定为碰撞
        collision_mask = mfcdetector.detect(
            gg, approach_dist=0.05, collision_thresh=self.collision_thresh
        )
        
        # 使用布尔索引过滤：保留 collision_mask 为 False 的抓取（未碰撞）
        gg = gg[~collision_mask]
        
        # 记录日志：输出碰撞检测后剩余的抓取数量
        rospy.loginfo(f"碰撞检测后剩余{len(gg)}个")
        
        return gg

    def get_top_n(self, gg: GraspGroup, n):
        """
        对抓取组做 NMS、排序，返回前 n 个最优抓取。
        
        参数:
            gg: GraspGroup，待处理的抓取组
            n: int，返回的抓取数量
            
        返回:
            GraspGroup，前 n 个最优抓取
        """
        # 非极大值抑制：去除位置和姿态过于接近的重复抓取
        # 注意：nms() 返回新的 GraspGroup，需要接收返回值
        gg = gg.nms()
        rospy.loginfo(f"非极大值抑制后剩余{len(gg)}个")
        
        # 按分数降序排序：高分在前
        gg.sort_by_score()
        
        # 如果抓取数量超过 n，记录日志
        if len(gg) > n:
            rospy.loginfo(f"发布前{n}个")
        
        # 记录最高分抓取的分数（保留 3 位小数）
        rospy.loginfo(f"最高分{gg[0].score:.3f}")
        
        # 返回前 n 个抓取（切片操作）
        return gg[:n]

    def to_pose_msg(self, gg):
        """
        将 GraspGroup 转换为 ROS Pose 消息列表。
        
        坐标系转换说明：
        - GraspNet 定义的夹爪坐标系需要转换为机器人坐标系（沿 x 轴开合、沿 z 轴接近）
        - 使用旋转矩阵组合实现坐标系变换
        
        参数:
            gg: GraspGroup，待转换的抓取组
            
        返回:
            msgs: list of Pose，ROS 位姿消息列表
            q: list of float，抓取质量分数列表
        """
        # 将 graspnetAPI 中定义的夹爪轴方向转换为沿 x 轴开合、沿 z 轴接近的坐标系
        
        # 绕 z 轴旋转 90 度：[[0,-1,0],[1,0,0],[0,0,1]]
        rot_mat_z = Rotation.from_euler(
            "z", 90, degrees=True
        ).as_matrix()  # 旋转矩阵：绕 z 轴逆时针旋转 90°
        
        # 绕 y 轴旋转 90 度：[[0,0,1],[0,1,0],[-1,0,0]]
        rot_mat_y = Rotation.from_euler(
            "y", 90, degrees=True
        ).as_matrix()  # 旋转矩阵：绕 y 轴逆时针旋转 90°
        
        # 组合旋转：先绕 z 轴，再绕 y 轴
        # 结果矩阵：[[0,0,1],[1,0,0],[0,1,0]]
        rot_mat = rot_mat_y @ rot_mat_z
        
        # 初始化输出列表
        msgs = []  # 存储转换后的 Pose 消息
        q = []  # 存储抓取质量分数
        
        # 遍历每条抓取
        for i in range(len(gg)):
            # 获取第 i 条抓取
            grasp: Grasp = gg[i]
            
            # 记录该抓取的质量分数
            q.append(grasp.score)
            
            # 构造 4×4 齐次变换矩阵
            pose_mat = np.eye(4)  # 初始化为单位矩阵
            # 设置平移部分：抓取中心位置（相机坐标系，单位：米）
            pose_mat[:3, 3] = grasp.translation
            # 设置旋转部分：应用坐标系变换后的旋转矩阵
            pose_mat[:3, :3] = rot_mat @ grasp.rotation_matrix
            
            # 沿 approach 方向前移 depth 距离：将抓取中心移到夹爪闭合面（指尖）位置
            # 参考：https://github.com/graspnet/graspnet-baseline/issues/23#issuecomment-893119187
            pose_mat = forward_pose(pose_mat, grasp.depth)
            
            # 将变换矩阵转换为 ROS Pose 消息
            pose = matrix_to_pose_msg(pose_mat)
            msgs.append(pose)
        
        return msgs, q

    def vis_grasps(self, poses, f, qualities):
        """
        在 RViz 中可视化抓取（使用 MarkerArray）。
        
        参数:
            poses: list of Pose，抓取位姿列表
            f: str，坐标系名称（frame_id）
            qualities: list of float，抓取质量分数列表
        """
        # 清除之前的标记：发送 DELETEALL 消息
        clear_markers(self.marker_pub)
        
        # 定义颜色映射函数：根据分数 s 返回 (R, G, B, A)
        # s=0 时颜色为 (1,0,0,1) 红色（低分），s=1 时为 (0,1,0,1) 绿色（高分）
        cm = lambda s: tuple([float(1 - s), float(s), float(0), float(1)])
        
        # 分数归一化的最小值和最大值
        vmin = 0
        vmax = 1
        
        # 重置标记 ID 计数器
        self.draw_grasp_id = 0
        
        # 遍历每条抓取，逐个发布标记
        for pose, q in zip(poses, qualities):
            # 根据分数计算颜色：将分数归一化到 [0,1]，再映射到颜色
            color = cm((q - vmin) / (vmax - vmin))
            
            # 创建该抓取的标记（4 个圆柱体：左指、右指、手腕、手掌）
            markers = create_grasp_markers(f, pose, color, "grasp", self.draw_grasp_id)
            
            # 更新标记 ID：每个抓取占用 4 个 ID（左指、右指、手腕、手掌）
            self.draw_grasp_id += 4
            
            # 发布标记数组
            self.marker_pub.publish(MarkerArray(markers=markers))
            
            # 短暂延迟：避免一次发布太多标记导致 RViz 卡顿
            time.sleep(0.05)

    def pub_grasp(self, poses):
        """
        发布抓取位姿到 ROS 话题。
        
        参数:
            poses: list of Pose，抓取位姿消息列表
        """
        # 遍历每条抓取位姿，逐个发布
        for pose in poses:
            self.grasp_pub.publish(pose)

    def predict_grasps(self, data):
        """
        点云回调函数：接收点云 → 预测抓取 → 碰撞检测 → 发布。
        
        参数:
            data: sensor_msgs/PointCloud2，接收到的点云消息
        """
        # 步骤 1：将 ROS 点云消息转换为模型输入格式
        end_points, points_o3d = self.get_points(data)
        
        # 步骤 2：执行 GraspNet 推理，得到预测抓取
        gg = self.get_grasps(end_points)
        
        # 步骤 3：如果碰撞检测阈值 > 0，执行碰撞检测并过滤
        if self.collision_thresh > 0:
            # 将 Open3D Vector3dVector 转为 numpy 数组用于碰撞检测
            gg = self.collision_detection(gg, np.array(points_o3d))
        
        # 步骤 4：NMS、排序，取前 max_grasps_num 个最优抓取
        gg = self.get_top_n(gg, self.max_grasps_num)
        
        # 步骤 5：将抓取组转换为 ROS Pose 消息列表
        grasps, q = self.to_pose_msg(gg)
        
        # 步骤 6：在 RViz 中可视化抓取（发布 MarkerArray）
        self.vis_grasps(grasps, data.header.frame_id, q)
        
        # 步骤 7：发布抓取位姿到话题
        self.pub_grasp(grasps)


def clear_markers(pub):
    """
    清除 RViz 中的所有标记。
    
    参数:
        pub: rospy.Publisher，标记发布器
    """
    # 创建删除所有标记的消息
    delete = [Marker(action=Marker.DELETEALL)]
    # 发布删除消息
    pub.publish(MarkerArray(delete))


def create_grasp_markers(frame, pose: Pose, color, ns, id=0):
    """
    创建单个抓取的可视化标记（4 个圆柱体：左指、右指、手腕、手掌）。
    
    注意：抓取点位于指尖位置（pose 已通过 forward_pose 前移 depth）。
    
    参数:
        frame: str，坐标系名称
        pose: Pose，抓取位姿（指尖位置）
        color: tuple，颜色 (R, G, B, A)，范围 [0,1]
        ns: str，命名空间
        id: int，起始标记 ID
        
    返回:
        list of Marker，4 个标记（左指、右指、手腕、手掌）
    """
    # 将 ROS Pose 消息转换为 4×4 变换矩阵
    pose_mat = pose_msg_to_matrix(pose)
    
    # 夹爪几何参数（单位：米）
    w = 0.075  # 夹爪开口宽度：7.5 cm
    d = 0.05  # 夹爪深度：5 cm
    radius = 0.005  # 圆柱体半径：0.5 cm（用于可视化）

    # ========== 左指（左侧夹爪） ==========
    # 计算左指位置：在夹爪坐标系下，左指位于 (-w/2, 0, -d/2)
    # 使用齐次坐标 [x, y, z, 1] 进行变换
    left_point = np.dot(pose_mat, np.array([-w / 2, 0, -d / 2, 1]))
    # 复制位姿矩阵
    left_pose = pose_mat.copy()
    # 设置左指位置（平移部分）
    left_pose[:3, 3] = left_point[:3]
    # 设置圆柱体尺寸：半径为 radius，高度为 d（沿 z 轴）
    scale = [radius, radius, d]
    # 创建左指标记（圆柱体）
    left = create_marker(Marker.CYLINDER, frame, left_pose, scale, color, ns, id)

    # ========== 右指（右侧夹爪） ==========
    # 计算右指位置：在夹爪坐标系下，右指位于 (w/2, 0, -d/2)
    right_point = np.dot(pose_mat, np.array([w / 2, 0, -d / 2, 1]))
    right_pose = pose_mat.copy()
    right_pose[:3, 3] = right_point[:3]
    scale = [radius, radius, d]
    # 创建右指标记（ID 为 id+1）
    right = create_marker(Marker.CYLINDER, frame, right_pose, scale, color, ns, id + 1)

    # ========== 手腕（连接部分） ==========
    # 计算手腕位置：在夹爪坐标系下，手腕位于 (0, 0, -d*5/4)，即沿 z 轴向后 6.25 cm
    wrist_point = np.dot(pose_mat, np.array([0.0, 0.0, -d * 5 / 4, 1]))
    wrist_pose = pose_mat.copy()
    wrist_pose[:3, 3] = wrist_point[:3]
    # 手腕圆柱体高度为 d/2（2.5 cm）
    scale = [radius, radius, d / 2]
    # 创建手腕标记（ID 为 id+2）
    wrist = create_marker(Marker.CYLINDER, frame, wrist_pose, scale, color, ns, id + 2)

    # ========== 手掌（连接左右指的横梁） ==========
    # 计算手掌位置：在夹爪坐标系下，手掌位于 (0, 0, -d)
    palm_point = np.dot(pose_mat, np.array([0.0, 0.0, -d, 1]))
    palm_pose = pose_mat.copy()
    palm_pose[:3, 3] = palm_point[:3]
    
    # 手掌需要绕 y 轴旋转 90 度，使其沿 x 轴方向（连接左右指）
    rot = np.eye(4)  # 4×4 单位矩阵
    # 设置旋转部分：绕 y 轴旋转 90 度（π/2 弧度）
    rot[:3, :3] = Rotation.from_rotvec([0, np.pi / 2, 0]).as_matrix()
    # 应用旋转：将旋转矩阵右乘到位姿矩阵
    palm_pose = np.dot(palm_pose, rot)
    # 手掌圆柱体尺寸：半径为 radius，高度为 w（沿 x 轴，连接左右指）
    scale = [radius, radius, w]
    # 创建手掌标记（ID 为 id+3）
    palm = create_marker(Marker.CYLINDER, frame, palm_pose, scale, color, ns, id + 3)

    # 返回 4 个标记的列表
    return [left, right, wrist, palm]


def create_marker(type, frame, pose, scale=[1, 1, 1], color=(1, 1, 1, 1), ns="", id=0):
    """
    创建单个 ROS Marker 消息。
    
    参数:
        type: int，标记类型（如 Marker.CYLINDER）
        frame: str，坐标系名称
        pose: np.ndarray，4×4 变换矩阵
        scale: list or scalar，标记尺寸 [x, y, z] 或标量（会转为 [s, s, s]）
        color: tuple，颜色 (R, G, B, A)，范围 [0,1]
        ns: str，命名空间
        id: int，标记 ID
        
    返回:
        Marker，ROS 标记消息
    """
    # 如果 scale 是标量，转为列表 [scale, scale, scale]
    if np.isscalar(scale):
        scale = [scale, scale, scale]
    
    # 创建 Marker 消息对象
    msg = Marker()
    # 设置坐标系名称
    msg.header.frame_id = frame
    # 设置时间戳为当前时间
    msg.header.stamp = rospy.Time()
    # 设置命名空间
    msg.ns = ns
    # 设置标记 ID
    msg.id = id
    # 设置标记类型（如圆柱体）
    msg.type = type
    # 设置动作为添加（ADD）
    msg.action = Marker.ADD
    # 将 4×4 变换矩阵转换为 ROS Pose 消息并设置到位姿
    msg.pose = matrix_to_pose_msg(pose)
    # 设置尺寸：Vector3(x, y, z)
    msg.scale = Vector3(*scale)
    # 设置颜色：ColorRGBA(r, g, b, a)
    msg.color = ColorRGBA(*color)
    
    return msg


def pose_msg_to_matrix(pose_msg: Pose):
    """
    将 ROS 的 Pose 消息转换为 4×4 齐次变换矩阵。
    
    参数:
        pose_msg: Pose，ROS 位姿消息
        
    返回:
        np.ndarray，4×4 变换矩阵
    """
    # 提取平移部分：位置 (x, y, z)
    translation = np.array(
        [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
    )
    
    # 提取旋转部分：四元数 (x, y, z, w)
    rotation = np.array(
        [
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ]
    )
    
    # 将四元数转换为旋转矩阵
    # from_quat 的参数顺序为 (x, y, z, w)
    rotation_matrix = Rotation.from_quat(rotation).as_matrix()
    
    # 构造 4×4 齐次变换矩阵
    transform_matrix = np.eye(4)  # 初始化为单位矩阵
    # 设置旋转部分（3×3）
    transform_matrix[:3, :3] = rotation_matrix[:3, :3]
    # 设置平移部分（3×1）
    transform_matrix[:3, 3] = translation
    
    return transform_matrix


def matrix_to_pose_msg(pose_mat):
    """
    将 4×4 变换矩阵转换为 ROS 的 Pose 消息。
    
    注意：变量名 translation 实际存储的是旋转四元数（代码中的命名有误，但逻辑正确）。
    
    参数:
        pose_mat: np.ndarray，4×4 变换矩阵
        
    返回:
        Pose，ROS 位姿消息
    """
    # 创建 Pose 消息对象
    pose = Pose()
    
    # 将旋转矩阵转换为四元数
    # as_quat() 返回 (x, y, z, w) 格式的四元数
    translation = Rotation.from_matrix(pose_mat[:3, :3]).as_quat()
    
    # 设置四元数到 orientation（注意：变量名 translation 实际是旋转四元数）
    pose.orientation.x = translation[0]
    pose.orientation.y = translation[1]
    pose.orientation.z = translation[2]
    pose.orientation.w = translation[3]
    
    # 设置平移部分到 position
    pose.position.x = pose_mat[0, 3]
    pose.position.y = pose_mat[1, 3]
    pose.position.z = pose_mat[2, 3]
    
    return pose


def forward_pose(pose_mat, length):
    """
    沿位姿的 z 轴方向前移指定距离。
    
    用于将抓取中心位置前移到夹爪闭合面（指尖）位置。
    
    参数:
        pose_mat: np.ndarray，4×4 变换矩阵
        length: float，前移距离（米）
        
    返回:
        np.ndarray，前移后的 4×4 变换矩阵
    """
    # 在局部坐标系下，z 轴方向为 (0, 0, length)，使用齐次坐标 [0, 0, length, 1]
    # 通过矩阵乘法变换到世界坐标系
    point = np.dot(pose_mat, np.array([0.0, 0.0, length, 1]))
    
    # 复制原变换矩阵
    new_pose_mat = pose_mat.copy()
    # 更新平移部分：设置为前移后的位置
    new_pose_mat[:3, 3] = point[:3]
    
    return new_pose_mat


if __name__ == "__main__":
    # 初始化 ROS 节点：节点名为 "gnb_server"，日志级别为 DEBUG
    rospy.init_node("gnb_server", log_level=rospy.DEBUG)
    
    # 创建 GNBServer 实例（会执行 __init__，加载模型、订阅/发布话题）
    p = GNBServer()
    
    # 打印就绪信息
    print("gnb_server就绪")
    
    # 进入 ROS 主循环：阻塞等待回调函数被调用
    rospy.spin()

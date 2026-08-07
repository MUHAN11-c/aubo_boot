"""
参数层 — PeachPoseNode 的 ROS 参数声明 / 加载 / 热调.

集中管理 DEFAULTS / DESCRIPTIONS / DYNAMIC 三张表；节点侧只调三个函数：
declare_params(node) 声明、load_params(node) 读成节点属性、
make_on_set_callback(node) 生成运行时热调回调。默认值与
config/peach_pose.yaml 对齐（yaml 可覆盖）。
"""
import numpy as np
from peach_pose_ros2.peach_pose.contracts import ToolGeometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration

# 参数默认值（与 config/peach_pose.yaml 对齐，可被 yaml 覆盖）
DEFAULTS = {
    'color_topic': '/camera/color/image_raw',
    'depth_topic': '/camera/depth/image_raw',
    'camera_info_topic': '/camera/color/camera_info',
    # 手眼链挂在 color optical；深度 registration 后几何也在此系
    'camera_optical_frame': 'camera_color_optical_frame',
    'output_frame': 'base_link',
    'tf_timeout_sec': 0.5,
    # Percipio 原始深度常需 ×0.25 才是毫米量级（再 /1000→米）
    'depth_scale_unit': 0.25,
    'sync_slop_s': 0.05,
    'min_detection_conf': 0.5,
    'yolo_conf': 0.3,
    'yolo_iou': 0.5,
    'sam_max_bboxes': 8,
    'sam_min_area': 100,
    'min_depth_m': 0.3,
    'max_depth_m': 2.5,
    'min_points': 100,
    'publish_debug_image': True,
    'publish_masks': True,
    'publish_detection_cloud': True,
    'detection_cloud_stride': 2,
    # 融合决策段（候选时序融合 + ROI 云重拟合；算法在 peach_pose/temporal_fuser|refiner）
    'refined_outputs_enabled': True,
    'recon_cloud_topic': '/recon_refiner_node/roi_cloud',
    'roi_size_m': 0.4,
    'recon_min_points': 3000,
    'recon_refined_period_sec': 1.0,
    'target_kind': 'bag',
    'yolo_model_path': 'model/best.pt',
    'sam_model_path': 'model/mobile_sam.pt',
    'model_version': 'yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f',
    'calibration_version':
        'percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006',
    # 空串 → 算法默认相机系 +Y；否则 "x,y,z" 重力提示
    'gravity_hint_xyz': '',
    'tool.D_inner': 0.104,
    'tool.L_insert': 0.200,
    'tool.L_blade': 0.025,
    'tool.entry_d_tool': 0.030,
    'tool.entry_d_s': 0.040,
    'tool.clearance_min': 0.005,
    'tool.margin_neck': 0.015,
    'tool.version': '1.1',
}

DESCRIPTIONS = {
    'color_topic': '彩色图话题（bgr8）',
    'depth_topic': '深度图话题（uint16，须与彩色图 registration 对齐）',
    'camera_info_topic': '彩色相机内参话题',
    'camera_optical_frame': '相机光学系 frame_id（手眼链所挂）；'
                            '空串则用深度图 header.frame_id',
    'output_frame': '输出坐标系：几何经 TF 变到此帧；空串=保持相机系',
    'tf_timeout_sec': 'TF 查询超时 (s)',
    'depth_scale_unit': '深度比例因子：raw × 本值 = 毫米量级（Percipio 常见 '
                        '0.25）；数据集回放（真毫米）设 1.0',
    'sync_slop_s': 'RGB-D 近似同步允差 (s)',
    'min_detection_conf': '检测置信度下限，低于该值的目标不入管线',
    'yolo_conf': 'YOLO 推理置信度阈值',
    'yolo_iou': 'YOLO NMS 的 IoU 阈值',
    'sam_max_bboxes': 'SAM 单次推理最多处理的检测框数',
    'sam_min_area': 'SAM 掩膜面积下限（像素），小于则丢弃',
    'min_depth_m': '有效深度下限 (m)，过近视为噪声',
    'max_depth_m': '有效深度上限 (m)，过远视为背景',
    'min_points': '有效前景点数下限，不足直接 REJECT',
    'publish_debug_image': '是否发布 debug 叠加图 (~/debug_image)',
    'publish_masks': '是否发布 SAM 掩膜图 (~/masks)',
    'publish_detection_cloud': '是否发布检测框内深度反投影彩色点云 '
                               '(~/detection_cloud)',
    'detection_cloud_stride': '点云降采样步长（>1 减轻 RViz 负载）',
    'refined_outputs_enabled': '是否启用融合决策段（refined_candidates/'
                               'fusion_stats/refinement_evolution/recon_refined）',
    'recon_cloud_topic': 'ROI 点云话题（peach_recon 纯重建输出）',
    'roi_size_m': '感知侧 ROI 盒边长 (m)：以融合位置为中心自裁完整云',
    'recon_min_points': 'ROI 裁剪后达到该点数才重拟合（~/recon_refined）',
    'recon_refined_period_sec': '~/recon_refined 发布周期 (s)',
    'target_kind': "重拟合类别：'bag' 圆柱 | 'fruit' 球",
    'yolo_model_path': 'YOLO 权重；空/相对路径相对包目录解析，可给目录',
    'sam_model_path': 'MobileSAM 权重；解析规则同 yolo_model_path',
    'model_version': '模型版本标识（随结果发布，便于追溯）',
    'calibration_version': '内外参版本标识（内参 color_camera_info.yaml；'
                           '外参 hand_eye/active.yaml）',
    'gravity_hint_xyz': '重力方向提示 "x,y,z"（相机系）；'
                        '空串=算法默认相机系 +Y',
    'tool.D_inner': '工具圆柱内径 (m)，袋子必须能通过',
    'tool.L_insert': '最大插入深度 (m)',
    'tool.L_blade': '刀刃平面到圆柱入口平面的距离 (m)',
    'tool.entry_d_tool': '入口 standoff 的工具分量 (m)',
    'tool.entry_d_s': '入口 standoff 的安全裕量分量 (m)',
    'tool.clearance_min': '袋体与工具内壁的最小径向余量 (m)',
    'tool.margin_neck': '袋颈前方的安全停止距离 (m)',
    'tool.version': '工具几何配置的语义版本号',
}

# 可运行时热调的参数（rqt_reconfigure 可编辑）；其余为结构性参数，
# 声明为只读（改它们需要重建订阅/模型/工具，重启才生效）
DYNAMIC = {
    'min_detection_conf', 'yolo_conf', 'yolo_iou',
    'sam_max_bboxes', 'sam_min_area',
    'min_depth_m', 'max_depth_m', 'min_points',
    'publish_debug_image', 'publish_masks',
    'publish_detection_cloud', 'detection_cloud_stride',
}


def declare_params(node):
    """
    在 node 上声明全部参数（DYNAMIC 之外一律 read_only）.

    Args:
        node: PeachPoseNode 实例.

    Returns
    -------
        无返回值（None）；参数声明进 node.

    """
    for k, v in DEFAULTS.items():
        node.declare_parameter(
            k, v, ParameterDescriptor(description=DESCRIPTIONS[k],
                                      read_only=k not in DYNAMIC))


def load_params(node):
    """
    从参数服务器读出并缓存为 node 实例属性；重力串解析失败则抛错.

    Args:
        node: PeachPoseNode 实例（属性名与参数名一一对应）.

    Returns
    -------
        无返回值（None）；结果写入 node 属性（含 node.tool / node.gravity_hint）.

    """
    g = node.get_parameter
    node.color_topic = g('color_topic').get_parameter_value().string_value
    node.depth_topic = g('depth_topic').get_parameter_value().string_value
    node.camera_info_topic = g('camera_info_topic').get_parameter_value().string_value
    node.camera_optical_frame = g(
        'camera_optical_frame').get_parameter_value().string_value.strip()
    node.output_frame = g('output_frame').get_parameter_value().string_value.strip()
    node.tf_timeout = Duration(seconds=float(g('tf_timeout_sec').value))
    node.depth_scale_unit = float(g('depth_scale_unit').value)
    node.sync_slop_s = float(g('sync_slop_s').value)
    node.min_detection_conf = float(g('min_detection_conf').value)
    node.yolo_conf = float(g('yolo_conf').value)
    node.yolo_iou = float(g('yolo_iou').value)
    node.sam_max_bboxes = max(1, int(g('sam_max_bboxes').value))
    node.sam_min_area = max(1, int(g('sam_min_area').value))
    node.min_depth_m = float(g('min_depth_m').value)
    node.max_depth_m = float(g('max_depth_m').value)
    node.min_points = max(1, int(g('min_points').value))
    node.publish_debug_image = bool(g('publish_debug_image').value)
    node.publish_masks = bool(g('publish_masks').value)
    node.publish_detection_cloud = bool(g('publish_detection_cloud').value)
    node.detection_cloud_stride = max(1, int(g('detection_cloud_stride').value))
    node.refined_outputs_enabled = bool(g('refined_outputs_enabled').value)
    node.recon_cloud_topic = g('recon_cloud_topic').get_parameter_value().string_value
    node.roi_size_m = float(g('roi_size_m').value)
    node.recon_min_points = max(1, int(g('recon_min_points').value))
    node.recon_refined_period_sec = float(g('recon_refined_period_sec').value)
    node.target_kind = g('target_kind').get_parameter_value().string_value.strip()
    node.yolo_model_path = g('yolo_model_path').get_parameter_value().string_value
    node.sam_model_path = g('sam_model_path').get_parameter_value().string_value
    node.model_version = g('model_version').get_parameter_value().string_value
    node.calibration_version = g(
        'calibration_version').get_parameter_value().string_value
    gh_s = g('gravity_hint_xyz').get_parameter_value().string_value.strip()
    if gh_s:
        parts = [float(x) for x in gh_s.split(',')]
        if len(parts) != 3:
            raise ValueError(
                f'gravity_hint_xyz needs 3 comma-separated floats, got {gh_s!r}')
        node.gravity_hint = np.asarray(parts, dtype=float)
    else:
        node.gravity_hint = None
    # entry_standoff = 刀具伸出 + 安全间隙，与 contracts.ToolGeometry 一致
    node.tool = ToolGeometry(
        D_inner=float(g('tool.D_inner').value),
        L_insert=float(g('tool.L_insert').value),
        L_blade=float(g('tool.L_blade').value),
        entry_d_tool=float(g('tool.entry_d_tool').value),
        entry_d_s=float(g('tool.entry_d_s').value),
        entry_standoff=float(g('tool.entry_d_tool').value)
        + float(g('tool.entry_d_s').value),
        clearance_min=float(g('tool.clearance_min').value),
        margin_neck=float(g('tool.margin_neck').value),
        version=str(g('tool.version').value),
    )


def make_on_set_callback(node):
    """
    生成运行时参数变更回调（把新值应用到节点成员与引擎/管线实例）.

    结构性参数（话题/坐标系/模型路径/工具等）已声明 read_only，不会到这；
    回调内只处理可热调的阈值/开关/门限。

    Args:
        node: PeachPoseNode 实例（回调闭包捕获）.

    Returns
    -------
        on_set(params) 回调：全部接受（unknown 名忽略），返回 SetParametersResult.

    """
    def _on_params_set(params):
        for p in params:
            name, value = p.name, p.value
            if name in ('yolo_conf', 'yolo_iou'):
                setattr(node, name, float(value))
                setattr(node.engine, f'_{name}', float(value))
            elif name in ('sam_max_bboxes', 'sam_min_area'):
                setattr(node, name, max(1, int(value)))
                setattr(node.engine, f'_{name}', getattr(node, name))
            elif name in ('min_depth_m', 'max_depth_m', 'min_points'):
                v = float(value) if name != 'min_points' else max(1, int(value))
                setattr(node, name, v)
                # 袋/果两条管线共用同一组门限，同步更新
                setattr(node.estimator.pipeline, name, v)
                setattr(node.estimator.fruit_pipeline, name, v)
            elif name in ('min_detection_conf', 'publish_debug_image',
                          'publish_masks', 'publish_detection_cloud',
                          'detection_cloud_stride'):
                setattr(node, name, value)
            else:
                continue
            node.get_logger().info(f'参数已更新: {name} = {value}')
        return SetParametersResult(successful=True)

    return _on_params_set

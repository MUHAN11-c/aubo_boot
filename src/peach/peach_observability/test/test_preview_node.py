"""节点接线必须独立录制，并从真实工具 TF 缓存导出只读字段."""
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from peach_observability.observability_node import ObservabilityNode
from peach_observability.state import ObservabilityState


def node_stub():
    """不构造 ROS context 的节点窄接口夹具."""
    node = ObservabilityNode.__new__(ObservabilityNode)
    node._params = SimpleNamespace(record_enabled=False, record_save_images=False,
                                   topics={})
    node._image_preview = None
    node._tool_frames = {}
    node._debug_bridge = None
    node._stop_recording = lambda: None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_500_000_000))
    return node


def test_preview_subscriptions_without_recording():
    """录制关闭仍各订阅一条；录制开图像亦不能再添第二订户."""
    for recording in (False, True):
        node = node_stub()
        node._params.record_enabled = recording
        node._params.record_save_images = recording
        node._params.record_save_clouds = False
        subscriptions = []
        node._topic = lambda key: key
        node._subscribe = lambda kind, topic, cb, qos: subscriptions.append((topic, cb))
        for callback in ('_targets_callback', '_harvest_callback',
                         '_recon_status_callback', '_recon_diagnostics_callback',
                         '_recon_debug_callback', '_recon_decision_callback',
                         '_refined_pose_callback', '_refined_axis_callback',
                         '_refined_diagnostics_callback', '_manipulation_callback',
                         '_grasp_hypothesis_callback', '_task_executor_callback',
                         '_events_callback', '_robot_status_callback',
                         '_joint_state_callback', '_joint_status_callback',
                         '_diagnostics_callback', '_rosout_callback'):
            setattr(node, callback, lambda _msg: None)
        node._create_subscriptions()
        for topic in ('debug_image_topic', 'debug_image_raw_topic'):
            assert sum(name == topic for name, _cb in subscriptions) == 1


def test_same_callback_previews_and_conditionally_records():
    """预览常开。稳定识别图可进袋；即时检测图只在 all 档进袋."""
    node = node_stub()
    received, recorded = [], []
    node._image_preview = SimpleNamespace(submit=lambda *args: received.append(args))
    node._record_raw = lambda *args: recorded.append(args)
    message = object()
    callback = node._image_cb('raw', 'debug_image_raw_topic')
    callback(message)
    assert received == [('raw', message)] and not recorded
    node._params.record_enabled = node._params.record_save_images = True
    node._params.record_level = 'std'
    callback(message)
    assert recorded == []
    stable = node._image_cb('perception', 'debug_image_topic')
    stable(message)
    assert recorded == [('debug_image_topic', message)]
    node._params.record_level = 'all'
    callback(message)
    assert recorded[-1] == ('debug_image_raw_topic', message)


def test_tool_frames_real_tf_fields_and_missing_stale():
    """查询 base_link←工具三帧，HTTP 读取缓存，缺失和陈旧明示."""
    node = node_stub()
    queried = []

    def lookup(base, frame, stamp):
        from tf2_ros import TransformException
        queried.append((base, frame))
        if frame == 'sleeve_mouth':
            raise TransformException('missing')
        tf = TransformStamped()
        tf.header.stamp.sec = 10 if frame == 'tool_axis' else 8
        tf.transform.translation.x = 1
        tf.transform.rotation.w = 1
        return tf

    node._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    node._sample_tool_frames()
    assert queried == [('base_link', name) for name in
                       ('tool_axis', 'sleeve_mouth', 'cutting_plane')]
    frames = node._tool_frames_now()
    assert frames['tool_axis']['orientation'] == [0., 0., 0., 1.]
    assert frames['tool_axis']['position'] == [1., 0., 0.]
    assert frames['tool_axis']['available'] is True
    assert frames['tool_axis']['frame_id'] == 'base_link'
    assert frames['sleeve_mouth']['available'] is False
    assert frames['sleeve_mouth']['error'].startswith('tf_unavailable:')
    assert frames['cutting_plane']['error'] == 'stale'
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=15_000_000_000))
    assert node._tool_frames_now()['tool_axis']['error'] == 'stale'
    assert len(queried) == 3


def test_snapshot_images_and_release_worker():
    """快照携带两流元数据；cleanup 关闭缓存并销毁图像订阅."""
    node = node_stub()
    node._state = ObservabilityState()
    node._params = None
    node._debug_bridge = None
    node._stop_recording = lambda: None
    node._logs = []
    import threading
    node._log_lock = threading.Lock()
    assert set(node.snapshot()['images']) == {'perception', 'raw'}


def test_cleanup_closes_worker_and_destroys_subscriptions():
    """清理时收掉唯一 JPEG 工作线程及原订阅，重复收尾安全."""
    from peach_observability.image_preview import ImagePreview
    node = node_stub()
    preview = ImagePreview()
    node._image_preview = preview
    node._sweep_thread = node._startup_thread = node._selfcheck = None
    node._traj_timer = node._tcp_path_pub = node._tcp_marker_pub = None
    node._tf_listener = node._param_timer = None
    node._param_clients = {}
    node._param_inflight = {}
    node._fsm_tracker = node._arm_tracker = SimpleNamespace(reset=lambda: None)
    node._subs = [object(), object()]
    destroyed = []
    node.destroy_subscription = destroyed.append
    node._release_resources()
    assert len(destroyed) == 2
    assert not preview.worker_alive
    assert node._image_preview is None and not node._subs
    node._release_resources()


def test_zero_tool_stamp_not_reported_fresh():
    """只有静态零时间工具链不能伪造动态 robot TF 的新鲜度."""
    node = node_stub()
    node._tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: TransformStamped())
    node._sample_tool_frames()
    assert all(row['error'] == 'unknown_stamp' and not row['available']
               for row in node._tool_frames_now().values())


def test_http_trajectory_includes_tool_frames_without_querying_tf():
    """工具缓存输出挂入真实 HTTP trajectory 入口，入口不查询 tf2."""
    node = node_stub()
    node._params = None
    node._tcp_path = None
    node._tcp_viz_bundle = lambda *_args: ([], [], [])
    node._landmarks_now = lambda: {}
    payload = node.trajectory()
    assert set(payload['tool_frames']) == {'tool_axis', 'sleeve_mouth', 'cutting_plane'}
    assert payload['frame_id'] == 'base_link'


def test_all_recording_excludes_shared_image_subscribers():
    """全量发现跳过已共享的两流，避免双订阅双落盘."""
    from peach_observability.catch_all_recorder import CatchAllRecorder
    recorder = CatchAllRecorder(None, None, level='all',
                                excluded_topics={'/debug_image', '/debug_image_raw'})
    assert not recorder._allow('/debug_image')
    assert not recorder._allow('/debug_image_raw')
    assert recorder._allow('/camera/color/image_raw')
    node = node_stub()
    node._params.record_enabled = True
    node._params.record_level = 'all'
    recorded = []
    node._record_raw = lambda *args: recorded.append(args)
    message = object()
    node._image_cb('raw', 'debug_image_raw_topic')(message)
    assert recorded == [('debug_image_raw_topic', message)]


def test_tool_frames_ros_clock_backwards():
    """缓存 TF 时间领先当前 ROS 时钟时不得继续报新鲜."""
    node = node_stub()
    node._tool_frames = {'tool_axis': {
        'available': True, 'position': [1., 0., 0.],
        'orientation': [0., 0., 0., 1.], 'stamp_sec': 11.,
        'frame_id': 'base_link', 'age_s': None, 'error': ''}}
    row = node._tool_frames_now()['tool_axis']
    assert not row['available'] and row['error'] == 'clock_mismatch'
    assert row['age_s'] == -0.5

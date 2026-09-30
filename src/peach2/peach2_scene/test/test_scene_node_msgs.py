"""CollisionObject assembly of the node, exercised without rclpy.init (messages only)."""
from types import SimpleNamespace

from moveit_msgs.msg import CollisionObject
import numpy as np
from peach2_scene import scene_core as sc
from shape_msgs.msg import SolidPrimitive


def _result() -> sc.SceneResult:
    box = sc.HardObject(sc.KIND_BOX, np.array([0.5, 0.0, 0.8]), np.eye(3),
                        np.array([0.1, 0.05, 0.02]))
    cap = sc.HardObject(sc.KIND_CAPSULE, np.array([0.7, 0.1, 0.9]), np.eye(3),
                        np.array([0.02, 0.02, 0.15]))
    return sc.SceneResult([box, cap], np.zeros((0, 3)), False, {})


def test_remove_then_add_in_one_diff():
    # Imported here: rclpy / cv_bridge start threads at import, which would make the flake8
    # test's multiprocessing fork warn during collection.
    from peach2_scene.scene_node import OBJECT_PREFIX, SceneNode
    fake = SimpleNamespace(_params=SimpleNamespace(base_frame='base_link'))
    old = [f'{OBJECT_PREFIX}0000', f'{OBJECT_PREFIX}0001', f'{OBJECT_PREFIX}0002']
    objects, ids = SceneNode._collision_objects(fake, _result(), None, old)
    assert [o.operation for o in objects] == [CollisionObject.REMOVE] * 3 + \
        [CollisionObject.ADD] * 2
    assert [o.id for o in objects[:3]] == old
    assert ids == [f'{OBJECT_PREFIX}0000', f'{OBJECT_PREFIX}0001']
    assert all(o.header.frame_id == 'base_link' for o in objects)
    box, cap = objects[3], objects[4]
    assert [p.type for p in box.primitives] == [SolidPrimitive.BOX]
    assert np.allclose(box.primitives[0].dimensions, [0.2, 0.1, 0.04])
    assert box.pose.orientation.w == 1.0
    assert [p.type for p in cap.primitives] == [SolidPrimitive.CYLINDER, SolidPrimitive.SPHERE,
                                                SolidPrimitive.SPHERE]
    assert np.allclose(cap.primitives[0].dimensions, [0.3, 0.02])
    ends = sorted(p.position.z for p in cap.primitive_poses[1:])
    assert np.allclose(ends, [0.75, 1.05])
    assert len(cap.primitive_poses) == len(cap.primitives)


def test_object_prefix_is_the_srv_contract_constant():
    from peach2_interfaces.srv import BuildSceneSnapshot
    from peach2_scene.scene_node import OBJECT_PREFIX, SceneNode
    assert OBJECT_PREFIX == BuildSceneSnapshot.Request.HARD_OBJECT_PREFIX == 'peach_scene_hard_'
    fake = SimpleNamespace(_params=SimpleNamespace(base_frame='base_link'))
    _, ids = SceneNode._collision_objects(fake, _result(), None, [])
    assert all(i.startswith(BuildSceneSnapshot.Request.HARD_OBJECT_PREFIX) for i in ids)
    resp = BuildSceneSnapshot.Response()
    resp.truncated, resp.n_frames = True, 3
    assert resp.frame_stamp.sec == 0 and resp.n_frames == 3 and resp.truncated


def test_primitive_constants_match_shape_msgs():
    from peach2_scene import primitives
    assert primitives.BOX == SolidPrimitive.BOX
    assert primitives.SPHERE == SolidPrimitive.SPHERE
    assert primitives.CYLINDER == SolidPrimitive.CYLINDER
    assert SolidPrimitive.CYLINDER_HEIGHT == 0 and SolidPrimitive.CYLINDER_RADIUS == 1

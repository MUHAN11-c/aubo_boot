import numpy as np
from peach2_scene import scene_core as sc
from peach2_scene.primitives import BOX, CYLINDER, object_primitives, SPHERE
import pytest
from scipy.spatial.transform import Rotation


def test_box_primitive_keeps_size_and_orientation():
    axes = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    obj = sc.HardObject(sc.KIND_BOX, np.array([1.0, 2.0, 3.0]), axes, np.array([0.1, 0.2, 0.3]))
    (p,) = object_primitives(obj)
    assert p.shape_type == BOX
    assert p.dimensions == pytest.approx((0.2, 0.4, 0.6))
    assert np.allclose(Rotation.from_quat(p.quat_xyzw).as_matrix(), axes)


def test_capsule_becomes_cylinder_plus_end_spheres():
    centers = sc.voxel_centers(np.column_stack([np.arange(6), np.arange(6), np.zeros(6)]), 0.03)
    obj = sc.enclosing_object(centers, 0.03)
    obj = sc.HardObject(sc.KIND_CAPSULE, obj.center, obj.axes, np.array([0.02, 0.02, 0.1]))
    cyl, s0, s1 = object_primitives(obj)
    assert cyl.shape_type == CYLINDER and s0.shape_type == SPHERE and s1.shape_type == SPHERE
    assert cyl.dimensions == pytest.approx((0.2, 0.02))
    z = Rotation.from_quat(cyl.quat_xyzw).as_matrix()[:, 2]
    assert np.allclose(z, obj.axes[:, 2])
    assert np.allclose(s0.position, obj.p0) and np.allclose(s1.position, obj.p1)
    assert np.linalg.norm(s1.position - s0.position) == pytest.approx(0.2)


def test_zero_length_capsule_is_sphere_and_unknown_kind_rejected():
    obj = sc.HardObject(sc.KIND_CAPSULE, np.zeros(3), np.eye(3), np.array([0.05, 0.05, 0.0]))
    (p,) = object_primitives(obj)
    assert p.shape_type == SPHERE and p.dimensions == (0.05,)
    with pytest.raises(ValueError):
        object_primitives(sc.HardObject('cone', np.zeros(3), np.eye(3), np.ones(3)))

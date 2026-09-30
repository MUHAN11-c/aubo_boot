"""
HardObject -> shape_msgs/SolidPrimitive-equivalent tuples (zero ROS).

shape_msgs has no capsule primitive: a capsule becomes one cylinder plus two end spheres inside
the same CollisionObject.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .scene_core import HardObject, KIND_BOX, KIND_CAPSULE

# Values of shape_msgs/SolidPrimitive type constants.
BOX = 1
SPHERE = 2
CYLINDER = 3

_EPS_LENGTH_M = 1e-6


@dataclass(frozen=True)
class Primitive:
    shape_type: int
    dimensions: tuple[float, ...]   # BOX (x, y, z) | SPHERE (r,) | CYLINDER (height, radius)
    position: np.ndarray            # (3,) base_link
    quat_xyzw: np.ndarray           # (4,)


def _quat(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_quat()


def object_primitives(obj: HardObject) -> list[Primitive]:
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    if obj.kind == KIND_BOX:
        size = tuple(float(2.0 * h) for h in obj.half_extents)
        return [Primitive(BOX, size, obj.center.copy(), _quat(obj.axes))]
    if obj.kind != KIND_CAPSULE:
        raise ValueError(f'unknown object kind {obj.kind!r}')
    r = obj.radius
    half = float(obj.half_extents[2])
    if half < _EPS_LENGTH_M:
        return [Primitive(SPHERE, (r,), obj.center.copy(), identity)]
    # CYLINDER is along its local +Z; obj.axes[:, 2] is the capsule axis.
    return [Primitive(CYLINDER, (2.0 * half, r), obj.center.copy(), _quat(obj.axes)),
            Primitive(SPHERE, (r,), obj.p0, identity),
            Primitive(SPHERE, (r,), obj.p1, identity)]

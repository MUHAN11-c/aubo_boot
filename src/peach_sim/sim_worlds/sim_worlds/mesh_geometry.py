"""Map measured legacy GLB coordinates to ROS bag-bottom and neck semantics."""

import math

# Measured from embedded POSITION data with scene-node transforms and separately
# verified by this host's gz-common5 MeshManager (2026-10-29). Values are metres.
# This applies only to this fingerprint; changing the asset requires remeasurement.
BAG_MESH_SHA256 = '303434b21af70d395782c227a04c455d56ab435e77ac83373850b28637be431c'
BAG_MESH_URI = 'model://sim_description/meshes/bagged_peach.glb'
BAG_RAW_MIN = (-0.038894493132829666, -0.03999999910593033, -0.03493685647845268)
BAG_RAW_MAX = (0.04197113215923309, 0.06249406933784485, 0.03405386954545975)

# The original Blender tube axis is raw +Y in the shipped Y-up GLB. The
# helix's two complete turns span Y=0.0202..0.0398; its tail extends farther.
# The neck anchor is the helix midplane, not the wire-with-tail bbox centre.
BAG_NECK_AXIS = 0.03
BAG_FRUIT_RAW_CENTER = (0.0, -0.00065, 0.0)
BAG_FRUIT_RAW_SIZE = (0.04946998320519924, 0.05302124843001366, 0.05099999159574509)

# Both shipped peach_tree_[ab].glb assets also have raw +Y up, with Ymin=0.
# Rx(+pi/2) restores Z up and retains their trunk-root origin at the ground.
# BUNKER is already Z-up and must never receive this correction.
TREE_VISUAL_RPY = (math.pi / 2.0, 0.0, 0.0)


def _positive(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must be a finite positive number') from exc
    if isinstance(value, bool) or not math.isfinite(number) or number <= 0.0:
        raise ValueError(f'{name} must be a finite positive number')
    return number


def bag_geometry(width, thickness, bottom_to_neck):
    """Return the visual transform and matching local geometry for one bag instance."""
    width = _positive(width, 'width')
    thickness = _positive(thickness, 'thickness')
    length = _positive(bottom_to_neck, 'bottom_to_neck')
    low, high = BAG_RAW_MIN, BAG_RAW_MAX
    # SDF scales in the mesh's own coordinates before applying visual pose:
    # raw XYZ means width / longitudinal axis / thickness for this old asset.
    scale = (width / (high[0] - low[0]),
             length / (BAG_NECK_AXIS - low[1]),
             thickness / (high[2] - low[2]))
    if not all(math.isfinite(value) for value in scale):
        raise ValueError('bag dimensions produce an unrepresentable mesh scale')
    sx, sy, sz = scale
    z_offset = -low[1] * sy
    # Rx(+pi/2): (x, y, z) -> (x, -z, y). Do not translate X or Y to
    # centre the asymmetric AABB: doing so moves the physical tube axis away
    # from bag_bottom/neck. Only the collision proxy follows its AABB centre.
    minimum = (low[0] * sx, -high[2] * sz, 0.0)
    maximum = (high[0] * sx, -low[2] * sz, high[1] * sy + z_offset)
    collision_size = tuple(b - a for a, b in zip(minimum, maximum))
    collision_center = tuple((a + b) / 2.0 for a, b in zip(minimum, maximum))
    if not all(math.isfinite(value) and value > 0.0 for value in collision_size):
        raise ValueError('bag dimensions produce an unrepresentable collision envelope')
    fruit_center = (BAG_FRUIT_RAW_CENTER[0] * sx,
                    -BAG_FRUIT_RAW_CENTER[2] * sz,
                    BAG_FRUIT_RAW_CENTER[1] * sy + z_offset)
    fruit_size = (BAG_FRUIT_RAW_SIZE[0] * sx,
                  BAG_FRUIT_RAW_SIZE[2] * sz,
                  BAG_FRUIT_RAW_SIZE[1] * sy)
    return {
        'visual_pose': (0.0, 0.0, z_offset, *TREE_VISUAL_RPY),
        'scale': scale,
        'visual_bounds': {'min': minimum, 'max': maximum},
        'collision_size': collision_size,
        'collision_center': collision_center,
        'bag_bottom': (0.0, 0.0, 0.0),
        'neck_center': (0.0, 0.0, length),
        'fruit_center': fruit_center,
        'fruit_ellipsoid_size': fruit_size,
    }

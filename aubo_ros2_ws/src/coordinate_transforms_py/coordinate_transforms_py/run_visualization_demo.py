#!/usr/bin/env python3
"""Non-ROS2 visualization: 2D (matplotlib) and optional 3D (matplotlib or open3d).
Output dir: COORD_TF_OUTPUT_DIR env or ./coord_tf_output."""
from coordinate_transforms_py.core import (
    CameraIntrinsics,
    project_3d_to_2d,
    project_3d_to_2d_batch,
    unproject_2d_to_3d,
    reprojection_error,
)
import numpy as np
import os

def _output_dir():
    return os.environ.get("COORD_TF_OUTPUT_DIR") or "coord_tf_output"

def main():
    out_dir = _output_dir()
    os.makedirs(out_dir, exist_ok=True)

    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    P_3d = np.array([[0.1, 0.0, 1.0], [0.0, 0.1, 1.0], [-0.05, -0.05, 0.8]])
    uv_proj = project_3d_to_2d_batch(P_3d, K)
    uv_obs = uv_proj + np.random.randn(*uv_proj.shape) * 3.0

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        ax.set_xlim(0, 640)
        ax.set_ylim(480, 0)
        ax.set_aspect('equal')
        ax.scatter(uv_obs[:, 0], uv_obs[:, 1], c='red', s=50, label='observed')
        ax.scatter(uv_proj[:, 0], uv_proj[:, 1], c='green', s=30, label='projected')
        for i in range(len(uv_obs)):
            ax.plot([uv_obs[i, 0], uv_proj[i, 0]], [uv_obs[i, 1], uv_proj[i, 1]], 'y-', alpha=0.7)
        ax.legend()
        ax.set_title('Reprojection (red=obs, green=proj, yellow=error)')
        out_path = os.path.join(out_dir, "coord_tf_visualization_2d.png")
        fig.savefig(out_path, dpi=100)
        print("Saved", out_path)
    except ImportError:
        print("matplotlib not found; install with: pip install matplotlib")

    errs = [reprojection_error(uv_obs[i], P_3d[i], K) for i in range(len(uv_obs))]
    print("Per-point reprojection errors (pixels):", errs)
    print("run_visualization_demo done.")

if __name__ == '__main__':
    main()

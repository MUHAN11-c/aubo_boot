#!/usr/bin/env python3
"""Optional: with OpenCV, read/draw 2D points and reprojection error, save PNG and process data YAML.
Output dir: COORD_TF_OUTPUT_DIR env or ./coord_tf_output."""
try:
    import cv2
except ImportError:
    print("opencv-python not found. Install with: pip install opencv-python")
    raise SystemExit(1)

import os
import numpy as np
from coordinate_transforms_py.core import (
    CameraIntrinsics,
    project_3d_to_2d,
    project_3d_to_2d_batch,
    reprojection_error,
)

def _output_dir():
    return os.environ.get("COORD_TF_OUTPUT_DIR") or "coord_tf_output"

def main():
    out_dir = _output_dir()
    os.makedirs(out_dir, exist_ok=True)

    # Dummy image (or pass path via argv)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)

    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    P_3d = np.array([[0.1, 0.0, 1.0], [0.0, 0.1, 1.0], [-0.05, -0.05, 0.8]])
    uv_proj = project_3d_to_2d_batch(P_3d, K)
    uv_obs = uv_proj + np.random.randn(*uv_proj.shape) * 3.0

    for i in range(len(uv_obs)):
        uo, vo = int(uv_obs[i, 0]), int(uv_obs[i, 1])
        up, vp = int(uv_proj[i, 0]), int(uv_proj[i, 1])
        cv2.circle(img, (uo, vo), 5, (0, 0, 255), -1)
        cv2.circle(img, (up, vp), 5, (0, 255, 0), -1)
        cv2.line(img, (uo, vo), (up, vp), (255, 255, 0), 1)

    png_path = os.path.join(out_dir, "coord_tf_reprojection.png")
    cv2.imwrite(png_path, img)
    print("Wrote", png_path, "(observed=red, projected=green, line=error)")

    yaml_path = os.path.join(out_dir, "coord_tf_process_data.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("fx", float(K.fx))
    fs.write("fy", float(K.fy))
    fs.write("cx", float(K.cx))
    fs.write("cy", float(K.cy))
    fs.write("points_3d", P_3d)
    fs.write("observed_uv", uv_obs)
    fs.write("projected_uv", uv_proj)
    fs.release()
    print("Wrote", yaml_path)

    errs = [reprojection_error(uv_obs[i], P_3d[i], K) for i in range(len(uv_obs))]
    print("Reprojection errors (pixels):", errs)
    print("run_opencv_process_demo done.")

if __name__ == "__main__":
    main()

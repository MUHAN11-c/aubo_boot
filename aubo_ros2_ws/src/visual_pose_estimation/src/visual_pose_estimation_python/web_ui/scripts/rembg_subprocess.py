#!/usr/bin/env python3
"""
独立的 rembg 处理脚本，在 conda 环境中运行。
通过命令行参数接收图像和 bbox，返回掩模和抠图。
"""

import sys
import json
import base64
import numpy as np
from PIL import Image
import io
import time

# 导入 rembg（在 conda 环境中）
try:
    from rembg import new_session, remove
    import onnxruntime as ort
    # 成功导入时不再写调试日志，只在标准输出报错
except ImportError as e:
    print(json.dumps({"error": f"无法导入 rembg: {e}"}), file=sys.stderr)
    sys.exit(1)


def process_roi(image_b64: str, bbox_json: str, model: str = "u2net", prefer_cuda: bool = True) -> dict:
    """处理 ROI 图像，返回掩模和抠图的 base64 编码"""
    try:
        # 解析输入（注意：这里的 bbox 第三个和第四个分量是宽高，而不是右下角坐标）
        bbox = json.loads(bbox_json)
        x, y, w, h = bbox["x0"], bbox["y0"], bbox["x1"], bbox["y1"]
        
        # 解码图像
        image_data = base64.b64decode(image_b64)
        img = Image.open(io.BytesIO(image_data))
        img_rgb = np.array(img.convert("RGB"))
        img_h, img_w = img_rgb.shape[:2]
        
        # 计算并裁剪 ROI（确保在图像范围内）
        x0 = max(0, int(x))
        y0 = max(0, int(y))
        x1 = min(img_w, int(x + w))
        y1 = min(img_h, int(y + h))
        if x1 <= x0 or y1 <= y0:
            raise ValueError(f"无效的 ROI 边界: {(x0, y0, x1, y1)} (原始 bbox={bbox})")
        
        roi = img_rgb[y0:y1, x0:x1]
        roi_pil = Image.fromarray(roi)
        
        # 初始化 session：强制使用 GPU（CUDA），如果不可用则直接报错
        available = ort.get_available_providers()
        if prefer_cuda:
            if "CUDAExecutionProvider" not in available:
                raise RuntimeError(
                    f"需要 CUDAExecutionProvider，但当前可用 providers={available}"
                )
            providers = ["CUDAExecutionProvider"]
        else:
            providers = ["CPUExecutionProvider"]
        session = new_session(model, providers=providers)
        
        # 处理 ROI
        roi_bytes = io.BytesIO()
        roi_pil.save(roi_bytes, format="PNG")
        roi_bytes.seek(0)
        
        result = remove(roi_bytes.read(), session=session)
        result_img = Image.open(io.BytesIO(result)).convert("RGBA")
        rgba = np.array(result_img)
        alpha = rgba[:, :, 3]
        
        # 生成全尺寸掩模（与裁剪后的 ROI 对齐）
        full_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        full_mask[y0:y1, x0:x1] = np.where(alpha > 127, 255, 0).astype(np.uint8)
        
        # 生成白底抠图
        rgb = rgba[:, :, :3].astype(np.float32)
        alpha_f = (alpha.astype(np.float32) / 255.0)[:, :, None]
        white = np.full_like(rgb, 255.0)
        composite_rgb = (alpha_f * rgb + (1.0 - alpha_f) * white).astype(np.uint8)
        composite_bgr = composite_rgb[:, :, ::-1]
        
        full_cutout = np.full((img_h, img_w, 3), 255, dtype=np.uint8)
        full_cutout[y0:y1, x0:x1] = composite_bgr
        
        # 编码结果
        mask_pil = Image.fromarray(full_mask)
        mask_bytes = io.BytesIO()
        mask_pil.save(mask_bytes, format="PNG")
        mask_b64 = base64.b64encode(mask_bytes.getvalue()).decode("utf-8")
        
        cutout_pil = Image.fromarray(full_cutout)
        cutout_bytes = io.BytesIO()
        cutout_pil.save(cutout_bytes, format="PNG")
        cutout_b64 = base64.b64encode(cutout_bytes.getvalue()).decode("utf-8")
        
        return {
            "success": True,
            "mask_b64": mask_b64,
            "cutout_b64": cutout_b64,
            "providers": providers,
        }
    except Exception as e:
        import traceback
        tb = traceback.format_exc()
        return {
            "success": False,
            "error": str(e),
            "traceback": tb,
        }


if __name__ == "__main__":
    if len(sys.argv) != 4:
        msg = {"error": "用法: rembg_subprocess.py <image_path> <bbox_json_path> <model>"}
        print(json.dumps(msg), file=sys.stderr)
        sys.exit(1)
    
    image_path = sys.argv[1]
    bbox_json_path = sys.argv[2]
    model = sys.argv[3]
    
    try:
        # 从文件读取图像并编码为 base64
        with open(image_path, 'rb') as f:
            image_data = f.read()
        image_b64 = base64.b64encode(image_data).decode("utf-8")
        
        # 从文件读取 bbox JSON
        with open(bbox_json_path, 'r') as f:
            bbox_json = f.read()
        
        result = process_roi(image_b64, bbox_json, model=model, prefer_cuda=True)
        print(json.dumps(result))
        sys.exit(0 if result["success"] else 1)
    except Exception as e:
        import traceback
        tb = traceback.format_exc()
        error_msg = {
            "success": False,
            "error": str(e),
            "traceback": tb
        }
        print(json.dumps(error_msg), file=sys.stderr)
        sys.exit(1)

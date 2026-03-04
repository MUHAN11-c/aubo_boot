#!/usr/bin/env python3
"""
诊断 CUBLAS_STATUS_INVALID_VALUE：检查 PyTorch/CUDA/cuBLAS 环境。
在 vision 包外也可直接运行: python3 check_cuda_cublas.py
"""
import sys
import subprocess

def run(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)

def main():
    print("========== 1. NVIDIA 驱动与运行时 ==========")
    r = run("nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null")
    if r.returncode == 0:
        print(r.stdout.strip())
    else:
        print("nvidia-smi 不可用")
    r = run("nvidia-smi 2>/dev/null | head -5")
    if r.returncode == 0:
        print(r.stdout)

    print("\n========== 2. PyTorch 与 CUDA 版本 ==========")
    try:
        import torch
        print("PyTorch:", torch.__version__)
        print("PyTorch 编译所用 CUDA (torch.version.cuda):", torch.version.cuda)
        print("cuDNN:", getattr(torch.backends.cudnn, "version", "N/A"))
        print("torch.cuda.is_available():", torch.cuda.is_available())
        if torch.cuda.is_available():
            print("当前 GPU:", torch.cuda.get_device_name(0))
    except Exception as e:
        print("导入 torch 失败:", e)
        return 1

    print("\n========== 3. 可能冲突的 nvidia-* 包 (pip) ==========")
    r = run("pip list 2>/dev/null | grep -i nvidia")
    if r.returncode == 0 and r.stdout.strip():
        print(r.stdout)
        print("  ^ 若存在 nvidia-cublas-cu* 且与 PyTorch 的 CUDA 不一致，可能触发 CUBLAS_STATUS_INVALID_VALUE")
    else:
        print("(未找到 nvidia-* 或 pip 不可用)")

    print("\n========== 4. 最小复现：GPU 上 attention 风格矩阵乘 ==========")
    try:
        import torch
        if not torch.cuda.is_available():
            print("CUDA 不可用，跳过")
        else:
            # 模拟 YOLO attention 里 q @ k 的 batched matmul
            device = "cuda"
            batch, heads, seq, dim = 1, 4, 64, 32
            q = torch.randn(batch, heads, seq, dim, device=device, dtype=torch.float32)
            k = torch.randn(batch, heads, seq, dim, device=device, dtype=torch.float32)
            scale = dim ** -0.5
            attn = (q @ k.transpose(-2, -1)) * scale
            torch.cuda.synchronize()
            print("  小规模 batched matmul 成功")
            # 再试一个更接近实际尺寸的
            batch, heads, seq, dim = 1, 8, 8400, 32  # 类似 YOLO 的序列长度量级
            q = torch.randn(batch, heads, seq, dim, device=device, dtype=torch.float32)
            k = torch.randn(batch, heads, seq, dim, device=device, dtype=torch.float32)
            attn = (q @ k.transpose(-2, -1)) * (dim ** -0.5)
            torch.cuda.synchronize()
            print("  较大规模 batched matmul 成功")
    except RuntimeError as e:
        print("  复现到错误:", e)
        return 1
    except Exception as e:
        print("  异常:", e)
        return 1

    print("\n========== 5. 建议 ==========")
    print("若上面出现 CUBLAS 错误或存在 nvidia-cublas-cu*：")
    print("  - 尝试: pip uninstall nvidia-cublas-cu11 nvidia-cublas-cu12 -y")
    print("  - 或安装与驱动匹配的 PyTorch: https://pytorch.org/get-started/locally/")
    print("若 PyTorch 为 cu128 但驱动较老：")
    print("  - 官方稳定版目前只到 cu126，可改用: pip install torch --index-url https://download.pytorch.org/whl/cu126")
    return 0

if __name__ == "__main__":
    sys.exit(main())

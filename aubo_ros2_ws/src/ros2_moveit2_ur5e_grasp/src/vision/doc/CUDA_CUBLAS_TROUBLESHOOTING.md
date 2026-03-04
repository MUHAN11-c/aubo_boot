# CUBLAS_STATUS_INVALID_VALUE 原因分析与排查

## 错误含义

`RuntimeError: CUDA error: CUBLAS_STATUS_INVALID_VALUE when calling cublasSgemmStridedBatched(...)` 出现在 YOLO 的 **attention 层**里的一次矩阵乘法（`q @ k`）时，说明是 **CUDA/cuBLAS 环境问题**，不是业务代码写错。

---

## 常见原因（按优先级）

### 1. **cuBLAS 库冲突（最常见）**

系统里单独安装了 `nvidia-cublas-cu11` 或 `nvidia-cublas-cu12` 等 pip 包，会覆盖或干扰 PyTorch 自带的 cuBLAS，导致版本/ABI 不一致，从而在部分矩阵运算里报 `CUBLAS_STATUS_INVALID_VALUE`。

**排查：**

```bash
pip list | grep -i nvidia
```

若看到 `nvidia-cublas-cu11`、`nvidia-cublas-cu12` 等，先尝试卸载与当前 PyTorch CUDA 版本不一致的：

```bash
pip uninstall nvidia-cublas-cu11 nvidia-cublas-cu12 -y
```

然后重新运行 `obj_detect`。很多人通过这一步即可解决。

---

### 2. **PyTorch 与系统 CUDA/驱动不匹配**

- `torch.version.cuda` 显示的是 **PyTorch 编译时用的 CUDA 版本**（例如 12.8）。
- 官方稳定版 PyTorch 目前只提供到 **CUDA 12.4/12.6** 等，没有正式的 cu128 稳定版；若你看到 12.8，多半是 nightly 或其它渠道的 cu128 构建。
- 若 **驱动较老**（`nvidia-smi` 右上角显示的 “CUDA Version” 低于 12.8），运行时可能在某些 kernel 上出现未定义行为或 CUBLAS 报错。

**建议：**

- 用 `nvidia-smi` 看驱动支持的 CUDA 版本。
- 使用与该驱动兼容的 PyTorch 构建，例如驱动支持 12.6 则用 cu126：

  ```bash
  pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
  ```

---

### 3. **特定 (m,n,k)/batch 触发的 cuBLAS 问题**

`cublasSgemmStridedBatched` 对某些维度组合或 stride 在部分驱动/cuBLAS 版本上有 bug，会直接报 `CUBLAS_STATUS_INVALID_VALUE`。YOLO 的 attention 里 batch=1、序列较长时，正好可能落到这些组合上。

这不是“模型错了”，而是 **当前环境下的 cuBLAS/驱动组合** 有问题。

---

### 4. **cuDNN 与 PyTorch 不匹配**

若系统里单独装了 cuDNN（例如 `/usr/lib` 下的 `libcudnn*`），且版本与 PyTorch 内置的不一致，也可能在部分算子上出错。优先使用 PyTorch 自带的库，避免系统级 cuDNN 覆盖。

---

## 推荐排查步骤

1. **运行诊断脚本（同一 Python 环境）：**

   ```bash
   cd /home/mu/IVG/aubo_ros2_ws/src/ros2_moveit2_ur5e_grasp/src/vision/scripts
   python3 check_cuda_cublas.py
   ```

   脚本会检查：驱动、PyTorch/CUDA 版本、pip 里的 nvidia-* 包、以及一次与 YOLO attention 类似的 GPU 矩阵乘是否报错。

2. **若脚本里出现 nvidia-cublas-cu*：**  
   按上面“原因 1”卸载冲突的 cublas 包后重跑仿真。

   **若卸载 nvidia-cublas-cu12 后出现 `import torch` 失败**（例如 `undefined symbol: __nvJitLinkCreate_12_8`）：  
   说明其余 pip 安装的 `nvidia-*` 与 PyTorch 自带的库混用、版本不一致。需**整堆重装**，让 pip 装回一套匹配的依赖：

   ```bash
   pip uninstall -y torch torchvision torchaudio
   pip uninstall -y nvidia-cublas-cu12 nvidia-cuda-cupti-cu12 nvidia-cuda-nvrtc-cu12 nvidia-cuda-runtime-cu12 nvidia-cudnn-cu12 nvidia-cufft-cu12 nvidia-curand-cu12 nvidia-cusolver-cu12 nvidia-cusparse-cu12 nvidia-cusparselt-cu12 nvidia-nvjitlink-cu12 nvidia-nvtx-cu12 2>/dev/null
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
   ```
   然后重新运行诊断脚本；若仍报 CUBLAS，可改用 cu126：`--index-url https://download.pytorch.org/whl/cu126`。

   **若重装 cu128 后 nvidia-cublas-cu12 又被装回且仍报 CUBLAS：**  
   说明 PyTorch 2.10+cu128 自带的 cublas 组合在你当前 GPU/驱动上有 bug。建议**改用 cu126**（与驱动 12.8 兼容，且更稳定）：

   ```bash
   pip uninstall -y torch torchvision torchaudio
   pip uninstall -y nvidia-cublas-cu12 nvidia-cuda-cupti-cu12 nvidia-cuda-nvrtc-cu12 nvidia-cuda-runtime-cu12 nvidia-cudnn-cu12 nvidia-cufft-cu12 nvidia-cufile-cu12 nvidia-curand-cu12 nvidia-cusolver-cu12 nvidia-cusparse-cu12 nvidia-cusparselt-cu12 nvidia-nccl-cu12 nvidia-nvjitlink-cu12 nvidia-nvshmem-cu12 nvidia-nvtx-cu12 2>/dev/null
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
   ```
   然后重新运行诊断脚本确认第 4 步通过。

3. **若脚本里“最小复现”也报 CUBLAS 错误：**  
   说明是环境问题。可尝试：
   - 换用与驱动匹配的 PyTorch（如 cu124/cu126），或
   - 在另一台机器/容器中用相同 PyTorch 版本验证。

4. **需要更精确的出错位置时：**

   ```bash
   CUDA_LAUNCH_BLOCKING=1 ros2 launch ur_bringup simulation.launch.py
   ```

   这样 CUDA 会同步执行，堆栈会指向真正触发 CUBLAS 调用的那一行（通常是 ultralytics 的 attention 里）。

---

## 总结

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `torch.cuda.is_available()` 为 True 但推理时报 CUBLAS | cuBLAS 库冲突 | 卸载 pip 的 nvidia-cublas-cu* |
| PyTorch 显示 cuda 12.8，驱动较老 | 构建 CUDA 与驱动不匹配 | 换用 cu124/cu126 等与驱动一致的 PyTorch |
| 仅 YOLO 推理时报错，小脚本也报错 | 驱动/cuBLAS 对某类维度有问题 | 更新驱动或换 PyTorch 版本 |

根本原因是 **当前 PyTorch 所用的 CUDA/cuBLAS 与系统里其它 CUDA 相关库或驱动不一致**，而不是 `obj_detect` 或 YOLO 模型本身写错；按上面顺序排查即可定位。

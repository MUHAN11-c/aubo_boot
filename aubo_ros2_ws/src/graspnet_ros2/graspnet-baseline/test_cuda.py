import torch

# 1. 检查 CUDA 是否可用
print(f"CUDA 是否可用: {torch.cuda.is_available()}")

# 2. 查看可用的 GPU 数量
if torch.cuda.is_available():
    print(f"可用 GPU 数量: {torch.cuda.device_count()}")
    
    # 3. 查看当前使用的 GPU 索引
    print(f"当前使用的 GPU 索引: {torch.cuda.current_device()}")
    
    # 4. 查看 GPU 名称
    print(f"GPU 名称: {torch.cuda.get_device_name(0)}")
    
    # 5. 测试张量在 GPU 上的运算
    # 创建一个张量并移到 GPU
    x = torch.tensor([1.0, 2.0, 3.0]).cuda()
    print(f"\nGPU 上的张量: {x}")
    print(f"张量所在设备: {x.device}")
    
    # 测试运算
    y = x * 2
    print(f"GPU 上的运算结果: {y}")
else:
    print("未检测到可用的 CUDA 设备，请检查驱动或 PyTorch 安装")
# ADR 0006：保持独立进程，不做 Python composition

## 决策

scene/reconstruction/supervisor 继续独立进程；scene 使用 SingleThreadedExecutor + bounded worker，reconstruction/supervisor 使用 MultiThreadedExecutor。

## 理由

- Python GIL 下合并进程不会带来零拷贝收益，反而扩大故障域。
- Nav2 composition 是可选优化，不是包边界要求。
- 当前真实链路已证明独立进程可工作；Orin profile 只需换 backend/参数，不需要改进程模型。

## 后果

- 跨进程只走 ROS 2；大图不跨主机。
- 未来若 camera→perception 零拷贝成为瓶颈，另立专项评估。

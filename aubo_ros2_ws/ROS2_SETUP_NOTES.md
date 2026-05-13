# ROS 2 环境搭建问题记录

## 1. rosdep init 失败：DNS 污染

**现象**：`sudo rosdep init` 报 `Connection refused`，无法连接 `raw.githubusercontent.com`

**原因**：DNS（systemd-resolved）将 `raw.githubusercontent.com` 解析到 `0.0.0.0`，为 GFW DNS 污染

**修复**：添加 `/etc/hosts` 记录绕过 DNS 污染

```bash
sudo bash -c 'cat >> /etc/hosts << EOF
185.199.108.133 raw.githubusercontent.com
185.199.109.133 raw.githubusercontent.com
185.199.110.133 raw.githubusercontent.com
185.199.111.133 raw.githubusercontent.com
EOF'
```

## 2. apt 代理不生效

**原因**：`sudo` 默认不传递用户环境变量（如 `http_proxy`、`all_proxy`），apt 下载不走代理

**方案 A** — 使用 `-E` 保留环境变量：

```bash
sudo -E apt install <package>
```

**方案 B** — 换国内镜像源（推荐）：

```bash
# ROS 2 源换清华镜像
sudo sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|' /etc/apt/sources.list.d/ros2.sources
```

## 3. colcon 未找到

**现象**：`colcon build` 报 "未找到命令"

**原因**：ROS 2 base 不自带 colcon

**修复**：

```bash
sudo apt install python3-colcon-common-extensions
```

## 4. ROS 包名不符合命名规范

**现象**：`rosdep install` 报 `Package name "xxx" does not follow naming conventions`

**原因**：ROS 包名只能包含小写字母、数字和下划线，不能有中文、空格或特殊字符

**修复**：编辑 `package.xml`，将 `<name>` 改为合法命名，如 `coffee_machine_urdf`

## 环境信息

- OS: Ubuntu 22.04 (Jammy)
- ROS: Humble
- 代理: Clash `127.0.0.1:7890`
- DNS: systemd-resolved (`127.0.0.53`)

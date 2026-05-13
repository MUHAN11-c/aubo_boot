#!/bin/bash
# ============================================================================
# protobuf 2.6.1 源码编译脚本（零基础可读）
#
# 背景：
#   奥比(Aubo)机械臂的 libauborobotcontroller.so 是预编译的闭源库，
#   它依赖 libprotobuf.so.9（protobuf 2.6.x 版本）。
#   Ubuntu 22.04 自带的是 libprotobuf.so.23（protobuf 3.12.x），
#   两个版本 ABI（Application Binary Interface，应用程序二进制接口）不兼容，
#   所以必须自己编译一份 protobuf 2.6.1 放到项目里。
#
# Linux 共享库命名规则（soname）:
#   libprotobuf.so        → 链接名（linker name），编译时用 -lprotobuf 就会找它
#   libprotobuf.so.9      → soname，运行时查找的名字，大版本号不变则 ABI 兼容
#   libprotobuf.so.9.0.1  → 真实名（real name），包含完整版本号的实际文件
#
# 用法（首次在新电脑上构建前执行一次即可）:
#   cd third_party
#   bash build_protobuf.sh
# ============================================================================

# set -e: 任何命令返回非零退出码（即出错）就立即停止脚本，
# 避免错误被忽略导致后续操作在错误状态下继续执行
set -e

# ---------------------------------------------------------------------------
# 1. 确定路径
# ---------------------------------------------------------------------------

# $0 = 脚本自身的路径（如 ./build_protobuf.sh 或 /path/to/build_protobuf.sh）
# dirname "$0" 取出目录部分，cd 进去，pwd 得到绝对路径
# 这样无论从哪里执行脚本，都能正确定位文件
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# SCRIPT_DIR/.. 是 aubo_driver_ros2 根目录
# 编译产物最终要放到 lib/lib64/protobuf/ 下
PROJECT_LIB="$(cd "$SCRIPT_DIR/.." && pwd)/lib/lib64/protobuf"

# protobuf 2.6.1 源码包，放在脚本同级目录
# 如果 git 仓库里没包含这个文件（太大或被 .gitignore 忽略），自动下载
TARBALL="$SCRIPT_DIR/protobuf-2.6.1.tar.gz"
TARBALL_URL="https://github.com/protocolbuffers/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz"

# $$ = 当前 shell 进程 PID，拼到目录名里避免多实例冲突
# /tmp 下的文件重启后系统会清理
BUILD_DIR="/tmp/protobuf_build_$$"

# ---------------------------------------------------------------------------
# 2. 检查源码包是否存在，缺失则自动下载
# ---------------------------------------------------------------------------

# -s 检查文件存在且大小 > 0（比 -f 多一层非空校验，防止下载到一半的残废文件）
if [ ! -s "$TARBALL" ]; then
    echo "[DOWNLOAD] 源码包缺失，自动下载 protobuf 2.6.1..."
    echo "            $TARBALL_URL"

    # wget: 非交互式下载工具
    #   -q  安静模式，不打印进度条（出错时仍会显示错误信息）
    #   -O  指定输出文件名（Output），不用重定向
    #   --show-progress  覆盖 -q 的安静效果，显示下载进度但不刷屏
    #   --timeout=30     连接超时 30 秒，防止卡死
    #   --tries=3        失败重试 3 次，应对网络抖动
    #
    # 备选下载源（GitHub 有时被墙）:
    #   如果第一个 URL 失败，wget 返回非零，手动换镜像重试
    if ! wget -q --show-progress --timeout=30 --tries=3 \
         -O "$TARBALL" "$TARBALL_URL" 2>&1; then

        # 备选：ghproxy 镜像（国内加速）
        MIRROR_URL="https://ghproxy.com/$TARBALL_URL"
        echo "[DOWNLOAD] 主源失败，尝试镜像: $MIRROR_URL"
        # 镜像也失败的话不要立即退出（set -e 会触发），用 || true 吞掉错误
        # 后面还有统一的结果校验
        wget -q --show-progress --timeout=30 --tries=3 \
             -O "$TARBALL" "$MIRROR_URL" || true
    fi

    # 下载后再校验一次，确保文件完整
    if [ ! -s "$TARBALL" ]; then
        echo ""
        echo "============================================"
        echo "  错误: 无法下载 protobuf-2.6.1.tar.gz"
        echo ""
        echo "  请手动下载后放到:"
        echo "    $SCRIPT_DIR/"
        echo ""
        echo "  下载地址:"
        echo "    $TARBALL_URL"
        echo "============================================"
        exit 1
    fi
    echo "[DOWNLOAD] 下载完成"
else
    echo "[CHECK] 源码包已存在: $TARBALL"
fi

# ---------------------------------------------------------------------------
# 3. 检查是否已经编译过（幂等性）
# ---------------------------------------------------------------------------

# -f 检查文件是否存在且为普通文件（会跟随软链接，检查目标文件）喵~
# 如果 .so.9 已存在说明之前编译过了，直接跳过
if [ -f "$PROJECT_LIB/libprotobuf.so.9" ]; then
    echo "[SKIP] libprotobuf.so.9 已存在（可能是软链接指向真实文件），无需重复编译"
    exit 0
fi

# ---------------------------------------------------------------------------
# 4. 解压源码
# ---------------------------------------------------------------------------

echo "[1/5] 解压 protobuf 2.6.1 源码..."

# mkdir -p: 递归创建目录，已存在也不报错
mkdir -p "$BUILD_DIR"

# tar:
#   x = extract（解压）
#   z = gzip（处理 .gz 压缩）
#   f = file（指定文件名）
#   -C = 解压到指定目录
tar xzf "$TARBALL" -C "$BUILD_DIR"

# cd 到解压后的源码目录
cd "$BUILD_DIR"/protobuf-2.6.1

# ---------------------------------------------------------------------------
# 5. 配置（configure）
# ---------------------------------------------------------------------------

# autotools 三步曲的第一步: ./configure
#   它会检测当前系统的编译器、库、头文件等，生成适合的 Makefile
#
# 参数说明:
#   --prefix=DIR      安装目标路径，make install 时会把文件复制到这里
#                     我们用 /tmp 下的临时路径，因为最终只要 .so 文件
#   --enable-shared   编译动态库（.so），这是默认行为，显式写出更清晰
#   --disable-static  不编译静态库（.a），我们不需要，节省时间
echo "[2/5] 配置编译选项（检测系统环境）..."
./configure --prefix="$BUILD_DIR/install" --enable-shared --disable-static

# ---------------------------------------------------------------------------
# 6. 编译
# ---------------------------------------------------------------------------

# make 读取 Makefile 编译源码
#   -j$(nproc): 并行编译，nproc 返回 CPU 核心数
#   比如 8 核就 make -j8，大幅加快编译速度
echo "[3/5] 编译中（使用 $(nproc) 个并行任务）..."
make -j$(nproc)

# make install 把编译产物复制到 --prefix 指定的目录
#   库文件 → $prefix/lib/
#   头文件 → $prefix/include/
make install

# ---------------------------------------------------------------------------
# 7. 复制 .so 文件到项目 lib 目录（一个真实文件 + 两个软链接）
# ---------------------------------------------------------------------------

# 为什么需要三个名字？
#
#   编译时（linker）: g++ ... -lprotobuf
#     → 链接器在库搜索路径里找 libprotobuf.so（链接名）
#
#   运行时（dynamic loader）: 程序启动时 ld.so 读取 ELF 的 NEEDED 段
#     → libauborobotcontroller.so 的 NEEDED 写死了 "libprotobuf.so.9"
#     → 动态链接器搜索 libprotobuf.so.9（soname）
#
#   所以三个名字都要有，但只需要一个真实文件，另外两个用软链接即可：
#   libprotobuf.so.9.0.1 → 真实文件（cp）
#   libprotobuf.so.9     → 软链接 → 真实文件（ln -s，运行时用）
#   libprotobuf.so       → 软链接 → 真实文件（ln -s，编译时用）
#   动态链接器和编译器都会自动 dereference 软链接，功能完全等价喵~
echo "[4/5] 复制库文件到项目目录..."

mkdir -p "$PROJECT_LIB"
cd "$PROJECT_LIB"

# --- libprotobuf（完整版）---
# 复制真实文件（真实名）
cp "$BUILD_DIR/install/lib/libprotobuf.so.9.0.1" .
# soname 和链接名用软链接即可，动态链接器和编译器都会自动 dereference
ln -sf libprotobuf.so.9.0.1 libprotobuf.so.9      # 运行时查找用（NEEDED 段写死这个名字）
ln -sf libprotobuf.so.9.0.1 libprotobuf.so        # 编译链接时用（-lprotobuf 搜索这个名字）

# --- libprotobuf-lite（精简版，protobuf 的轻量子集）---
# 有些库可能链接 lite 版本，一并带上
cp "$BUILD_DIR/install/lib/libprotobuf-lite.so.9.0.1" .
ln -sf libprotobuf-lite.so.9.0.1 libprotobuf-lite.so.9
ln -sf libprotobuf-lite.so.9.0.1 libprotobuf-lite.so

# ---------------------------------------------------------------------------
# 8. 清理临时文件
# ---------------------------------------------------------------------------

# rm -rf: 递归强制删除（r=recursive, f=force 不提示确认）
# 编译产生的中间文件（.o 等）不需要保留，删掉释放磁盘空间
rm -rf "$BUILD_DIR"

echo ""
echo "============================================"
echo "  完成! libprotobuf.so.9 已就绪"
echo "  位置: $PROJECT_LIB"
echo "============================================"

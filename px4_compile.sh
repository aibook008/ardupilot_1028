#!/bin/bash

# 设置 PX4 工具链环境变量
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

# 进入 PX4 目录（根据你的实际路径修改）
PX4_DIR="$HOME/PX4-Autopilot"

if [ ! -d "$PX4_DIR" ]; then
    echo "错误: PX4 目录不存在: $PX4_DIR"
    echo "请修改此脚本中的 PX4_DIR 变量"
    exit 1
fi

cd "$PX4_DIR"

# 显示当前工具链
echo "========================================="
echo "PX4 编译环境"
echo "========================================="
arm-none-eabi-g++ --version | head -1
echo "========================================="
echo ""

# 执行 make 命令
make "$@"

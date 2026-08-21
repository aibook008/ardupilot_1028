#!/bin/bash

# 设置 ArduPilot 工具链环境变量
export PATH="/opt/gcc-arm-none-eabi-13.3/bin:$PATH"

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 进入 ArduPilot 目录
cd "$SCRIPT_DIR"

# 显示当前工具链
echo "========================================="
echo "ArduPilot 编译环境"
echo "========================================="
arm-none-eabi-g++ --version | head -1
echo "========================================="
echo ""

# 执行 waf 命令
./waf "$@"

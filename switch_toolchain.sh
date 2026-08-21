#!/bin/bash

# ArduPilot 工具链路径
ARDUPILOT_TOOLCHAIN="/opt/gcc-arm-none-eabi-13.3/bin"

# PX4 工具链路径
PX4_TOOLCHAIN="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin"

case "$1" in
    "ap"|"ardupilot")
        export PATH="$ARDUPILOT_TOOLCHAIN:${PATH}"
        echo "✓ 已切换到 ArduPilot 工具链 (GCC 13.3)"
        echo "  路径: $ARDUPILOT_TOOLCHAIN"
        ;;
    "px4")
        export PATH="$PX4_TOOLCHAIN:${PATH}"
        echo "✓ 已切换到 PX4 工具链 (GCC 9.3.1)"
        echo "  路径: $PX4_TOOLCHAIN"
        ;;
    *)
        echo "用法: source switch_toolchain.sh [ap|px4]"
        echo ""
        echo "示例:"
        echo "  source switch_toolchain.sh ap      # 切换到 ArduPilot"
        echo "  source switch_toolchain.sh px4     # 切换到 PX4"
        echo ""
        echo "当前工具链:"
        arm-none-eabi-g++ --version | head -1
        return 1
        ;;
esac

# 显示当前版本
arm-none-eabi-g++ --version | head -1

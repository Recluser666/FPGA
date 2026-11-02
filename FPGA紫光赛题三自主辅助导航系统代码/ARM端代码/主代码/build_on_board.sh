#!/usr/bin/env bash
# ============================================================================
#  在 ARM 板子上直接编译（不需要交叉编译工具链）
#
#  前提：板子跑 Debian/Ubuntu，装了 gcc + make
#        sudo apt install -y build-essential
#
#  为什么能这么干：
#    源码是纯可移植 C（只用 libc + pthread + math），
#    RKNN 相关只依赖 librknnrt.so + rknn_api.h，两个都在包里。
#    板子本身就是 aarch64 Linux，原生编译最省事，
#    省掉"装交叉工具链 + 处理 sysroot"这一整套麻烦。
#
#  用法（在板子上）：
#    cd adas_pkg && bash build_on_board.sh
# ============================================================================
set -u
cd "$(dirname "$0")"

echo "============================================================"
echo " ADAS 板端构建"
echo "   $(uname -a)"
echo "============================================================"
echo

# ---- 0) 检查工具链 ----
for t in gcc make; do
    if ! command -v $t >/dev/null 2>&1; then
        echo "❌ 缺少 $t。请先装： sudo apt update && sudo apt install -y build-essential"
        exit 1
    fi
done
echo "✅ gcc: $(gcc --version | head -1)"

# ---- 1) 找 RKNN 头文件与库 ----
RKNN_INC=""
for d in "$PWD/rknpu2/include" /usr/include/rknn /usr/local/include/rknn \
         /usr/include; do
    [ -f "$d/rknn_api.h" ] && { RKNN_INC="$d"; break; }
done
[ -z "$RKNN_INC" ] && { echo "❌ 找不到 rknn_api.h（应随包放在 rknpu2/include/）"; exit 1; }
echo "✅ rknn_api.h: $RKNN_INC"

RKNN_LIB=""
for d in "$PWD/rknpu2/lib" /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/lib /opt/adas/lib; do
    ls "$d"/librknnrt.so >/dev/null 2>&1 && { RKNN_LIB="$d"; break; }
done
[ -z "$RKNN_LIB" ] && { echo "❌ 找不到 librknnrt.so（应随包放在 rknpu2/lib/）"; exit 1; }
echo "✅ librknnrt.so: $RKNN_LIB"
echo

# ---- 2) 编译 ----
CC=${CC:-gcc}
STD="-std=gnu99"
WARN="-Wall -Wextra"
OPT="-O2 -mcpu=cortex-a55"      # RK3568 是 4×Cortex-A55；若报错会自动回退
LIBS="-L$RKNN_LIB -lrknnrt -lm -lpthread"

echo "=== 编译（第 1 次尝试：带 -mcpu=cortex-a55）==="
if $CC $STD $WARN $OPT -DHAVE_RKNN -I"$RKNN_INC" -I. \
       -o adas adas_main.c infer_rknn.c infer_pre.c postproc.c decision.c \
              frame_parse.c uart_tx.c display.c $LIBS 2>build.log; then
    echo "✅ 编译成功"
else
    echo "⚠️  带 -mcpu 失败，回退到通用编译"
    head -5 build.log | sed 's/^/     /'
    OPT="-O2"
    if ! $CC $STD $WARN $OPT -DHAVE_RKNN -I"$RKNN_INC" -I. \
            -o adas adas_main.c infer_rknn.c infer_pre.c postproc.c decision.c \
                   frame_parse.c uart_tx.c display.c $LIBS 2>build.log; then
        echo "❌ 编译失败："
        tail -25 build.log | sed 's/^/     /'
        exit 1
    fi
    echo "✅ 编译成功（通用优化）"
fi

echo
ls -lh adas | sed 's/^/   /'

# ---- 3) 冒烟测试（--no-infer，验证运行时不缺 .so）----
echo
echo "=== 冒烟测试：--no-infer（只验证动态库与链路逻辑）==="
export LD_LIBRARY_PATH="$RKNN_LIB:$PWD/rknpu2/lib:${LD_LIBRARY_PATH:-}"
if ./adas --model ./best.rknn --transport file:/dev/null \
          --pixfmt rgb565 --src 960x720 --frames 1 --no-infer 2>&1 | head -12; then
    echo "   （--no-infer 让推理那步跳过；看到参数表就说明程序能跑）"
fi

echo
echo "============================================================"
echo " 下一步：用真模型跑（接 HDMI 看画面）"
echo "------------------------------------------------------------"
echo "   # C 版（无画面，只打印统计）："
echo "   LD_LIBRARY_PATH=$RKNN_LIB ./adas --model ./best.rknn \\"
echo "       --transport file:/dev/null --pixfmt rgb565 --src 960x720 --frames 10"
echo
echo "   # Python 版（有实时画面 + FPS）★ 先跑这个看效果"
echo "   python3 board_demo.py --backend rknnlite --model ./best.rknn \\"
echo "       --video ./1.mp4"
echo "============================================================"

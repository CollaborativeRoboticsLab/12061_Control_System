#!/bin/sh
# 不用 PlatformIO,直接用 arm-none-eabi-gcc 编译。
# 这份脚本的作用主要是"留个底":下面这组编译参数是实际验证过能编出
# 24,116 字节固件的那一组,PlatformIO 里 platformio.ini 配的是同样的东西。
# Linux/macOS 上装了 gcc-arm-none-eabi 就能跑。
set -e
cd "$(dirname "$0")/.."
INC="-ISYSTEM/sys -ISYSTEM/delay -ISYSTEM/usart \
 -IHARDWARE/ADC -IHARDWARE/ENCODER -IHARDWARE/EXTI -IHARDWARE/KEY \
 -IHARDWARE/LED -IHARDWARE/MOTOR -IHARDWARE/OLED -IHARDWARE/TIMER \
 -IHARDWARE/DataScope_DP -IBALANCE/CHECK -IBALANCE/CONTROL -IBALANCE/show"
CF="-mcpu=cortex-m3 -mthumb -Os -ffunction-sections -fdata-sections \
 -DSTM32F10X_MD -std=gnu99 $INC"
OUT=build_gcc
rm -rf $OUT && mkdir -p $OUT
for f in $(find USER SYSTEM HARDWARE BALANCE -name '*.c'); do
    arm-none-eabi-gcc $CF -c "$f" -o "$OUT/$(echo $f | tr '/' '_' | sed 's/\.c$/.o/')"
done
arm-none-eabi-gcc $CF -c boot/startup_stm32f103_gcc.s -o $OUT/startup.o
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Tboot/STM32F103C8.ld \
    -Wl,--gc-sections -Wl,-Map=$OUT/firmware.map \
    --specs=nano.specs --specs=nosys.specs \
    $OUT/*.o -o $OUT/firmware.elf
arm-none-eabi-objcopy -O binary $OUT/firmware.elf $OUT/firmware.bin
arm-none-eabi-size $OUT/firmware.elf

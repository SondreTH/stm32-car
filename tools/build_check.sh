#!/bin/bash
# build_check.sh - compile the firmware from a terminal, without CubeIDE.
#
# For checking that the code builds (also with -Wall -Wextra on our car_*.c
# files). Flash from CubeIDE as usual: this does not replace its build.
# Needs arm-none-eabi-gcc and arm-none-eabi-newlib. Output goes to build/.
#
#   tools/build_check.sh
R="$(cd "$(dirname "$0")/.." && pwd)"
O="$R/build"; mkdir -p "$O"
F="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F446xx
 -std=gnu11 -O2 -ffunction-sections -fdata-sections -Wall
 -I$R/Core/Inc -I$R/Drivers/STM32F4xx_HAL_Driver/Inc -I$R/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
 -I$R/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I$R/Drivers/CMSIS/Include"
objs=""; fail=0
for f in "$R"/Core/Src/*.c "$R"/Drivers/STM32F4xx_HAL_Driver/Src/*.c "$R"/Core/Startup/*.s; do
  o="$O/$(basename "$f").o"; objs="$objs $o"
  # our files are always recompiled (headers may have changed), the HAL only when new
  if [ "$f" -nt "$o" ] || [[ "$f" == */Core/Src/* ]]; then
    W=""; [[ "$f" == */Core/Src/car_* ]] && W="-Wextra"
    arm-none-eabi-gcc $F $W -c "$f" -o "$o" || fail=1
  fi
done
[ $fail = 0 ] || { echo "BUILD FAILED"; exit 1; }
arm-none-eabi-gcc $F -T"$R/STM32F446RETX_FLASH.ld" --specs=nano.specs --specs=nosys.specs \
  -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map="$O/stm32-car.map" $objs -lm -o "$O/stm32-car.elf" 2>&1 \
  | grep -v "is not implemented and will always fail\|in function \`_"
[ -f "$O/stm32-car.elf" ] && arm-none-eabi-size "$O/stm32-car.elf"

# =============================================================================
# CH32L103K8U6 + DRV8313 FOC - GNU Make build
# Toolchain: WCH RISC-V Embedded GCC (bundled with MounRiver Studio 2)
# Run `make` from the project root.  GNU make 就在 MounRiver 里, 不用装别的东西:
#   c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/others/Build_Tools/Make/bin
# (这台机器 PATH 上的 make 恰好是 STM32CubeIDE 带的那个 —— 也能用, 但跟 STM32 毫无关系,
#  这里只是借用现成的 GNU make; 真正编代码的是下面 TOOLCHAIN_PATH 那个
#  riscv-none-embed-gcc = MounRiver 自带的 WCH RISC-V 工具链。)
# =============================================================================

TOOLCHAIN_PATH ?= c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC
CC      := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-gcc"
OBJCOPY := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-objcopy"
OBJDUMP := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-objdump"
SIZE    := "$(TOOLCHAIN_PATH)/bin/riscv-none-embed-size"

# ---- OpenOCD (WCH build, for flash & debug) ----
OPENOCD        ?= "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"
OPENOCD_SCRIPTS ?= c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin

# ---- I2CONLY=1 : 只开 UART1 + I2C1 的最小固件, 用来单独排 I2C ----
ifeq ($(I2CONLY),1)
TARGET    := L103_I2CONLY
else
TARGET    := L103_DRV8313_FOC
endif
BUILD_DIR := build

SRC_DIR := SRC
APP_DIR := User

# ---- 绝对不许开 -flto (2026-09-28 定案, 都是实测) ----
# 起因: 2026-09-28 为了省 Flash 给 CFLAGS/LDFLAGS 加了 -flto, 结果板子烧完**完全没动静**
#       (PA15 绿灯不闪, COM15 一个字节都没有)。
# 根因(本机 openocd 读芯片 + nm/map 对账): 开了 -flto 之后,
#   SRC/Startup/startup_ch32l103.S 里那批 .weak 默认中断桩(全是一个 j . 死循环)
#   抢在所有 C 里的真 ISR 之前被链上, 向量表里 TIM1_UP / USART1 / USB_LP_CAN1_RX0 /
#   HardFault 全指向那个死循环 -> UsartTx_Init() 一开 USART1 中断, CPU 就永远停在 0x67F0。
#   实测证据: 开 -flto 的镜像里 nm 查 USART1_IRQHandler / TIM1_UP_IRQHandler 全是 W 且同地址;
#             去掉 -flto 后变 T 且各自有地址, 芯片 dump 出来的向量表也对上了。
# 去掉 -flto 之后 63 K 装得下 (实测 2026-09-28, 本机 make + 修好向量表):
#       FLASH 46560 B / 63 KB = 72.17%   RAM 5452 B / 20 KB = 26.62%
#       (MRS 侧 obj/makefile 编出来是 text 46484 + data 56 = 46540 B, 基本一致)
# 所以 -flto 既不必要, 也**绝对不能再加回来**。
# (旧注释里"去掉 -flto 会 overflowed by 2572 bytes"是更早那份更大的源码的测量,
#  在当前这份瘦身过的源码上不成立; 而且那次省下来的一部分其实就是中断代码根本没链进去。)
ARCH_FLAGS := -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore
CFLAGS  := $(ARCH_FLAGS) -Os -g -std=gnu99 \
           -ffunction-sections -fdata-sections -fno-common -fsigned-char \
           -fmessage-length=0 -Wall -fmax-errors=20 -MMD -MP \
           -I$(APP_DIR) -I$(SRC_DIR)/Debug -I$(SRC_DIR)/Core -I$(SRC_DIR)/Peripheral/inc
ASFLAGS := $(ARCH_FLAGS) -g -x assembler-with-cpp -I$(SRC_DIR)/Startup

LDFLAGS := $(ARCH_FLAGS) -nostartfiles -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
           -T$(SRC_DIR)/Ld/Link.ld --specs=nano.specs -lm \
           -Wl,--print-memory-usage

# ---- I2CONLY=1 : 只编 I2C 测试固件 (不含电机/命令/CAN/USB) ----
ifeq ($(I2CONLY),1)
C_SOURCES := \
	$(APP_DIR)/main_i2conly.c \
	$(APP_DIR)/mini_fmt.c \
	$(APP_DIR)/ch32l103_it.c \
	$(APP_DIR)/system_ch32l103.c \
	$(SRC_DIR)/Debug/debug.c \
	$(SRC_DIR)/Core/core_riscv.c \
	$(wildcard $(SRC_DIR)/Peripheral/src/*.c)
else
C_SOURCES := \
	$(APP_DIR)/main.c \
	$(APP_DIR)/mini_fmt.c \
	$(APP_DIR)/drv8313.c \
	$(APP_DIR)/csense.c \
	$(APP_DIR)/foc_utils.c \
	$(APP_DIR)/foc_pid.c \
	$(APP_DIR)/foc_sensor.c \
	$(APP_DIR)/foc_motor.c \
	$(APP_DIR)/cmd.c \
	$(APP_DIR)/can_bus.c \
	$(APP_DIR)/motion_pv.c \
	$(APP_DIR)/pos_log.c \
	$(APP_DIR)/faults.c \
	$(APP_DIR)/wave.c \
	$(APP_DIR)/ch32l103_it.c \
	$(APP_DIR)/system_ch32l103.c \
	$(SRC_DIR)/Debug/debug.c \
	$(SRC_DIR)/Core/core_riscv.c \
	$(wildcard $(SRC_DIR)/Peripheral/src/*.c)
endif

ASM_SOURCES := $(SRC_DIR)/Startup/startup_ch32l103.S

OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES)) \
           $(patsubst %.S,$(BUILD_DIR)/%.o,$(ASM_SOURCES))
DEPS    := $(OBJECTS:.o=.d)

ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
BIN := $(BUILD_DIR)/$(TARGET).bin
LST := $(BUILD_DIR)/$(TARGET).lst

.PHONY: all clean size flash rebuild
all: $(HEX) $(BIN) $(LST) size

$(ELF): $(OBJECTS) $(SRC_DIR)/Ld/Link.ld | $(BUILD_DIR)
	@echo "[LD]  $@"
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(HEX): $(ELF)
	@echo "[HEX] $@"
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	@echo "[BIN] $@"
	$(OBJCOPY) -O binary $< $@

$(LST): $(ELF)
	@echo "[LST] $@"
	$(OBJDUMP) -h -S $< > $@

size: $(ELF)
	@echo "[SIZE]"
	$(SIZE) $<

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "[CC]  $<"
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "[AS]  $<"
	$(CC) $(ASFLAGS) -c -o $@ $<

$(BUILD_DIR):
	@mkdir -p $@

flash: $(ELF)
	@echo "[FLASH] $<"
	$(OPENOCD) -f debug/openocd.cfg -s "$(OPENOCD_SCRIPTS)" \
		-c "init" -c "halt" -c "program $(ELF) verify reset" -c "exit"

clean:
	@rm -rf $(BUILD_DIR)

rebuild: clean all

-include $(DEPS)

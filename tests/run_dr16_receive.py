"""Run the real DR16 receive callbacks with synthetic HAL UART/DMA input in ARM/Unicorn.

This checks startup, callbacks, rejection, rearm and recovery logic. It does not
verify the board's UART pin, receiver wiring, waveform, baud rate or interrupts.
"""
import pathlib
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build/tests_dr16_receive"
BUILD.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(ROOT / "build/test_deps"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_SP

includes = [ROOT / item for item in [
    "1Bsp", "Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
    "Middlewares/Third_Party/FreeRTOS/Source/include",
    "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
    "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
]]
flags = ["-O1", "-g", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft",
         "-ffunction-sections", "-fdata-sections", "-DUSE_HAL_DRIVER", "-DSTM32F407xx"]
for path in includes:
    flags.extend(["-I", str(path)])
dr16 = BUILD / "dr16_bsp.o"
subprocess.run(["arm-none-eabi-gcc", "-std=c11", *flags, "-c", str(ROOT / "1Bsp/dr16_bsp.c"),
                "-o", str(dr16)], check=True)
elf = BUILD / "test_dr16_receive.elf"
subprocess.run(["arm-none-eabi-g++", "-std=c++17", *flags, "-fno-exceptions", "-fno-rtti",
                "-fno-threadsafe-statics", "-fno-builtin", "-nostartfiles", "-Wl,--gc-sections",
                "-T", str(ROOT / "tests/test.ld"), str(ROOT / "tests/test_dr16_receive.cpp"),
                str(dr16), "-lm", "-lc", "-lgcc", "-o", str(elf)], check=True)
emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
emu.mem_map(0x10000, 0x400000)
with elf.open("rb") as stream:
    obj = ELFFile(stream)
    for segment in obj.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            emu.mem_write(segment["p_vaddr"], segment.data())
    symbols = {s.name: s["st_value"] for s in obj.get_section_by_name(".symtab").iter_symbols()}
    entry = obj["e_entry"]
emu.reg_write(UC_ARM_REG_SP, 0x40f000)
emu.hook_add(UC_HOOK_INTR, lambda uc, number, user: uc.emu_stop())
emu.emu_start(entry | 1, 0, timeout=30_000_000, count=100_000_000)
read = lambda name: struct.unpack("<I", emu.mem_read(symbols[name], 4))[0]
checks, failed, done = read("test_checks"), read("test_failed_line"), read("test_done")
if not done or failed:
    raise SystemExit(f"FAIL DR16 receive: line={failed}, done={done}, assertions={checks}")
print(f"PASS [DR16 receive callbacks]: {checks} assertions; synthetic HAL input; no hardware validation")

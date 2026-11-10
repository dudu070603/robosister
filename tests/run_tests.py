"""在Unicorn中执行实际ARM机器码。合成CAN输入仅为逻辑测试，不能证明实车可运行。"""
import argparse
import pathlib
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "build" / "test_deps"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC

parser = argparse.ArgumentParser(description=__doc__)
profiles = parser.add_mutually_exclusive_group()
profiles.add_argument("--bench", dest="bench", action="store_true", help="Test four-axis mode (default)")
profiles.add_argument("--vehicle", dest="bench", action="store_false", help="Test the full-vehicle profile")
parser.set_defaults(bench=True)
args = parser.parse_args()
profile = "FourAxis" if args.bench else "Vehicle"
build = ROOT / "build" / ("tests_bench" if args.bench else "tests")
build.mkdir(parents=True, exist_ok=True)
elf = build / "test_arm.elf"
cmd = ["arm-none-eabi-g++", "-std=c++17", "-O1", "-g", "-mcpu=cortex-m4", "-mthumb",
       "-mfloat-abi=soft", "-fno-exceptions", "-fno-rtti", "-fno-threadsafe-statics", "-fno-builtin",
       "-ffunction-sections", "-fdata-sections", "-nostartfiles", "-Wl,--gc-sections",
       "-T", str(ROOT / "tests" / "test.ld"), "-I", str(ROOT / "User_Driver" / "Arm"),
       str(ROOT / "tests" / "test_arm.cpp"), str(ROOT / "User_Driver" / "Arm" / "arm_control.cpp"),
       str(ROOT / "User_Driver" / "Arm" / "arm_config.cpp"),
       str(ROOT / "tests" / "test_can_tx.cpp"), str(ROOT / "Bsp" / "CAN_bsp.cpp"),
       str(ROOT / "tests" / "test_operator.cpp"), str(ROOT / "tests" / "test_four_axis.cpp"),
       str(ROOT / "tests" / "test_protect.cpp"), str(ROOT / "Task" / "task_protect.cpp"),
       str(ROOT / "tests" / "test_robot_control.cpp"), str(ROOT / "Task" / "robot_control.cpp"),
       str(ROOT / "tests" / "test_cad_mapping.cpp"),
       str(ROOT / "tests" / "test_dm_id_config.cpp"), str(ROOT / "User_Driver" / "Arm" / "dm_id_config.cpp"),
       str(ROOT / "User_Driver/Remote/dr16_protocol.c"),
       str(ROOT / "User_Driver/Remote/operator_control.cpp"),
       str(ROOT / "User_Driver/Remote/operator_config.cpp"),
       "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
       "-lm", "-lc", "-lgcc", "-o", str(elf)]
for include in ["Task", "Bsp", "Core/Inc", "User_Driver/Remote", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
                "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
                "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"]:
    cmd.extend(["-I", str(ROOT / include)])
cmd.append(f"-DARM_FOUR_AXIS_BENCH={int(args.bench)}")
subprocess.run(cmd, check=True)
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
if not done:
    raise SystemExit(
        f"FAIL: test firmware did not reach completion; checks={checks}; "
        f"arm line={failed}; CAN line={read('test_can_failed_line')}; protect line={read('test_protect_failed_line')}; bridge line={read('test_robot_failed_line')}; four-axis line={read('test_four_axis_failed_line')}; "
        f"remote line={read('test_operator_failed_line')}; CAD line={read('test_cad_failed_line')}; DM config line={read('test_dm_config_failed_line')}; "
        f"PC=0x{emu.reg_read(UC_ARM_REG_PC):08x} (100M instructions / 30s limit)"
    )
if failed:
    raise SystemExit(f"FAIL at tests/test_arm.cpp:{failed}; CAN line={read('test_can_failed_line')}; remote line={read('test_operator_failed_line')}; CAD line={read('test_cad_failed_line')}; protect line={read('test_protect_failed_line')}; bridge line={read('test_robot_failed_line')}; four-axis line={read('test_four_axis_failed_line')}; DM config line={read('test_dm_config_failed_line')} ({checks} checks executed)")
print(f"PASS [{profile}]: {checks} assertions; ARM machine code, synthetic CAN frames; no hardware validation")

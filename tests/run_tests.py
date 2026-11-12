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
profiles.add_argument("--bench", dest="bench", action="store_true", help="Test legacy four-axis compatibility")
profiles.add_argument("--vehicle", dest="bench", action="store_false", help="Test the full-vehicle profile (default)")
parser.set_defaults(bench=False)
args = parser.parse_args()
profile = "FourAxisCompatibility" if args.bench else "Vehicle"
build = ROOT / "build" / ("tests_bench" if args.bench else "tests")
build.mkdir(parents=True, exist_ok=True)
elf = build / "test_arm.elf"
# Compile the merged BSP as C, as in firmware; the C++ tests use its real decoder/getter.
dr16_object = build / "dr16_bsp.o"
c_cmd = ["arm-none-eabi-gcc", "-std=c11", "-O1", "-g", "-mcpu=cortex-m4", "-mthumb",
         "-mfloat-abi=soft", "-ffunction-sections", "-fdata-sections",
         "-DUSE_HAL_DRIVER", "-DSTM32F407xx", "-c", str(ROOT / "1Bsp/dr16_bsp.c"),
         "-o", str(dr16_object)]
cmd = ["arm-none-eabi-g++", "-std=c++17", "-O1", "-g", "-mcpu=cortex-m4", "-mthumb",
       "-mfloat-abi=soft", "-fno-exceptions", "-fno-rtti", "-fno-threadsafe-statics", "-fno-builtin",
       "-ffunction-sections", "-fdata-sections", "-nostartfiles", "-Wl,--gc-sections",
       "-T", str(ROOT / "tests" / "test.ld"),
       str(ROOT / "tests" / "test_arm.cpp"), str(ROOT / "2User_Driver" / "Arm" / "arm_control.cpp"),
       str(ROOT / "2User_Driver" / "Arm" / "arm_config.cpp"),
       str(ROOT / "tests" / "test_can_tx.cpp"), str(ROOT / "1Bsp" / "CAN_bsp.cpp"),
       str(ROOT / "tests" / "test_operator.cpp"), str(ROOT / "tests" / "test_four_axis.cpp"),
       str(ROOT / "tests" / "test_protect.cpp"), str(ROOT / "3Task" / "task_protect.cpp"),
       str(ROOT / "tests" / "test_robot_control.cpp"), str(ROOT / "3Task" / "robot_control.cpp"),
       str(ROOT / "3Task" / "yaw_service.cpp"),
       str(ROOT / "tests" / "test_cad_mapping.cpp"),
       str(ROOT / "tests" / "test_chassis.cpp"),
       str(ROOT / "tests" / "test_yaw.cpp"),
       str(ROOT / "tests" / "test_vehicle.cpp"),
       str(ROOT / "tests" / "test_robot_tuning.cpp"),
       str(ROOT / "2User_Driver/Chassis/chassis_control.cpp"),
       str(ROOT / "2User_Driver/Chassis/chassis_motor.cpp"),
       str(ROOT / "2User_Driver/Chassis/chassis_bench.cpp"),
       str(ROOT / "2User_Driver/Power/power_meter.cpp"),
       str(ROOT / "2User_Driver/Power/chassis_power.cpp"),
       str(ROOT / "2User_Driver/Vehicle/vehicle_state.cpp"),
       str(ROOT / "2User_Driver/Vehicle/robot_tuning.cpp"),
       str(ROOT / "2User_Driver/Imu/attitude_estimator.cpp"),
       str(ROOT / "tests" / "test_dm_id_config.cpp"), str(ROOT / "2User_Driver" / "Arm" / "dm_id_config.cpp"),
       str(dr16_object),
       str(ROOT / "2User_Driver/Remote/operator_control.cpp"),
       str(ROOT / "2User_Driver/Remote/operator_config.cpp"),
       "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
       "-lm", "-lc", "-lgcc", "-o", str(elf)]
for include in ["3Task", "1Bsp", "Core/Inc", "2User_Driver", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
                "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
                "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"]:
    cmd.extend(["-I", str(ROOT / include)])
    c_cmd.extend(["-I", str(ROOT / include)])
cmd.append(f"-DARM_FOUR_AXIS_BENCH={int(args.bench)}")
cmd.append(f"-DVEHICLE_SAFETY_ENABLED={int(not args.bench)}")
subprocess.run(c_cmd, check=True)
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
        f"remote line={read('test_operator_failed_line')}; CAD line={read('test_cad_failed_line')}; DM config line={read('test_dm_config_failed_line')}; chassis line={read('test_chassis_failed_line')}; yaw line={read('test_yaw_failed_line')}; vehicle line={read('test_vehicle_failed_line')}; tune line={read('test_robot_tuning_failed_line')}; "
        f"PC=0x{emu.reg_read(UC_ARM_REG_PC):08x} (100M instructions / 30s limit)"
    )
if failed:
    raise SystemExit(f"FAIL at tests/test_arm.cpp:{failed}; CAN line={read('test_can_failed_line')}; remote line={read('test_operator_failed_line')}; CAD line={read('test_cad_failed_line')}; protect line={read('test_protect_failed_line')}; bridge line={read('test_robot_failed_line')}; four-axis line={read('test_four_axis_failed_line')}; DM config line={read('test_dm_config_failed_line')}; chassis line={read('test_chassis_failed_line')}; yaw line={read('test_yaw_failed_line')}; vehicle line={read('test_vehicle_failed_line')}; tune line={read('test_robot_tuning_failed_line')} ({checks} checks executed)")
print(f"PASS [{profile}]: {checks} assertions; ARM machine code, synthetic CAN frames; no hardware validation")

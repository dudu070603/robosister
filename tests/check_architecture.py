"""Check layer boundaries and compile business modules without HAL/RTOS/task paths."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build' / 'architecture_checks'
BUILD.mkdir(parents=True, exist_ok=True)
DRIVERS = ROOT / '2User_Driver'
BSP = ROOT / '1Bsp'
TASK = ROOT / '3Task'
LEGACY = {DRIVERS / 'pid_driver.cpp', DRIVERS / 'pid_driver.h'}
LEGACY.update((DRIVERS / 'Motor_Driver').rglob('*'))
INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.MULTILINE)


def sources(directory):
    return sorted(p for p in directory.rglob('*') if p.suffix in {'.h', '.c', '.cpp'} and p not in LEGACY)


roots = [BSP, DRIVERS, TASK, ROOT / 'Core/Inc']
errors = []
for directory in (BSP, DRIVERS, TASK):
    for source in sources(directory):
        for name in INCLUDE.findall(source.read_text(encoding='utf-8-sig')):
            if '..' in Path(name).parts:
                errors.append(f'{source.relative_to(ROOT)}: relative cross-module include {name}')
            candidates = [source.parent / name] + [root / name for root in roots]
            target = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
            if target is None:
                continue  # Vendor/system includes are checked by firmware compilation.
            if directory == BSP and (target.is_relative_to(DRIVERS) or target.is_relative_to(TASK)):
                errors.append(f'{source.relative_to(ROOT)}: BSP depends on {target.relative_to(ROOT)}')
            if directory == DRIVERS and target.is_relative_to(TASK):
                errors.append(f'{source.relative_to(ROOT)}: control depends on task {target.name}')

assert not errors, '\n'.join(errors)
# No firmware source glob: new reference files must not silently enter a build.
cmake = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8-sig')
assert 'GLOB_RECURSE' not in cmake, 'Firmware sources must have explicit ownership'
for source in LEGACY:
    if source.suffix == '.cpp':
        assert source.relative_to(ROOT).as_posix() not in cmake, f'Legacy driver still built: {source}'

# The compiler cannot resolve 3Task/Core/HAL/FreeRTOS here. This checks transitive
# dependencies too, rather than just looking for a few forbidden include names.
driver_sources = [p for p in sources(DRIVERS) if p.suffix in {'.cpp', '.c'}]
headers = [p for p in sources(DRIVERS) if p.suffix == '.h']
for header in headers:
    unit = BUILD / (header.stem + '_standalone.cpp')
    unit.write_text(f'#include "{header.relative_to(DRIVERS).as_posix()}"\n', encoding='utf-8')
    driver_sources.append(unit)
subprocess.run([
    'arm-none-eabi-g++', '-std=c++17', '-mcpu=cortex-m4', '-mthumb',
    '-fsyntax-only', '-I', str(DRIVERS), '-I', str(BSP),
    *map(str, driver_sources),
], check=True)
print(f'PASS: layer boundaries; {len(headers)} standalone headers; control sources compile without task/HAL/RTOS include paths')

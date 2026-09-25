"""Standalone validation build; leaves CubeIDE-generated build files untouched."""
from pathlib import Path
import argparse
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='/tmp/dshot600_test_build')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
compiler = shutil.which('arm-none-eabi-gcc')
if compiler is None:
    matches = sorted(Path('/opt/st').glob('**/bin/arm-none-eabi-gcc'))
    if not matches:
        raise SystemExit('arm-none-eabi-gcc not found')
    compiler = str(matches[-1])
flags = ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
         '-DUSE_HAL_DRIVER', '-DSTM32H743xx', '-DUSE_PWR_LDO_SUPPLY',
         '-O1', '-g3', '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra']
for include in ['Core/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc',
                'Drivers/STM32H7xx_HAL_Driver/Inc/Legacy',
                'Drivers/CMSIS/Device/ST/STM32H7xx/Include', 'Drivers/CMSIS/Include', 'USB_DEVICE/App', 'USB_DEVICE/Target',
                'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
                'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']:
    flags += ['-I' + str(root / include)]
sources = sorted((root / 'Core/Src').glob('*.c'))
sources += sorted((root / 'Drivers/STM32H7xx_HAL_Driver/Src').glob('*.c'))
sources += sorted((root / 'Core/Startup').glob('*.s'))
sources += sorted((root / 'USB_DEVICE').rglob('*.c'))
sources += sorted((root / 'Middlewares/ST/STM32_USB_Device_Library').rglob('*.c'))
objects = []
for source in sources:
    obj = out / (source.stem + '.o')
    extra = ['-Werror'] if source.name in ('dshot_bench.c', 'usb_log.c') else []
    subprocess.run([compiler, *flags, *extra, '-c', str(source), '-o', str(obj)], check=True)
    objects.append(str(obj))
subprocess.run([compiler, *flags, *objects, '-T' + str(root / 'STM32H743XIHX_FLASH.ld'),
                '--specs=nosys.specs', '--specs=nano.specs', '-Wl,--gc-sections',
                '-Wl,-Map=' + str(out / 'dshot600_test.map'),
                '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group',
                '-o', str(out / 'dshot600_test.elf')], check=True)
subprocess.run([str(Path(compiler).with_name('arm-none-eabi-size')),
                str(out / 'dshot600_test.elf')], check=True)
print('Firmware build PASS:', out / 'dshot600_test.elf')

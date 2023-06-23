import subprocess
from pathlib import Path
import sys
from os.path import join

#try:
#    import esptool
#except:
#    sys.path.append(join(env['UPLOADER'], '..'))
#    import esptool

Import('env')

def esp32_create_combined_bin(source, target, env):
    print("Generating combined binary for serial flashing")

    cmd2 = [
        'python',
        env.subst('$PROJECT_DIR/scripts/merge_bin_esp.py'),
        '--output_name',
        env.subst('$BUILD_DIR/bsc-complete.bin'),
        #env.subst('$PROJECT_DIR/output/bsc-complete.bin'),
        '--bin_path',
        env.subst("$BUILD_DIR/bootloader.bin"),
        env.subst("$BUILD_DIR/partitions.bin"),
        env.subst('$PROJECT_DIR/doc/boot_app0.bin'),
        env.subst("$BUILD_DIR/firmware.bin"),
        '--bin_address 0x1000 0x8000 0xe000 0x10000'
    ]
    cmd_string2 = ' '.join(cmd2)
    subprocess.run(cmd_string2)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", esp32_create_combined_bin)
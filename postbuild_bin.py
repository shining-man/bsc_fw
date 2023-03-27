import shutil
import os
#import esptool
import subprocess
#from SCons.Script import ARGUMENTS
from os import path
from pathlib import Path
import os
import sys
import time
from os.path import join


try:
    import esptool
except:
    sys.path.append(join(env['UPLOADER'], '..'))
    import esptool


Import('env')

def genOutput():
    #os.system('C:/Users/tobias/.platformio/penv/Scripts/platformio.exe run --target buildfs --environment bsc')

    projectPath=env.get('PROJECT_DIR');
    dataPath = os.path.join(env.get('PROJECT_DIR'), 'output')

    if(os.path.exists(dataPath)):
        shutil.rmtree(dataPath)

    os.mkdir(dataPath)

    print('dataPath: ' + str(dataPath))
    print('projectPath: ' + str(projectPath))

    #esptool.py --chip esp32 merge_bin -o ~/output/bsc-complete.bin --flash_mode=keep --flash_size 4MB 
    # 0x1000 ~/.pio/build/bsc/bootloader.bin 
    # 0x8000 ~/.pio/build/bsc/partitions.bin 
    # 0xe000 ~/doc/boot_app0.bin 
    # 0x10000 ~/.pio/build/bsc/firmware.bin 
    # 0x2E0000 ~/.pio/build/bsc/spiffs.bin

#C:\Users\tobias\.platformio\penv\Scripts\platformio.exe run --target buildfs --environment bsc 


def esp32_create_combined_bin(source, target, env):
    print("Generating combined binary for serial flashing")
    
    flash_size = env.BoardConfig().get("upload.flash_size")
    app_offset = 0x10000

    #new_file_name = env.subst("$BUILD_DIR/${PROGNAME}-complete.bin")
    new_file_name = env.subst('$PROJECT_DIR/output/bsc-complete.bin')
    #new_file_name=new_file_name.replace("\\", "/")

    bootloader = env.subst("$BUILD_DIR/bootloader.bin")
    #bootloader=bootloader.replace("\\", "/")
    partitions = env.subst("$BUILD_DIR/partitions.bin")
    #partitions=partitions.replace("\\", "/")
    boot_app0 = env.subst('$PROJECT_DIR/doc/boot_app0.bin')
    #boot_app0=boot_app0.replace("\\", "/")
    firmware = env.subst("$BUILD_DIR/firmware.bin")
    #firmware=firmware.replace("\\", "/")
    spiffs = env.subst("$BUILD_DIR/spiffs.bin")
    #spiffs=spiffs.replace("\\", "/")



    #print(os.path.dirname(os.path.abspath(__file__)))
    print(os.path.abspath(os.getcwd()))

    cmd = [
        #'pio pkg exec --package "platformio/tool-esptoolpy" -c "',
        #"C:/Users/tobias/.platformio/penv/Scripts/python.exe",
        #env.subst("$UPLOADER"),
        #"esptool.py",
        '--chip esp32',
        'merge_bin',
        '-o',
        new_file_name,
        '--flash_size',
        flash_size,
        '0x1000',
        bootloader,
        '0x8000',
        partitions,
        '0xe000',
        boot_app0
    ]


    cmd2 = [
        'python',
        env.subst('$PROJECT_DIR/scripts/merge_bin_esp.py'),
        '--output_name',
        env.subst('$PROJECT_DIR/output/bsc-complete.bin'),
        '--bin_path',
        env.subst("$BUILD_DIR/bootloader.bin"),
        env.subst("$BUILD_DIR/partitions.bin"),
        env.subst('$PROJECT_DIR/doc/boot_app0.bin'),
        env.subst("$BUILD_DIR/firmware.bin"),
        env.subst("$PROJECT_DIR/output/spiffs.bin"),
        '--bin_address 0x1000 0x8000 0xe000 0x10000 0x2E0000'
    ]

#"0x2E0000",
        #spiffs

    cmd_string2 = ' '.join(cmd2)

#,
#        "0x10000",
#        firmware
 #,
        #"0x2E0000",
        #spiffs

    cmd_string = ' '.join(cmd)
    cmd_string=cmd_string.replace("\\", "/")

    print(cmd_string)
    #s = *cmd
    #print(s)

    #print("    Offset | File")

    #for section in sections:
    #    sect_adr, sect_file = section.split(" ", 1)
    #    print(f" -  {sect_adr} | {sect_file}")
    #    cmd += [sect_adr, sect_file]

    #cmd += [hex(app_offset), firmware_name]

    #print(f" - {hex(app_offset)} | {firmware_name}")
    #print()
    #print(f"Using esptool.py arguments: {' '.join(cmd)}")
    #print()

    #if os.environ.get("ESPHOME_USE_SUBPROCESS") is None:
    #    esptool.main(cmd)
    #else:
    
    #subprocess.run(["esptool.py", *cmd])

    #subprocess.run([env.subst("$UPLOADER"), "-h"])

    #subprocess.run(cmd_string)

    #env.Execute(*cmd)

    #os.system('pio pkg exec -p tool-esptoolpy -c esptool.py')
    #subprocess.run(cmd_string)

    #esptool.main(*cmd)

    print("finish")
    #subprocess.run(["pio pkg exec --package platformio/tool-esptoolpy -- esptool.py --help"])

    subprocess.run(cmd_string2)

    #print(env.Dump())

#genOutput()
#esp32_create_combined_bin()

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", esp32_create_combined_bin)
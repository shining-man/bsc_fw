Import("env")

def merge_firmware(source, target, env):
    env.Execute("esptool.py --chip esp32s3 merge_bin -o $BUILD_DIR/firmware_full.bin --flash_mode dio --flash_freq 80m --flash_size 4MB 0x0 $BUILD_DIR/bootloader.bin 0x8000 $BUILD_DIR/partitions.bin 0x10000 $BUILD_DIR/firmware.bin")

env.AddPostAction("buildprog", merge_firmware)
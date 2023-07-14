#!/bin/bash
set -e

#python3 /app/workspace/firmware-brickml/bootloader/rm_mcuboot_port_sign.py sign --key /app/workspace/firmware-brickml/bootloader/root-ec-p256.pem -v 1.0.0 --header-size 0x200 --align 128 --max-align 128 --slot-size 0x70000 --max-sectors 14 --overwrite-only --confirm --pad-header /app/workspace/firmware-brickml/Debug/firmware-brickml.elf /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed 

#echo srec_cat Debug/firmware-brickml.bin.signed -binary -offset 0x20000 -o firmware-brickml-signed-offset.srec
# srec_cat Debug/firmware-brickml.bin.signed -binary -offset 0x20000 -o firmware-brickml-signed-offset.srec
# echo Generated signed offset
# echo srec_cat bootloader/bootloader_brickml.srec firmware-brickml-signed-offset.srec -o firmware-brickml-w-bootloader.srec
# srec_cat bootloader/bootloader_brickml.srec firmware-brickml-signed-offset.srec -o firmware-brickml-w-bootloader.srec
# echo Generated srec
echo Done

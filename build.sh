#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "Building"

/usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-brickml || true &

pid=$! # Process Id of the previous running command
while kill -0 $pid 2>/dev/null
do
    echo "Still building..."
    sleep 2
done

wait $pid
if [ -f /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed ]; then
    echo "Building done"
    exit 0
else
    python3 /app/workspace/firmware-brickml/bootloader/rm_mcuboot_port_sign.py sign --key /app/workspace/firmware-brickml/bootloader/root-ec-p256.pem -v 1.0.0 --header-size 0x200 --align 128 --max-align 128 --slot-size 0xe0000 --max-sectors 28 --overwrite-only --confirm --pad-header /app/workspace/firmware-brickml/Debug/firmware-brickml.elf /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed 

    # echo srec_cat /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed -binary -offset 0x20000 -o firmware-brickml-signed-offset.srec
    # srec_cat /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed -binary -offset 0x20000 -o firmware-brickml-signed-offset.srec
    # echo Generated signed offset
    # echo srec_cat /app/workspace/firmware-brickml/bootloader/bootloader_brickml.srec firmware-brickml-signed-offset.srec -o firmware-brickml-w-bootloader.srec
    # srec_cat /app/workspace/firmware-brickml/bootloader/bootloader_brickml.srec firmware-brickml-signed-offset.srec -o firmware-brickml-w-bootloader.srec
    # echo Generated srec
    if [ -f /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed ]; then
        echo "Building done"
        exit 0
    else
        echo "Building failed"
        exit 1
    fi
fi

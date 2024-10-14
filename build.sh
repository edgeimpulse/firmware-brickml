#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "Building"

if [ "$1" = "--som" ];
then
    echo "Building for SoM"
    cp -f /app/workspace/firmware-brickml/hw-config/module/* /app/workspace/firmware-brickml/
else
    echo "Building for BrickML"
    cp -f /app/workspace/firmware-brickml/hw-config/brickml/* /app/workspace/firmware-brickml/
fi

rm -f /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed # remove old binary before compilation

/usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-brickml || true &

pid=$! # Process Id of the previous running command
while kill -0 $pid 2>/dev/null
do
    echo "Still building..."
    sleep 2
done

wait $pid
if [ -f /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed ]; then # sucesfully generated from build
    echo "Building done"
    echo "Image for bootloader generated"
    exit 0
elif [ -f /app/workspace/firmware-brickml/Debug/firmware-brickml.elf ]; then # not generated from build but elf has been built
    # create the .bin.signed for bootlader
    echo "Building done"
    python3 /app/workspace/firmware-brickml/bootloader/rm_mcuboot_port_sign.py sign --key /app/workspace/firmware-brickml/bootloader/root-ec-p256.pem -v 1.0.0 --header-size 0x200 --align 128 --max-align 128 --slot-size 0xe0000 --max-sectors 28 --overwrite-only --confirm --pad-header /app/workspace/firmware-brickml/Debug/firmware-brickml.elf /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed 

    if [ -f /app/workspace/firmware-brickml/Debug/firmware-brickml.bin.signed ]; then
        echo "Image for bootloader generated"
        exit 0
    else
        echo "Error in generating image for the booloader"
        exit 1
    fi
else
    echo "Build failed"
    exit 1
fi

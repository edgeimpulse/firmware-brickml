# firmware-brickml
Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded machine learning. This repository contains the Edge Impulse firmware for Renesas Cloud Kit RA6M5. This device supports all of Edge Impulse's device features, including ingestion, remote management and inferencing.

## Requirements

### Hardware

* [BrickML](https://www.edgeimpulse.com/reference-designs/brickml)

### Software
* [Renesas e2studio](https://www.renesas.com/us/en/software-tool/e-studio)
* [Renesas FSP 4.2.0](https://github.com/renesas/fsp/releases/tag/v4.2.0)
* [GNU Arm Embedded Toolchain 10.3.2021](https://developer.arm.com/downloads/-/gnu-rm)
* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation)

Project created with e2studio v22.1.0 using FSP 4.2.0

Toolchain ARM GCC 10.3.2021

## How to build

### Using e2studio

> **Note:** e2studio is available for Windows 10 & 11 and Linux Os with x86_64 architecture.

1. Install [e2studio](https://www.renesas.com/us/en/software-tool/e-studio) (during installation add option for Renesas RA support) and download [FSP 4.2.0](https://github.com/renesas/fsp/releases/tag/v3.5.0)
1. Unpack FSP 4.2.0 in the directory ofe2studio looks for. It's OS depentand, can be checked from Help->CMSIS Packs Management->Renesas RA->Show in System Explorer. Go up to \internal folder and here the zip can be unpacked.
1. Verify that the toolchain is present checking in Help->Add Renesas Toolchains. If not, download the one for your OS and add it here.
1. Clone this repo and then import in e2studio: File->Import->General->Existing Projects into Workspace and then browse for the project.
1. Generate the code for peripherls: open the configuration.xml file (double click on it) then click on "Generate Project Content".
1. To build the project: Project->Build All


### Build with docker

> **Note:** Docker build can be done with MacOs, Windows 10 & 11 and Linux with x86_64 architecture

1. Build container

    ```
    docker build -t edge-impulse-brickml .
    ```

1. Build firmware

    ```
    docker run --rm -v $PWD:/app/workspace/firmware-brickml edge-impulse-brickml
    ```


## Flashing

To update the firwmare, use the provided script ei_uploader.py
```
python3 ei_uploader.py -s <serial_port> -f <bin to update>
```

The -f parameter is optional, by default the file to be uploaded is `firmware-brickml.bin.signed`.

### Bootloader mode

To enter in bootloader mode, press the button while powering on the board.

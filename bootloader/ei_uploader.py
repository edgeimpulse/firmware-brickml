'''
ei uploader for BrickML
usage:
-s <port> serial port to connect
'''
import sys
import time
import serial
import logging
import sys
import argparse
import os
from xmodem import XMODEM
from pathlib import Path

firmware_file = "firmware-brickml.bin.signed"
AT_COMMAND_UPDATE_FIRMWARE = "AT+UPDATEFIRMWARE"

print("Welcome to ei uploader")
port_to_use = ""
flasher_app = ""

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--serial_port", help= "Serial port")
parser.add_argument("-f", "--file", help= "File to be uploaded")

args = parser.parse_args()
if (len(sys.argv) == 1):
    print("missing arguments")
    exit(1)

port_to_use = args.serial_port

if args.file is not None:
    firmware_file = args.file

if port_to_use == None:
    print("serial port not specified. Please specify it adding -s <serial_port_to_use>")
    exit(1)

try_path = Path(firmware_file)

if not try_path.is_file():
    print(firmware_file + " not found")
    exit(1)

if firmware_file.endswith(".bin.signed") == False:
    print("Format is not correct, should be .bin.signed")
    exit(1)

try:
    ser = serial.Serial(port_to_use, 115200, timeout = 0.2)
except serial.SerialException:
    print("Error: Can't open port " + port_to_use)
    exit(1)

print("Port to use: " + port_to_use)

board_found = False
retries = 0
while(board_found == False):
    print("Testing...")    
    ser.write(str.encode("test\r\n"))
    str_read = ser.readlines()    

    for stringhez in str_read:
        if (stringhez.decode().startswith("Not a valid AT command (test)")): # the answer starts with this
            print("Board found!")
            ser.readlines()
            board_found = True
            break

    retries += 1
    if retries == 20:
        print("BrickML not answering!")
        ser.close()
        exit(2)
    time.sleep(0.5) # give some time

def getc(size, timeout = 0.2):
    read_bytes_here = ser.read(size)
    print("Received: " + str(read_bytes_here))
    return read_bytes_here or None

def putc(data, timeout = 0.2):
    # print("Sending: ", data)
    return ser.write(data) or None  # note that this ignores the timeout

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG) # enable DEBUG level
logging.basicConfig(stream=sys.stdout, level=logging.INFO) # enable DEBUG level

ser.flush()
ser.close()
ser.port = port_to_use
ser.baudrate = 921600     # max tested speed
ser.open()
str_read = ser.flush()

time.sleep(0.25)
ser.write(str.encode(AT_COMMAND_UPDATE_FIRMWARE + "\r\n"))
time.sleep(0.25)

answer_found = False
retries = 0

while(answer_found == False):
    print("Testing...")    

    str_read = ser.readlines()    

    for stringhez in str_read:
        print(stringhez)
        if (stringhez.decode().startswith("Ready to update firmware")): # the answer starts with this
            print("Let's start!")
            answer_found = True
            time.sleep(0.5) # give some time
            break

    retries += 1
    if retries == 20:
        print("BrickML not answering!")
        ser.close()
        exit(2)
    time.sleep(0.5) # give some time

modem = XMODEM(getc, putc)
stream = open(firmware_file, 'rb')
modem.send(stream)

stream.close()
ser.flush()
ser.close()

print("End of uploader script")

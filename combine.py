#!/usr/bin/env python

import json
import re
import sys


def extract_firmware_version():
    with open('main/CommandHandler.cpp', 'r') as file:
        for line in file:
            if 'const char FIRMWARE_VERSION[] = ' in line:
                # The line format is `const char FIRMWARE_VERSION[] = "2.0.0-adafruit";`
                # Split by double quote and get the second element
                version = line.split('"')[1]
                return version
        raise RuntimeError("FIRMWARE_VERSION not found in CommandHandler.cpp")

def get_idf_target():
    with open("build/config.env") as file:
        config = json.load(file)
        return config["IDF_TARGET"]

bootloaderData = open("build/bootloader/bootloader.bin", "rb").read()
partitionData = open("build/partition_table/partition-table.bin", "rb").read()
#phyData = open("data/phy.bin", "rb").read()
appData = open("build/nina-fw.bin", "rb").read()

# remove everything between certificate markers to save space. There might be comments and other information.
certsData = b""
with open("certificates/data/roots.pem", "rb") as certs_file:
    in_cert = False
    for line in certs_file:
        if line.startswith(b"-----BEGIN CERTIFICATE-----"):
            in_cert = True
        if in_cert:
            certsData += line
        if line.startswith(b"-----END CERTIFICATE-----"):
            in_cert = False

# calculate the output binary size, app offset
outputSize = 0x30000 + len(appData)
if outputSize % 1024:
    outputSize += 1024 - (outputSize % 1024)

# allocate and init to 0xff
outputData = bytearray(b"\xff") * outputSize

# copy data: bootloader, partitions, app
BOOTLOADER_OFFSET = {
    "esp32" : 0x1000,
    "esp32c6" : 0x0000,
    }

try:
    target = get_idf_target()
    bootloader_offset = BOOTLOADER_OFFSET[get_idf_target()]
except KeyError:
    raise RuntimeError(f"unsupported IDF_TARGET: {target}")

for i in range(0, len(bootloaderData)):
    outputData[bootloader_offset + i] = bootloaderData[i]

for i in range(0, len(partitionData)):
    outputData[0x8000 + i] = partitionData[i]

#for i in range(0, len(phyData)):
#    outputData[0xf000 + i] = phyData[i]

for i in range(0, len(certsData)):
    outputData[0x10000 + i] = certsData[i]

# zero terminate the pem file
outputData[0x10000 + len(certsData)] = 0

for i in range(0, len(appData)):
    outputData[0x30000 + i] = appData[i]

version = extract_firmware_version()
outputFilename = f"NINA_ADAFRUIT-{version}.bin"
if len(sys.argv) > 1:
    outputFilename = sys.argv[1]

# write out
with open(outputFilename, "w+b") as f:
    f.seek(0)
    f.write(outputData)

#!/usr/bin/env python

import json
import sys
import argparse
import os

def extract_firmware_version():
    with open('main/CommandHandler.cpp', 'r') as file:
        for line in file:
            if 'const char FIRMWARE_VERSION[] = ' in line:
                # The line format is `const char FIRMWARE_VERSION[] = "2.0.0-adafruit";`
                # Split by double quote and get the second element
                version = line.split('"')[1]
                return version
        raise RuntimeError("FIRMWARE_VERSION not found in CommandHandler.cpp")

def get_idf_target(build_dir):
    with open(f"{build_dir}/config.env") as file:
        config = json.load(file)
        return config["IDF_TARGET"]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('outfile', help='output file')
    parser.add_argument('-b', '--build_dir', default='build', help='build directory')

    args = parser.parse_args()
    outfile = args.outfile
    build_dir = os.path.normpath(args.build_dir)

    bootloaderData = open(f"{build_dir}/bootloader/bootloader.bin", "rb").read()
    partitionData = open(f"{build_dir}/partition_table/partition-table.bin", "rb").read()
    #phyData = open("data/phy.bin", "rb").read()
    appData = open(f"{build_dir}/nina-fw.bin", "rb").read()

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
        idf_target = get_idf_target(build_dir)
        bootloader_offset = BOOTLOADER_OFFSET[idf_target]
    except KeyError:
        raise RuntimeError(f"unsupported IDF_TARGET: {idf_target}")

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
    outputFilename = f"{outfile}-{version}.bin"

    # write out
    with open(outputFilename, "w+b") as f:
        f.seek(0)
        f.write(outputData)


if __name__ == '__main__':
    main()

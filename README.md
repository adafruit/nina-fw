# Adafruit fork of the Arduino NINA-W102 firmware

[![Build Status](https://travis-ci.com/adafruit/nina-fw.svg?branch=master)](https://travis-ci.com/adafruit/nina-fw)

This is the Adafruit fork of the Arduino NINA-W102 firmware. The original
repository is located at https://github.com/arduino/nina-fw

This firmware uses [Espressif's IDF](https://github.com/espressif/esp-idf)

## Contributing to nina-fw

Please be aware that by contributing to this project
you are agreeing to the [Code of Conduct](https://github.com/adafruit/nina-fw/blob/master/code-of-conduct.md).
Contributors who follow the [Code of Conduct](https://github.com/adafruit/nina-fw/blob/master/code-of-conduct.md)
are welcome to submit pull requests and they will be promptly
reviewed by project admins. Please join the [Discord](https://adafru.it/discord) too.

The NINA firmware version needs to be updated in two places in this repo:
1. CommandHandler.cpp
1. CHANGELOG

## Building

The firmware shipped in Adafruit's products is compiled following these
instructions. These may differ from the instructions included in the
original Arduino firmware repository.

1. [Download the ESP32 toolchain](https://docs.espressif.com/projects/esp-idf/en/v3.3.1/get-started/index.html#setup-toolchain)
1. Extract it and add it to your `PATH`: `export PATH=$PATH:<path/to/toolchain>/bin`
1. Clone **v5.5** of the IDF: `git clone --branch v5.5 --recursive https://github.com/espressif/esp-idf.git`
1. Set the `IDF_PATH` environment variable: `export IDF_PATH=<path/to/idf>`
1. `git submodule update --init` to fetch the `certificates` submodule.
1. Run `idf.py -DBOARD=your-board build` to build the firmware (in the directory of this readme).
Where `your-board` can be found in boards/ folder, e.g. `esp32` or `fruitjam_c6`
1. You may need to set up a python3 `venv` to avoid Python library version issues.
1. You should have a file named `NINA_W102-x.x.x.bin` in the top directory
1. Use appropriate tools (`esptool.py`, appropriate pass-through firmware etc)
   to load this binary file onto your board.
    a. If you do not know how to do this, [we have an excellent guide on the Adafruit Learning System for upgrading your ESP32's firmware](https://learn.adafruit.com/upgrading-esp32-firmware)

## Packaging
The `make` command produces a bunch of binary files that must be flashed at very precise locations, making `esptool` commandline quite complicated.
Instead, once the firmware has been compiled, you can invoke `combine.py` script to produce a monolithic binary that can be flashed at 0x0.
```
make
python combine.py
```
This produces `NINA_W102.bin-{version}` file (a different name can be specified as parameter). To flash this file you can use https://learn.adafruit.com/upgrading-esp32-firmware

## Build a new certificate list (based on the Google Android root CA list)
```bash
git clone https://android.googlesource.com/platform/system/ca-certificates
cp nina-fw/tools/nina-fw-create-roots.sh ca-certificates/files
cd ca-certificates/files
./nina-fw-create-roots.sh
cp roots.pem ../../nina-fw/data/roots.pem
```

## Check certificate list against URL list
```bash
cd tools
./sslcheck.sh -c ../data/roots.pem -l url_lists/url_list_moz.com.txt -e
```

## License

Copyright (c) 2018-2019 Arduino SA. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

#!/usr/bin/env python3
"""Simple script that translates "Backtrace:" lines from the ESP output to files
and line numbers.

Run with: python3 tools/decode_backtrace.py <board>

Enter the backtrace line at the "? " prompt. CTRL-C to exit the script.
"""

import subprocess
import sys

while True:
    addresses = input("? ")
    if addresses.startswith("Backtrace:"):
        addresses = addresses[len("Backtrace:") :]
    if addresses.startswith("Stack memory:"):
        addresses = addresses[len("Stack memory:") :]
    addresses = addresses.strip().split()
    addresses = [address.split(":")[0] for address in addresses]
    for address in addresses:
        result = subprocess.run(
            ["xtensa-esp32s2-elf-addr2line", "-aipfe", "build/nina-fw.elf"]
            + [address],
            capture_output=True,
        )
        stdout = result.stdout.decode("utf-8")
        if "?? ??" not in stdout:
            print(stdout.strip())

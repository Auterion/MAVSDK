#!/usr/bin/env python3
import sys
import time

"""
Dummy cargo compartment control script
"""

def cargo_control(signal):
    if (signal == "open"):
        print("\n - Opening cargo compartment latch...\n");
        time.sleep(3)
    elif (signal == "close"):
        print("\n - Closing cargo compartment latch...\n");
        time.sleep(3)

def main():
    cargo_control(sys.argv[1])


main()

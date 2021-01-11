#!/usr/bin/env python3
import sys

"""
Dummy cargo compartment control script
"""

def cargo_control(signal):
    if (signal == "open"):
        print("\n - Opening cargo compartment latch...\n");
    elif (signal == "close"):
        print("\n - Closing cargo compartment latch...\n");

def main():
    cargo_control(sys.argv[1])


main()

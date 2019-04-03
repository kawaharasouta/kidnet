#!/usr/bin/env python

import sys 
import os
from os.path import exists, abspath, dirname, basename


def main():
    filename = "/sys/bus/pci/drivers/ixgbe/bind"
    if os.path.exists(filename):
        print("open driver bind")
        f = open(filename, "a")
    else:
        print("drivers/ixgbe/bind not found")
        sys.exit(1)

    f.write("0000:01:00.0")
    f.close()

    filename = "/sys/bus/pci/devices/0000:01:00.0/driver_override"
    if os.path.exists(filename):
        print("open device override")
        f = open(filename, "w")
    else:
        print("devices/0000:01:00.0/driver_override not found")
        sys.exit(1)

    f.write("\00")
    f.close()

if __name__ == "__main__":
    main()

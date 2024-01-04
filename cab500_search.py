

# Pseudo Code
# First try but implemented in cab_search.py

import os
import numpy as np
import can
from enum import IntEnum
import sys



# setup arguments input, range of list
start = 0x40B
end = 0x7F0
start2 = 0x221
end2 = 0x3FF
idList = np.arange(start, end, 1) # maybe try range(start, end+1, 1),
idList2 = range(start, end+1, 1)
udsClientID = 0x0000        #
udsServerID = 0x0000        #
cab500IpID = 0x0000         # Make to lists, or create class for handle it?
debugging = False           # If true, extra debug message is printed


def print_hi(name):
    print(f'Msg:, {name}')


def set_args(arguments):
    test = 0x10 + int(arguments[1])
    print_hi(hex(test))

if __name__ == "__main__":
    print_hi('Start program')
    print_hi(f'Min: {hex(min(idList))}, Max: {hex(max(idList))}')
    print_hi(f'Min: {hex(min(idList2))}, Max: {hex(max(idList2))}')
    while True:
        start_new = input("Find CAB500 sensors, define search parameters. Select start value [0x40B]: ")
        end_new = input("And end value [0x7F0]: ")
        if start_new == "":
            start = start  # do nothing
        else:
            start_new = int(start_new, 16)
            if start_new in range(start, end) or start_new in range(start2, end2):
                start = start_new
        if end_new == "":
            end = end  # do nothing
        else:
            end_new = int(end_new, 16)
            if end_new in range(start, end) or end_new in range(start2, end2):
                end = end_new

        stop = input("Run search, Yes of (N)o? [Y]")
        if stop == "N":
            print("Restart")
        else:
            print("Running search")
            break
    print("Range set to: [" + str(hex(start)) + ":" + str(hex(end)) + "]")
    print_hi("Number of lenght: " + str(len(sys.argv)))

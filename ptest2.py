#!/usr/bin/env python

import pmultiwii
import time

if __name__ == "__main__":
    timer1 = 0 
    board = pmultiwii.Multiwii("/dev/ttyAMA0")
    try:
        print "Connecting..."
        board.recieveIMU(20)
    except Exception,error:
        print "Error on Main: "+str(error)

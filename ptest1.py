#!/usr/bin/env python

import pmultiwii
import time

if __name__ == "__main__":

    board = pmultiwii.Multiwii("/dev/ttyAMA0")
    timer1 = 0
    try:
        print "Arming..."
        board.arm()
        start1 = time.time()
        while timer1 < 4:
            request = pmultiwii.Multiwii.MSP_SET_RAW_RC
            board.sendRequestMSP(board.requestMSP(request,[1500,1500,1500,1100]))
            time.sleep(0.05)
            timer1 = timer1 + (time.time() - start1)
            start1 = time.time()
        time.sleep(0.05)
        board.disarm()
        print "Disarmed!"
        time.sleep(3)
        
    except Exception,error:
        print "Error on Main: "+str(error)

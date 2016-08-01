#!/usr/bin/env python

import struct
import serial
import time
  
class Multiwii:
    """ Multiwii Serial Protocol """
    MSP_IDENT                =100
    MSP_STATUS               =101
    MSP_RAW_IMU              =102
    MSP_SERVO                =103
    MSP_MOTOR                =104
    MSP_RC                   =105
    MSP_RAW_GPS              =106
    MSP_COMP_GPS             =107
    MSP_ATTITUDE             =108
    MSP_ALTITUDE             =109
    MSP_ANALOG               =110
    MSP_RC_TUNING            =111
    MSP_PID                  =112
    MSP_BOX                  =113
    MSP_MISC                 =114
    MSP_MOTOR_PINS           =115
    MSP_BOXNAMES             =116
    MSP_PIDNAMES             =117
    MSP_SERVO_CONF           =120


    MSP_SET_RAW_RC           =200
    MSP_SET_RAW_GPS          =201
    MSP_SET_PID              =202
    MSP_SET_BOX              =203
    MSP_SET_RC_TUNING        =204
    MSP_ACC_CALIBRATION      =205
    MSP_MAG_CALIBRATION      =206
    MSP_SET_MISC             =207
    MSP_RESET_CONF           =208
    MSP_SELECT_SETTING       =210
    MSP_SET_HEAD             =211
    MSP_SET_SERVO_CONF       =212
    MSP_SET_MOTOR            =214


    MSP_BIND                 =241

    MSP_EEPROM_WRITE         =250

    MSP_DEBUGMSG             =253
    MSP_DEBUG                =254


    IDLE = 0
    HEADER_START = 1
    HEADER_M = 2
    HEADER_ARROW = 3
    HEADER_SIZE = 4
    HEADER_CMD = 5
    HEADER_ERR = 6

    PIDITEMS = 10


    def __init__(self, PortAddr):

        self.msp_ident = { 'version':None, 'multiType':None, 'multiCapability':None}
        self.msp_status = {'cycleTime':None, 'i2cError':None, 'present':None, 'mode':None}
        self.msp_raw_imu = {'size':0, 'accx':0.0, 'accy':0.0, 'accz':0.0, 'gyrx':0.0, 'gyry':0.0, 'gyrz':0.0}
        self.msp_set_rc = {'roll':0, 'pitch':0, 'yaw':0, 'throttle':0, 'aux1':0, 'aux2':0, 'aux3':0, 'aux4':0}
        self.msp_raw_gps = {'GPS_fix':0,'GPS_numSat':0,'GPS_latitude':0,'GPS_longitude':0,'GPS_altitude':0,'GPS_speed':0}
        self.msp_comp_gps = {'GPS_distanceToHome':0, 'GPS_directionToHome':0, 'GPS_update':0}
        self.msp_attitude = {'angx':0, 'angy':0, 'heading':0}
        self.msp_altitude = {'alt':0}
        self.msp_rc_tuning = {'byteRC_RATE':0, 'byteRC_EXPO':0, 'byteRollPitchRate':0, 'byteYawRate':0, 'byteDynThrPID':0, 'byteThrottle_MID':0, 'byteThrottle_EXPO':0}
        self.msp_misc = {'intPowerTrigger': 0}

        self.inBuf = []
        self.p = 0
        self.c_state = Multiwii.IDLE
        self.err_rcvd = False
        self.checksum = 0
        self.cmd = 0
        self.offset=0
        self.dataSize=0
        self.servo = []
        self.mot = []
        self.RCChan = []
        self.byteP = []
        self.byteI = []
        self.byteD = []
        self.confINF = []
        self.byteMP = []

        self.confP = []
        self.confI = []
        self.confD = []


        self.ser = serial.Serial()
        self.ser.port = PortAddr
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2

        try:
            self.ser.open()
            print "Waking up board on "+self.ser.port+"..."
            time.sleep(2)
        except Exception, error:
            print "\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n"



    def read32(self):
        value =  (self.inBuf[self.p]&0xff) + ((self.inBuf[self.p+1]&0xff)<<8) + ((self.inBuf[self.p+2]&0xff)<<16) + ((self.inBuf[self.p+3]&0xff)<<24)
        self.p = self.p + 4
        return value
    def read16(self):
        value = (self.inBuf[self.p]&0xff) + ((self.inBuf[self.p+1])<<8)
        self.p = self.p + 2
        return value
    def read8(self):
        value = (self.inBuf[self.p])&0xff
        self.p = self.p + 1
        return value


    def requestMSP (self, msp, payload = [], payloadinbytes = False):

        if msp < 0:
            return 0
        checksum = 0
        bf = ['$', 'M', '<']

        pl_size = 2 * ((len(payload)) & 0xFF)
        bf.append(pl_size)
        checksum ^= (pl_size&0xFF)

        bf.append(msp&0xFF)
        checksum ^= (msp&0xFF)
        if payload > 0:
            if (payloadinbytes == False):
                for c in struct.pack('<%dh' % ((pl_size) / 2), *payload):
                    checksum ^= (ord(c) & 0xFF)
            else:
                for c in struct.pack('<%Bh' % ((pl_size) / 2), *payload):
                    checksum ^= (ord(c) & 0xFF)
        bf = bf + payload
        bf.append(checksum)
        #print "here in requesrMSP"
        #print bf
        return bf


    def sendRequestMSP(self, msp, payloadinbytes = False):
        #print "here in sendRequestMSP"
        data = []
        for i in msp:
            data.append(i)
        #print data
        #print "Data length %d " %(len(data)-6)
        try:
            if payloadinbytes == False:
                b = None
                b = self.ser.write(struct.pack('<3c2B%dhB' % (len(data)-6), *data))
            #b = self.ser.write(struct.pack('<3c3B', *data))
            else:
                b = None
                b = self.ser.write(struct.pack('<3c2B%BhB' % (len(data) - 6), *data))
        except Exception, error:
            print "Error in sendRequestMSP"
            print "("+str(error)+")\n\n"        

    def evaluateCommand(self, cmd, dataSize):
        if cmd == Multiwii.MSP_IDENT:
            self.msp_ident['version'] = self.read8()
            self.msp_ident['multiType'] = self.read8()
            self.read8() # MSP version
            self.msp_ident['multiCapability'] = self.read32()

        elif cmd == Multiwii.MSP_STATUS:
            self.msp_status['cycleTime'] = self.read16()
            self.msp_status['i2cError'] = self.read16()
            self.msp_status['present'] = self.read16()
            self.msp_status['mode'] = self.read32()

        elif cmd == Multiwii.MSP_RAW_IMU:
            self.msp_raw_imu['accx'] = float(self.read16())
            self.msp_raw_imu['accy'] = float(self.read16())
            self.msp_raw_imu['accz'] = float(self.read16())
            self.msp_raw_imu['gyrx'] = float(self.read16())
            self.msp_raw_imu['gyry'] = float(self.read16())
            self.msp_raw_imu['gyrz'] = float(self.read16())
            self.msp_raw_imu['magx'] = float(self.read16())
            self.msp_raw_imu['magy'] = float(self.read16())
            self.msp_raw_imu['magz'] = float(self.read16())
            self.msp_raw_imu['size'] = dataSize

        elif cmd == Multiwii.MSP_SERVO:
            for i in range(0,8,1):
                self.servo.append(self.read16())

        elif cmd == Multiwii.MSP_MOTOR:
            for i in range(0, 8, 1):
                self.mot.append(self.read16())

        elif cmd == Multiwii.MSP_RC:
            for i in range(0, 8, 1):
                self.RCChan.append(self.read16())

        elif cmd == Multiwii.MSP_RAW_GPS:
            self.msp_raw_gps['GPS_fix'] = self.read8()
            self.msp_raw_gps['GPS_numSat'] = self.read8()
            self.msp_raw_gps['GPS_latitude'] = self.read32()
            self.msp_raw_gps['GPS_longitude'] = self.read32()
            self.msp_raw_gps['GPS_altitude'] = self.read16()
            self.GPS_speed = self.read16()

        elif cmd == Multiwii.MSP_COMP_GPS:
            self.msp_comp_gps['GPS_distanceToHome'] = self.read16()
            self.msp_comp_gps['GPS_directionToHome'] = self.read16()
            self.msp_comp_gps['GPS_update'] = self.read8()

        elif cmd == Multiwii.MSP_ATTITUDE:
            self.msp_attitude['angx'] = (self.read16())/10
            self.msp_attitude['angy'] = (self.read16())/10
            self.msp_attitude['head'] = self.read16()

        elif cmd == Multiwii.MSP_ALTITUDE:
            self.msp_altitude['alt'] = self.read32()

        elif cmd == Multiwii.MSP_ANALOG:
            x = None

        elif cmd == Multiwii.MSP_RC_TUNING:
            self.msp_rc_tuning['byteRC_RATE'] = self.read8()
            self.msp_rc_tuning['byteRC_EXPO'] = self.read8()
            self.msp_rc_tuning['byteRollPitchRate'] = self.read8()
            self.msp_rc_tuning['byteYawRate'] = self.read8()
            self.msp_rc_tuning['byteDynThrPID'] = self.read8()
            self.msp_rc_tuning['byteThrottle_MID'] = self.read8()
            self.msp_rc_tuning['byteThrottle_EXPO'] = self.read8()

        elif cmd == Multiwii.MSP_ACC_CALIBRATION:
            x = None

        elif cmd == Multiwii.MSP_MAG_CALIBRATION:
            x = None

        elif cmd == Multiwii.MSP_PID:
            for i in range(0, 8, 1):
                self.byteP[i] = (self.read8())
                self.byteI[i] = (self.read8())
                self.byteD[i] = (self.read8())
                if (i != 4) and (i != 5) and (i != 6):
                    self.confP[i] = (float(self.byteP[i])/10.0)
                    self.confI[i] = (float(self.byteI[i])/1000.0)
                    self.confD[i] = (float(self.byteD[i]))
            self.confP[4] = (float(self.byteP[4]) / 100.0)
            self.confI[4] = (float(self.byteI[4]) / 100.0)
            self.confD[4] = (float(self.byteD[4]) / 1000.0)
            self.confP[5] = (float(self.byteP[5]) / 10.0)
            self.confI[5] = (float(self.byteI[5]) / 100.0)
            self.confD[5] = (float(self.byteD[5]) / 1000.0)
            self.confP[6] = (float(self.byteP[6]) / 10.0)
            self.confI[6] = (float(self.byteI[6]) / 100.0)
            self.confD[6] = (float(self.byteD[6]) / 1000.0)
        elif cmd == Multiwii.MSP_BOX:
            x = None

        elif cmd == Multiwii.MSP_BOXNAMES:
            x = None

        elif cmd == Multiwii.MSP_PIDNAMES:
            x = None

        elif cmd == Multiwii.MSP_SERVO_CONF:
            x = None

        elif cmd == Multiwii.MSP_MISC:
            self.msp_misc['intPowerTrigger'] = self.read16()
            for i in range(0,4,1):
                self.MConf[i] = (self.read16())
            self.MConf[4] = (self.read32())
            self.MConf[5] = (self.read32())

        elif cmd == Multiwii.MSP_MOTOR_PINS:
            for i in range(0, 8, 1):
                self.byteMP.append(self.read16())

        elif cmd == Multiwii.MSP_DEBUGMSG:
            x = None
        elif cmd == Multiwii.MSP_DEBUG:
            x = None


    def recieveData(self, cmd):
        while self.ser.inWaiting():

            c = self.ser.read()
            #print "recieveData..."
            #print c
            if self.c_state == Multiwii.IDLE:
                if c == '$':
                    self.c_state = Multiwii.HEADER_START
                else:
                    self.c_state = Multiwii.IDLE
            elif self.c_state == Multiwii.HEADER_START:
                if c == 'M':
                    self.c_state = Multiwii.HEADER_M
                else:
                    self.c_state = Multiwii.IDLE
            elif self.c_state == Multiwii.HEADER_M:
                if c == '>':
                    self.c_state = Multiwii.HEADER_ARROW
                elif c == '!':
                    self.c_state = Multiwii.HEADER_ERR
                else:
                    self.c_state = Multiwii.IDLE
        
            elif self.c_state == Multiwii.HEADER_ARROW or self.c_state == Multiwii.HEADER_ERR:
                self.err_rcvd = (self.c_state == Multiwii.HEADER_ERR)
                #print (struct.unpack('<B',c)[0])
                dataSize = ((struct.unpack('<B',c)[0])&0xFF)
                # reset index variables
                p = 0
                offset = 0
                checksum = 0
                checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
                # the command is to follow
                self.c_state = Multiwii.HEADER_SIZE
            elif self.c_state == Multiwii.HEADER_SIZE:
                #print (struct.unpack('<B',c)[0])
                cmd = ((struct.unpack('<B',c)[0])&0xFF)
                checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
                self.c_state = Multiwii.HEADER_CMD
            elif self.c_state == Multiwii.HEADER_CMD and offset < dataSize:
                #print (struct.unpack('<B',c)[0])
                checksum ^= ((struct.unpack('<B',c)[0])&0xFF)
                self.inBuf.append(((struct.unpack('<B',c)[0]) & 0xFF))
                offset += 1
                #print "self.inBuf..."
                #print self.inBuf[offset-1]
            elif self.c_state == Multiwii.HEADER_CMD and offset >= dataSize:
                # compare calculated and transferred checksum
                #print "Final step..."
                if ((checksum&0xFF) == ((struct.unpack('<B',c)[0])&0xFF)):
                    if self.err_rcvd:
                        print "Copter didn't understand the request type"
                    else:
                        self.evaluateCommand(cmd, dataSize)
                        self.ser.flushInput()
                        self.ser.flushOutput()
                else:
                    print '"invalid checksum for command "+((int)(cmd&0xFF))+": "+(checksum&0xFF)+" expected, got "+(int)(c&0xFF))'
                    print '"<"+(cmd&0xFF)+" "+(dataSize&0xFF)+"> {");'
                    for i in range(0, len(dataSize), 1):
                        if (i != 0):
                            print ""
                        print ((self.inBuf[i] & 0xFF))
                    print "} ["+(struct.unpack('<B',c)[0])+"]"
                    print "String"

                self.c_state = Multiwii.IDLE
                #self.ser.flushOutput()
                break

    def setPID(self):
        self.sendRequestMSP(self.requestMSP(Multiwii.MSP_PID))
        self.recieveData(Multiwii.MSP_PID)
        time.sleep(0.04)
        payload = []
        for i in range(0, Multiwii.PIDITEMS, 1):
            self.byteP[i] = int((round(self.confP[i] * 10)))
            self.byteI[i] = int((round(self.confI[i] * 1000)))
            self.byteD[i] = int((round(self.confD[i])))


        # POS - 4 POSR - 5 NAVR - 6

        self.byteP[4] = int((round(self.confP[4] * 100.0)))
        self.byteI[4] = int((round(self.confI[4] * 100.0)))
        self.byteP[5] = int((round(self.confP[5] * 10.0)))
        self.byteI[5] = int((round(self.confI[5] * 100.0)))
        self.byteD[5] = int((round(self.confD[5] * 10000.0))) / 10

        self.byteP[6] = int((round(self.confP[6] * 10.0)))
        self.byteI[6] = int((round(self.confI[6] * 100.0)))
        self.byteD[6] = int((round(self.confD[6] * 10000.0))) / 10

        for i in range(0, Multiwii.PIDITEMS, 1):
            payload.append(self.byteP[i])
            payload.append(self.byteI[i])
            payload.append(self.byteD[i])
        print "Payload:..."
        print payload
        self.sendRequestMSP(self.requestMSP(Multiwii.MSP_SET_PID, payload, True), True)


    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.sendRequestMSP(self.requestMSP(Multiwii.MSP_SET_RAW_RC,data))
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.sendRequestMSP(self.requestMSP(Multiwii.MSP_SET_RAW_RC,data))
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()


    def recieveIMU(self, duration):
        timer = 0
        start = time.time()
        while timer < duration:
            self.sendRequestMSP(self.requestMSP(Multiwii.MSP_RAW_IMU))
            self.recieveData(Multiwii.MSP_RAW_IMU)
            if self.msp_raw_imu['accx'] > 32768:  # 2^15 ...to check if negative number is recieved
                self.msp_raw_imu['accx'] -= 65536 # 2^16 ...converting into 2's complement
            if self.msp_raw_imu['accy'] > 32768:
                self.msp_raw_imu['accy'] -= 65536
            if self.msp_raw_imu['accz'] > 32768:
                self.msp_raw_imu['accz'] -= 65536
            if self.msp_raw_imu['gyrx'] > 32768:
                self.msp_raw_imu['gyrx'] -= 65536
            if self.msp_raw_imu['gyry'] > 32768:
                self.msp_raw_imu['gyry'] -= 65536
            if self.msp_raw_imu['gyrz'] > 32768:
                self.msp_raw_imu['gyrz'] -= 65536
            print "size: %d, accx: %f, accy: %f, accz: %f, gyrx: %f, gyry: %f, gyrz: %f  " %(self.msp_raw_imu['size'], self.msp_raw_imu['accx'], self.msp_raw_imu['accy'], self.msp_raw_imu['accz'], self.msp_raw_imu['gyrx'], self.msp_raw_imu['gyry'], self.msp_raw_imu['gyrz'])
            time.sleep(0.04)
            timer = timer + (time.time() - start)
            start = time.time()


    def calibrateIMU(self):
        self.sendRequestMSP(self.requestMSP(Multiwii.MSP_ACC_CALIBRATION))
        time.sleep(0.01)
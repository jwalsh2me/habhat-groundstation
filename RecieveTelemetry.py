import os, sys
import os.path
import serial
import time
import datetime

def GenerateChecksum(DataToChecksum):
    payload = DataToChecksum[1:]
    checksum = 0
    for i in list(range(len(payload))):
        checksum =  checksum ^ ord(payload[i])
    return ("%02X" % checksum)


time.sleep(1)


file_time  = datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S') 
print(file_time)

ser = serial.Serial('/dev/ttyUSB0', 9600, 8, 'N', 1, timeout=20)

while True:
    print(ser.readline())
    telemetry = ser.readline()
import os, sys
import os.path
import serial
import time
import datetime

delay = 1

def GenerateChecksum(DataToChecksum):
    payload = DataToChecksum[1:]
    checksum = 0
    for i in list(range(len(payload))):
        checksum =  checksum ^ ord(payload[i])
    return ("%02X" % checksum)

def LogUplinkedCommands(data):
    with open(UplinkedCommands, 'a') as txtfile:  # Open in append mode
        txtfile.write(data + '\r\n')   
        
        
time.sleep(1)


file_time  = datetime.datetime.now().strftime('%m_%d_%Y_%H_%M_%S') 
print(file_time)

# ser = serial.Serial('/dev/ttyUSB0', 9600, 8, 'N', 1, timeout=20)
# ser = serial.Serial('/dev/ttyUSB0', 9600)
InComm = serial.Serial('/dev/ttyUSB0', 9600)

# while True:
#     print(ser.readline())
# telemetry = ser.readline()
# dline = telemetry.decode('utf-8')
# print(dline)

while True:
    
    while InComm.in_waiting > 0:  # Check for available data
        received_data = InComm.readline().decode('utf-8').rstrip('\n')  # Read and decode
        # LogUplinkedCommands(received_data)  # Log the received line
        print(received_data)

    # time.sleep(delay)
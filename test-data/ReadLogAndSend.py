import os
import os.path
import serial
import sys
import time
import platform

def GenerateChecksum(DataToChecksum):
    payload = DataToChecksum[1:]
    checksum = 0
    for i in list(range(len(payload))):
        checksum =  checksum ^ ord(payload[i])
    return ("%02X" % checksum)

# Function to Log Data to Text File (Including Line Terminators)
def LogUplinkedCommands(data):
    with open(UplinkedCommands, 'a') as txtfile:  # Open in append mode
        txtfile.write(data + '\r\n')         # Add a newline terminator and write to a file

def disk_stat(path):
    disk = os.statvfs(path)
    percent = (disk.f_bavail * 100.0) / disk.f_blocks
    return round(percent, 2)


#================================================================================
# USE
# serial port is expected as an argument. for example:
#
# Linux: python ReadLogAndSend.py /dev/ttyUSB1 picYES
# Windows: python ReadLogAndSend.py COM5 picYES
#
# "picYES"  argument is required to report image counts for images located 
# in the directory defined in "camera_dir" 
#================================================================================

#================================================================================
# CONFIG
#================================================================================

# Sets the delay between data transmitions
delay = 1

# path to the location of the GPS Log file
GPSLogFile = '/home/pi/GPSLog.csv'

# path to the location of the IMU Log file
IMULogFile = '/home/pi/PresIMULog.csv'

# path to the location of the Uplinked Command Log file
UplinkedCommands = '/home/pi/UplinkedCommands.txt'

# path to the location of the ACS Log file
ACSLogFile = '/home/pi/IncomingACSData.csv'

# path to the location of the SYS Log file
SYSLogFile = '/home/pi/SYSIncomingData.csv'

# path to the location of the camera directory. expectation is that within that directory, 
# each camera will have its own subfolder for image storage
camera_dir = "/home/pi/Pictures"

#================================================================================
# END CONFIG
#================================================================================

GPSLogSerial = 1000000
IMULogSerial = 3000000
CameraStatsLogSerial = 5000000
ACSLogSerial = 7000000
SYSLogSerial = 9000000

GPSData = "$GPS"
IMUData = "$IMU"
ACSData = "$ACS"
SYSData = "$SYS"
CameraStats = "$CAM"

#start the serial port specifed as argument
OutComm = serial.Serial(sys.argv[1], 9600)

# wait for 2 minutes before sending any data
#time.sleep(120)

while True:
    #Check if file exists, then read last line of the GPS log file
    if os.path.exists(GPSLogFile):
        with open(GPSLogFile, 'rb') as GPSFile:
            try:  # catch OS Error in case of a one line file 
                GPSFile.seek(-2, os.SEEK_END)
                while GPSFile.read(1) != b'\n':
                    GPSFile.seek(-2, os.SEEK_CUR)
            except OSError:
                GPSFile.seek(0)
            GPSLastLine = GPSFile.readline().decode()
        #Split line around commas, strip blanks
        GPSLastLineArray = [x.strip() for x in GPSLastLine.split(',')]
        #Check if GPS array is complete, meaning that it has 13 elements
        #if it does, parse out GPS Array and send it
        if len(GPSLastLineArray) == 13:
            CurrTimestampGPS = GPSLastLineArray[0]
            GPSLat = GPSLastLineArray[1]
            GPSLong = GPSLastLineArray[2]
            GPSTime = GPSLastLineArray[3]
            GPSAlt = GPSLastLineArray[4]
            GPSSpeed = GPSLastLineArray[5]
            GPSClimb = GPSLastLineArray[6]
            GPSHeading = GPSLastLineArray[7]
            #build GPS string that will be sent
            GPSData = "$GPS," + str(GPSLogSerial) + "," + str(CurrTimestampGPS) + "," + GPSLat + "," + GPSLong + "," + GPSTime + "," + GPSAlt + "," + GPSSpeed + "," + GPSClimb + "," + GPSHeading
            GPSData = GPSData + "*" + GenerateChecksum(GPSData) + "\r\n"
            #send GPS data, flush serial buffers, increment stream packet serial number
            OutComm.write(GPSData.encode())
            GPSLogSerial += 1
            time.sleep(0.5)


    #Check if file exists, then read last line of the IMU log file
    if os.path.exists(IMULogFile):
        with open(IMULogFile, 'rb') as IMUFile:
            try:  # catch OS Error in case of a one line file 
                IMUFile.seek(-2, os.SEEK_END)
                while IMUFile.read(1) != b'\n':
                    IMUFile.seek(-2, os.SEEK_CUR)
            except OSError:
                IMUFile.seek(0)
            IMULastLine = IMUFile.readline().decode()
        #Split line around commas, strip blanks
        IMULastLineArray = [x.strip() for x in IMULastLine.split(',')]
        #Check if IMU array is complete, meaning that it has 16 elements
        #if it does, parse out IMU Array and send it
        if len(IMULastLineArray) == 16:
            CurrTimestampIMU = IMULastLineArray[0]
            TempC = IMULastLineArray[1]
            pressure = IMULastLineArray[2]
            mAltitude = IMULastLineArray[3]
            imuAccelX = IMULastLineArray[4]
            imuAccelY = IMULastLineArray[5]
            imuAccelZ = IMULastLineArray[6]
            imuGyroX = IMULastLineArray[7]
            imuGyroY = IMULastLineArray[8]
            imuGyroZ = IMULastLineArray[9]
            imuMagX = IMULastLineArray[10]
            imuMagY = IMULastLineArray[11]
            imuMagZ = IMULastLineArray[12]
            imuRoll = IMULastLineArray[13]
            imuPitch = IMULastLineArray[14]
            imuYaw = IMULastLineArray[15]
            #build IMU string that will be sent
            IMUData = "$IMU," + str(IMULogSerial) + "," + CurrTimestampIMU + "," + TempC + "," + pressure + "," + mAltitude + "," + imuAccelX + "," + imuAccelY + "," + imuAccelZ + "," + imuGyroX + "," + imuGyroY + "," + imuGyroZ + "," + imuMagX + "," + imuMagY + "," + imuMagZ + "," + imuRoll + "," + imuPitch + "," + imuYaw
            IMUData = IMUData + "*" + GenerateChecksum(IMUData) + "\r\n"
            #send IMU data, flush serial buffers, increment stream packet serial number
            OutComm.write(IMUData.encode())
            time.sleep(0.5)
            IMULogSerial += 1



    #Check if file exists, then read last line of the ACS log file
    if os.path.exists(ACSLogFile):
        with open(ACSLogFile, 'rb') as ACSFile:
            try:  # catch OS Error in case of a one line file 
                ACSFile.seek(-2, os.SEEK_END)
                while ACSFile.read(1) != b'\n':
                    ACSFile.seek(-2, os.SEEK_CUR)
            except OSError:
                ACSFile.seek(0)
            ACSLastLine = ACSFile.readline().decode()
        #Split line around commas, strip blanks
        ACSLastLineArray = [x.strip() for x in ACSLastLine.split(',')]
        #Check if ACS array is complete, meaning that it has 28 elements
        #if it does, parse out ACS Array and send it
        if len(ACSLastLineArray) == 28:
            MissionTimer = ACSLastLineArray[1]
            FHFlightID = ACSLastLineArray[2]
            ClosedPosition = ACSLastLineArray[3]
            OpenPosition = ACSLastLineArray[4]
            VentingAltitudeMeters = ACSLastLineArray[5]
            FloatTimeMin = ACSLastLineArray[6]
            FloatAltitudeToleranceMeters = ACSLastLineArray[7]
            FlightLocationAltMeters = ACSLastLineArray[8]
            GPSLat = ACSLastLineArray[9]
            GPSLon = ACSLastLineArray[10]
            GPSAlt = ACSLastLineArray[11]
            GPSHeading = ACSLastLineArray[12]
            GPSSpeed = ACSLastLineArray[13]
            GPSNumberSats = ACSLastLineArray[14]
            GPSDate = ACSLastLineArray[15]
            GPSTime = ACSLastLineArray[16]
            Temp = ACSLastLineArray[17]
            Pressure = ACSLastLineArray[18]
            BetterAltitude = ACSLastLineArray[19]
            AltitudeTrend = ACSLastLineArray[20]
            ascentRate = ACSLastLineArray[21]
            ActPosition = ACSLastLineArray[22]
            ACSState = ACSLastLineArray[23]
            LastProcessedCommandReceived = ACSLastLineArray[24]
            LastProcessedCommandSerial = ACSLastLineArray[25]
            LastProcessedCommandType = ACSLastLineArray[26]
            LastProcessedCommandStatus = ACSLastLineArray[27]
            #build ACS string that will be sent
            ACSData = "$ACS," + str(ACSLogSerial) + "," + MissionTimer + "," + FHFlightID + "," + ClosedPosition + "," + OpenPosition + "," + VentingAltitudeMeters + "," + FloatTimeMin + "," + FloatAltitudeToleranceMeters + "," + FlightLocationAltMeters + "," + GPSLat + "," + GPSLon + "," + GPSAlt + "," + GPSHeading + "," + GPSSpeed + "," + GPSNumberSats + "," + GPSDate + "," + GPSTime + "," + Temp + "," + Pressure + "," + BetterAltitude + "," + AltitudeTrend + "," + ascentRate + "," + ActPosition + "," + ACSState + "," + LastProcessedCommandReceived + "," + LastProcessedCommandSerial + "," + LastProcessedCommandType + "," + LastProcessedCommandStatus
            ACSData = ACSData + "*" + GenerateChecksum(ACSData) + "\r\n"
            #send ACS data, flush serial buffers, increment stream packet serial number
            OutComm.write(ACSData.encode())
            time.sleep(0.5)
            ACSLogSerial += 1



    #Check if file exists, then read last line of the SYS log file
    if os.path.exists(SYSLogFile):
        with open(SYSLogFile, 'rb') as SYSFile:
            try:  # catch OS Error in case of a one line file 
                SYSFile.seek(-2, os.SEEK_END)
                while SYSFile.read(1) != b'\n':
                    SYSFile.seek(-2, os.SEEK_CUR)
            except OSError:
                SYSFile.seek(0)
            SYSLastLine = SYSFile.readline().decode()
        #Split line around commas, strip blanks
        SYSLastLineArray = [x.strip() for x in SYSLastLine.split(',')]
        #Check if SYS array is complete, meaning that it has 7 elements
        #if it does, parse out SYS Array and send it
        if len(SYSLastLineArray) == 7:
            LoadVoltage_V = SYSLastLineArray[0]
            Current_mA = SYSLastLineArray[1]
            BusPower_mW = SYSLastLineArray[2]
            S0Temp_C = SYSLastLineArray[3]
            S0Temp_F = SYSLastLineArray[4]
            S1Temp_C = SYSLastLineArray[5]
            S1Temp_F = SYSLastLineArray[6]
            #build SYS string that will be sent
            SYSData =  "$SYS," + str(SYSLogSerial) + "," + LoadVoltage_V + "," + Current_mA + "," + BusPower_mW + "," + S0Temp_C + "," + S0Temp_F + "," + S1Temp_C + "," + S1Temp_F
            SYSData = SYSData + "*" + GenerateChecksum(SYSData) + "\r\n"
            #send SYS data, flush serial buffers, increment stream packet serial number
            OutComm.write(SYSData.encode())
            time.sleep(0.5)
            SYSLogSerial += 1

# If comand line argument "picYES" is present, get folders and count files in folders within 
# camera_dir, store it all into CamResults, ready to send
    if sys.argv[2] == "picYES":
        CamResults = ""
        dir_count = 0
        names= os.listdir(camera_dir) # get all file and folder names in the current directory
        for found_names in names: # loop through all the files and folders
            if os.path.isdir(os.path.join(os.path.abspath(camera_dir), found_names)): # check whether the current object is a folder or not
                dir_count = dir_count + 1
                dir_file_count = len([name for name in os.listdir(camera_dir + "/" + found_names) if os.path.isfile(os.path.join(camera_dir + "/" + found_names, name))])
                if CamResults == "" : 
                    CamResults = CamResults + found_names + "," + str(dir_file_count)
                else:
                    CamResults = CamResults + "," + found_names + "," + str(dir_file_count)

        CamResults = "$CAM," + str(CameraStatsLogSerial) + "," + str(dir_count) + "," + CamResults + "," + str(disk_stat('/'))
        CamResults = CamResults + "*" + GenerateChecksum(CamResults) + "\r\n"
        #send Cam data, flush serial buffers, increment stream packet serial number
        OutComm.write(CamResults.encode())
        time.sleep(0.5)
#        OutComm.flushOutput()
        CameraStatsLogSerial += 1

    #flush serial buffers


# print strings that are being sent, used for debug
#    print(GPSData)
#    print(len(GPSLastLineArray))
#    print(IMUData)
#    print(len(IMULastLineArray))
#    print(ACSData)
#    print(len(ACSLastLineArray))
#    print(SYSData)
#    print(len(SYSLastLineArray))
#    if sys.argv[2] == "picYES":
#        print(CamResults)

    # Receive and log data (if available)
    while OutComm.in_waiting > 0:  # Check for available data
        received_data = OutComm.readline().decode('utf-8').rstrip('\n')  # Read and decode
        LogUplinkedCommands(received_data)  # Log the received line

    time.sleep(delay)
# Copyright (c) 2014, Dawn Robotics Ltd
# All rights reserved.

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.

# 3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import logging
import os.path
import re
import serial
import threading
import time
import Queue
import struct

import ino_uploader
import sensors

MESSAGE_MARKER = chr( 0xFF ) + chr( 0xFF )
COMMAND_ID_READ_DB_ENTRY = 1
COMMAND_ID_WRITE_DB_ENTRY = 2

COMMAND_ID_GET_FIRMWARE_INFO = 1
COMMAND_ID_SET_OUTPUTS = 2
COMMAND_ID_SET_PAN_SERVO_LIMITS = 3
COMMAND_ID_SET_TILT_SERVO_LIMITS = 4
COMMAND_ID_SET_SENSOR_CONFIGURATION = 5

RESPONSE_ID_FIRMWARE_INFO = 1
RESPONSE_ID_INVALID_COMMAND = 2
RESPONSE_ID_INVALID_CHECK_SUM = 3
RESPONSE_ID_BATTERY_READING = 4
RESPONSE_ID_SENSOR_READINGS = 5

NUM_ANALOG_PINS = 6

NO_ULTRASONIC_SENSOR_PRESENT = 1000  # Value returned if it looks like no ultrasonic sensor is attached
MAX_ULTRASONIC_RANGE_CM = 400

ADC_REF_VOLTAGE = 5.0
BATTERY_VOLTAGE_SCALE = 2.0   # Battery voltage is divided by 2 before it is 
                              # passed to the ADC so we must undo that

PIN_FUNC_INACTIVE = "inactive"
PIN_FUNC_DIGITAL_READ = "digital"
PIN_FUNC_ANALOG_READ = "analog"
PIN_FUNC_ULTRASONIC_READ = "ultrasonic"
ENCODER_TYPE_SINGLE_OUTPUT = "single_output"
ENCODER_TYPE_QUADRATURE = "quadrature"
                              
#---------------------------------------------------------------------------------------------------
class FirmwareInfo:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, idHigh=0x00, idLow=0x00, versionMajor=0, versionMinor=0 ):
        
        self.idHigh = idHigh
        self.idLow = idLow
        self.versionMajor = versionMajor
        self.versionMinor = versionMinor
        
    #-----------------------------------------------------------------------------------------------
    def __repr__( self ):
        
        return "{0} {1}.{2}".format( 
            hex( self.idHigh << 8 | self.idLow ).upper(), self.versionMajor, self.versionMinor )
            
    #-----------------------------------------------------------------------------------------------
    def __eq__( self, other ): 
        return self.__dict__ == other.__dict__
        
    #-----------------------------------------------------------------------------------------------
    def __ne__( self, other ): 
        return self.__dict__ != other.__dict__

#---------------------------------------------------------------------------------------------------
class SensorConfiguration:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, configD12=PIN_FUNC_ULTRASONIC_READ, 
        configD13=PIN_FUNC_INACTIVE, 
        configA0=PIN_FUNC_ANALOG_READ, configA1=PIN_FUNC_ANALOG_READ,
        configA2=PIN_FUNC_ANALOG_READ, configA3=PIN_FUNC_ANALOG_READ,
        configA4=PIN_FUNC_ANALOG_READ, configA5=PIN_FUNC_ANALOG_READ,
        leftEncoderType=ENCODER_TYPE_QUADRATURE, rightEncoderType=ENCODER_TYPE_QUADRATURE ):
            
        self.configD12 = configD12
        self.configD13 = configD13
        self.configA0 = configA0
        self.configA1 = configA1
        self.configA2 = configA2
        self.configA3 = configA3
        self.configA4 = configA4
        self.configA5 = configA5
        self.leftEncoderType = leftEncoderType
        self.rightEncoderType = rightEncoderType

    #-----------------------------------------------------------------------------------------------
    @classmethod
    def createFromDictionary( self, dictionary ):
        
        sensorConfiguration = SensorConfiguration()
        
        try:
            if type( dictionary ) == dict:
                if "configD12" in dictionary:
                    value = str( dictionary[ "configD12" ] ).lower()
                    
                    if value == PIN_FUNC_DIGITAL_READ:
                        sensorConfiguration.configD12 = PIN_FUNC_DIGITAL_READ
                        
                if "configD13" in dictionary:
                    value = str( dictionary[ "configD13" ] ).lower()
                    
                    if value == PIN_FUNC_DIGITAL_READ:
                        sensorConfiguration.configD13 = PIN_FUNC_DIGITAL_READ
                
                for i in range( 6 ):
                    
                    varName = "configA{0}".format( i )
                    
                    if varName in dictionary:
                        value = str( dictionary[ varName ] ).lower()
                        
                        if value == PIN_FUNC_DIGITAL_READ:
                            setattr( sensorConfiguration, varName, PIN_FUNC_DIGITAL_READ )
                            
                if "leftEncoderType" in dictionary:
                    value = str( dictionary[ "leftEncoderType" ] ).lower()
                    
                    if value == ENCODER_TYPE_SINGLE_OUTPUT:
                        sensorConfiguration.leftEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
                        
                if "rightEncoderType" in dictionary:
                    value = str( dictionary[ "rightEncoderType" ] ).lower()
                    
                    if value == ENCODER_TYPE_SINGLE_OUTPUT:
                        sensorConfiguration.rightEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
                        
        except Exception as e:
            logging.error( "Caught exception when parsing Mini Driver SensorConfiguration dictionary" ) 
            logging.error( str( e ) )
            
        return sensorConfiguration
        
    #-----------------------------------------------------------------------------------------------
    def setFromBytes( self, configByteA, configByteB ):
        
        configByteA = ord( configByteA )
        configByteB = ord( configByteB )
        
        # Decode configByteA
        if configByteA & (1 << 0):
            self.configD12 = PIN_FUNC_DIGITAL_READ
        else:
            self.configD12 = PIN_FUNC_ULTRASONIC_READ
            
        if configByteA & (1 << 1):
            self.configD13 = PIN_FUNC_DIGITAL_READ
        else:
            self.configD13 = PIN_FUNC_INACTIVE
            
        if configByteA & (1 << 2):
            self.configA0 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA0 = PIN_FUNC_ANALOG_READ
            
        if configByteA & (1 << 3):
            self.configA1 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA1 = PIN_FUNC_ANALOG_READ
            
        if configByteA & (1 << 4):
            self.configA2 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA2 = PIN_FUNC_ANALOG_READ
            
        if configByteA & (1 << 5):
            self.configA3 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA3 = PIN_FUNC_ANALOG_READ
            
        if configByteA & (1 << 6):
            self.configA4 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA4 = PIN_FUNC_ANALOG_READ
            
        if configByteA & (1 << 7):
            self.configA5 = PIN_FUNC_DIGITAL_READ
        else:
            self.configA5 = PIN_FUNC_ANALOG_READ
            
        # Decode configByteB
        if configByteB & (1 << 0):
            self.leftEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
        else:
            self.leftEncoderType = ENCODER_TYPE_QUADRATURE
            
        if configByteB & (1 << 1):
            self.rightEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
        else:
            self.rightEncoderType = ENCODER_TYPE_QUADRATURE
            
    #-----------------------------------------------------------------------------------------------
    def getAsBytes( self ):
        
        configByteA = 0
        
        if self.configD12 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 0)
        if self.configD13 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 1)
        if self.configA0 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 2)
        if self.configA1 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 3)
        if self.configA2 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 4)
        if self.configA3 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 5)
        if self.configA4 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 6)
        if self.configA5 == PIN_FUNC_DIGITAL_READ:
            configByteA |= (1 << 7)
        
        configByteB = 0
        if self.leftEncoderType == ENCODER_TYPE_SINGLE_OUTPUT:
            configByteB |= (1 << 0)
        if self.rightEncoderType == ENCODER_TYPE_SINGLE_OUTPUT:
            configByteB |= (1 << 1)
        
        return chr( configByteA ), chr( configByteB )
        
    #-----------------------------------------------------------------------------------------------
    def __str__( self ):
        
        return "configD12: {0}\n".format( self.configD12 ) \
            + "configD13: {0}\n".format( self.configD13 ) \
            + "configA0: {0}\n".format( self.configA0 ) \
            + "configA1: {0}\n".format( self.configA1 ) \
            + "configA2: {0}\n".format( self.configA2 ) \
            + "configA3: {0}\n".format( self.configA3 ) \
            + "configA4: {0}\n".format( self.configA4 ) \
            + "configA5: {0}\n".format( self.configA5 ) \
            + "leftEncoderType: {0}\n".format( self.leftEncoderType ) \
            + "rightEncoderType: {0}".format( self.rightEncoderType )
        
#---------------------------------------------------------------------------------------------------
def calculateCheckSum( msgBuffer ):
    
    # Use all of the data apart from the message start bytes
    checkSum = 0
    
    for msgByte in msgBuffer[ 2:-1 ]:
        checkSum += ord( msgByte )

    return ~checkSum & 0xFF

#---------------------------------------------------------------------------------------------------
class SerialReadProcess( threading.Thread ):

    DEFAULT_UPDATE_RATE_HZ = 100.0

    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPort, responseQueue, statusQueue, updateRateHz=DEFAULT_UPDATE_RATE_HZ ):
        
        threading.Thread.__init__( self )
        self.serialPort = serialPort
        self.responseQueue = responseQueue
        self.statusQueue = statusQueue
        self.serialBuffer = ""
        
        self.stopEvent = threading.Event()
        
        self.updateRateHz = updateRateHz
        if self.updateRateHz <= 0.0:
            self.updateRateHz = 1.0

    #-----------------------------------------------------------------------------------------------
    def stop( self ):
        self.stopEvent.set()

    #-----------------------------------------------------------------------------------------------
    def isStopped( self ):
        return self.stopEvent.is_set()
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.isStopped():
            
            startTime = time.time()
            
            numBytesAvailable = self.serialPort.inWaiting()
            if numBytesAvailable > 0:
                
                newBytes = self.serialPort.read( numBytesAvailable )
                self.serialBuffer += newBytes
               
                # Check to see if we've received a message
                msgStartPos = self.serialBuffer.find( MESSAGE_MARKER )
                while msgStartPos != -1:
                    
                    msgFound = False
                    
                    self.serialBuffer = self.serialBuffer[ msgStartPos: ]
                    bufferLength = len( self.serialBuffer )
                    if bufferLength > 4:
                        
                        msgLength = ord( self.serialBuffer[ 3 ] )
                        if msgLength <= bufferLength:
                            
                            # Once we've got all the bytes for a message, process them and
                            # check the rest of a buffer for another message
                            self.processMessage( self.serialBuffer[ :msgLength ] )
                            self.serialBuffer = self.serialBuffer[ msgLength: ]
                            
                            msgFound = True
                    
                    if msgFound:
                        msgStartPos = self.serialBuffer.find( MESSAGE_MARKER )
                    else:
                        msgStartPos = -1
                        
            endTime = time.time()
            sleepTime = 1.0/self.updateRateHz - (endTime - startTime)
            if sleepTime > 0.0:
                time.sleep( sleepTime )
                    
    #-----------------------------------------------------------------------------------------------
    def processMessage( self, msgBuffer ):
          
        if calculateCheckSum( msgBuffer ) != ord( msgBuffer[ -1 ] ):
            logging.warning( "Got message with invalid checksum" )
            self.responseQueue.put( "Invalid" )
            return
        
        messageId = ord( msgBuffer[ 2 ] )
        if messageId == RESPONSE_ID_FIRMWARE_INFO:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 4:
                
                logging.warning( "Got message with invalid number of bytes" )
                self.responseQueue.put( "Invalid" )
                
            else:
                
                idHigh = ord( dataBytes[ 0 ] )
                idLow = ord( dataBytes[ 1 ] )
                versionMajor = ord( dataBytes[ 2 ] )
                versionMinor = ord( dataBytes[ 3 ] )
                
                firmwareInfo = FirmwareInfo( idHigh, idLow, versionMajor, versionMinor )
 
                self.responseQueue.put( firmwareInfo )
            
        elif messageId == RESPONSE_ID_INVALID_COMMAND:
            
            logging.info( "Invalid command sent" )
            self.responseQueue.put( "Invalid" )
            
        elif messageId == RESPONSE_ID_INVALID_CHECK_SUM:
            
            logging.info( "Sent message had invalid checksum" )
            self.responseQueue.put( "Invalid Checksum" )
        
        elif messageId == RESPONSE_ID_BATTERY_READING:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 2:
                
                logging.warning( "Got message with invalid number of bytes" )
                self.responseQueue.put( "Invalid" )
                
            else:
                
                # The RESPONSE_ID_BATTERY_READING is no longer used so we just discard it.
                # The mini driver firmware should now use RESPONSE_ID_SENSOR_READINGS instead.
                pass
        
        elif messageId == RESPONSE_ID_SENSOR_READINGS:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 27:
                
                logging.warning( "Got message with invalid number of bytes" )
                self.responseQueue.put( "Invalid" )
                
            else:
                
                # Unpack the sensor message
                sensorConfiguration = SensorConfiguration()
                sensorConfiguration.setFromBytes( dataBytes[ 0 ], dataBytes[ 1 ] )
                
                batteryReading = ord( dataBytes[ 2 ] ) << 8 | ord( dataBytes[ 3 ] )
                batteryVoltage = BATTERY_VOLTAGE_SCALE * ADC_REF_VOLTAGE * float( batteryReading )/1023.0
        
                digitalReadings = ord( dataBytes[ 4 ] )
                analogReadings = [
                    ord( dataBytes[ 5 ] ) << 8 | ord( dataBytes[ 6 ] ),
                    ord( dataBytes[ 7 ] ) << 8 | ord( dataBytes[ 8 ] ),
                    ord( dataBytes[ 9 ] ) << 8 | ord( dataBytes[ 10 ] ),
                    ord( dataBytes[ 11 ] ) << 8 | ord( dataBytes[ 12 ] ),
                    ord( dataBytes[ 13 ] ) << 8 | ord( dataBytes[ 14 ] ),
                    ord( dataBytes[ 15 ] ) << 8 | ord( dataBytes[ 16 ] )
                ]
                ultrasonicReading = ord( dataBytes[ 17 ] ) << 8 | ord( dataBytes[ 18 ] )
                
                leftEncoderReading = struct.unpack( ">i", dataBytes[ 19:23 ] )[0]
                rightEncoderReading = struct.unpack( ">i", dataBytes[ 23:27 ] )[0]

                # Timestamp the sensor reading with the current time
                # TODO: This timestamp should be reasonably accurate as it is assumed that the delay introduced
                # by transmitting the sensor readings from the Mini Driver to the Pi is small. However, the
                # timestamp will be less accurate for the Ultrasonic sensor which is sampled at just 2HZ (compared
                # to 100Hz for the other sensors). At some point the mini driver firmware should be updated
                # to send the delay since the ultrasonic sensor was last read.
                sensorReadingTimestamp = time.time()
                
                self.statusQueue.put( ( "s", 
                    sensorReadingTimestamp, sensorConfiguration,
                    batteryVoltage, digitalReadings, 
                    analogReadings, ultrasonicReading,
                    leftEncoderReading, rightEncoderReading ) )
        
        else:
            
            logging.warning( "Got unrecognised response id - " + str( messageId ) )
            self.responseQueue.put( "Invalid" )

#---------------------------------------------------------------------------------------------------
class Connection():
    
    STARTUP_DELAY = 10.0     # Needed to wait for mini driver reset
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPortName, baudRate ):
        
        self.serialPort = serial.Serial( serialPortName, baudRate, timeout=0 )
        
        self.responseQueue = Queue.Queue()
        self.statusQueue = Queue.Queue()
        self.serialReadProcess = SerialReadProcess( 
            self.serialPort, self.responseQueue, self.statusQueue )
        self.serialReadProcess.start()
        
        self.sensorConfiguration = SensorConfiguration()
        self.batteryVoltageReading = sensors.SensorReading( 0.0 )
        self.digitalReadings = sensors.SensorReading( 0 )
        self.analogReadings = sensors.SensorReading( [0] * NUM_ANALOG_PINS )
        self.ultrasonicReading = sensors.SensorReading( 0 )
        self.encodersReading = sensors.SensorReading( ( 0, 0 ) )
        
        time.sleep( self.STARTUP_DELAY )
        
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.close()
        
    #-----------------------------------------------------------------------------------------------
    def close( self ):
        
        if self.serialReadProcess != None:
            self.serialReadProcess.stop()
            self.serialReadProcess.join()
            self.serialReadProcess = None
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        while not self.statusQueue.empty():
            statusData = self.statusQueue.get_nowait()
            
            if statusData[ 0 ] == "s":
                
                sensorReadingTimestamp = statusData[ 1 ]
                self.sensorConfiguration = statusData[ 2 ]
                self.batteryVoltageReading = sensors.SensorReading( statusData[ 3 ], sensorReadingTimestamp )
                self.digitalReadings = sensors.SensorReading( statusData[ 4 ], sensorReadingTimestamp )
                self.analogReadings = sensors.SensorReading( statusData[ 5 ], sensorReadingTimestamp )
                self.ultrasonicReading = sensors.SensorReading( statusData[ 6 ], sensorReadingTimestamp )
                self.encodersReading = sensors.SensorReading( ( statusData[ 7 ], statusData[ 8 ] ), sensorReadingTimestamp )
    
    #-----------------------------------------------------------------------------------------------
    def sendMessageAskingForFirmwareInfo( self ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_GET_FIRMWARE_INFO ) + chr( 5 ) + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )
        
        self.serialPort.write( msgBuffer )
    
    #-----------------------------------------------------------------------------------------------
    def getFirmwareInfo( self ):
        
        # Clear out the response queue
        while not self.responseQueue.empty():
            responseData = self.responseQueue.get_nowait()
        
        self.sendMessageAskingForFirmwareInfo()
        
        # Wait for a response
        firmwareInfo = FirmwareInfo()
        startTime = time.time()
        readFirmwareInfo = False
        while not readFirmwareInfo and time.time() - startTime < 5.0:
            
            while not self.responseQueue.empty():
                response = self.responseQueue.get_nowait()
                
                if isinstance( response, FirmwareInfo ):
                    firmwareInfo = response
                elif response == "Invalid Checksum":
                    
                    # Request failed resend
                    self.sendMessageAskingForFirmwareInfo()
                
            self.update()

        return firmwareInfo
    
    #-----------------------------------------------------------------------------------------------
    def setSensorConfiguration( self, sensorConfiguration ):
        
        sensorConfigByteA, sensorConfigByteB = sensorConfiguration.getAsBytes()
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_SENSOR_CONFIGURATION) \
            + chr( 7 ) \
            + sensorConfigByteA + sensorConfigByteB \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )

        self.serialPort.write( msgBuffer )
    
    #-----------------------------------------------------------------------------------------------
    def getSensorConfiguration( self ):
        
        return self.sensorConfiguration
    
    #-----------------------------------------------------------------------------------------------
    def getBatteryVoltageReading( self ):
        
        return self.batteryVoltageReading
        
    #-----------------------------------------------------------------------------------------------
    def getDigitalReadings( self ):
        
        return self.digitalReadings
        
    #-----------------------------------------------------------------------------------------------
    def getAnalogReadings( self ):
        
        return self.analogReadings
        
    #-----------------------------------------------------------------------------------------------
    def getUltrasonicReading( self ):
        
        return self.ultrasonicReading
        
    #-----------------------------------------------------------------------------------------------
    def getEncodersReading( self ):
        
        return self.encodersReading
    
    #-----------------------------------------------------------------------------------------------
    def setOutputs( self, leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_OUTPUTS ) \
            + chr( 9 ) \
            + chr( leftMotorSpeed ) + chr( rightMotorSpeed ) \
            + chr( panAngle ) + chr( tiltAngle ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )
        
        self.serialPort.write( msgBuffer )
        
    #-----------------------------------------------------------------------------------------------
    def setPanServoLimits( self, servoMin, servoMax ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_PAN_SERVO_LIMITS) \
            + chr( 9 ) \
            + chr( (servoMin >> 8) & 0xFF ) + chr( servoMin & 0xFF ) \
            + chr( (servoMax >> 8) & 0xFF ) + chr( servoMax & 0xFF ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )

        self.serialPort.write( msgBuffer )
        
    #-----------------------------------------------------------------------------------------------
    def setTiltServoLimits( self, servoMin, servoMax ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_TILT_SERVO_LIMITS) \
            + chr( 9 ) \
            + chr( (servoMin >> 8) & 0xFF ) + chr( servoMin & 0xFF ) \
            + chr( (servoMax >> 8) & 0xFF ) + chr( servoMax & 0xFF ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )

        self.serialPort.write( msgBuffer )
        
#---------------------------------------------------------------------------------------------------
class PresetMotorSpeeds:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, batteryVoltage, maxAbsMotorSpeed, maxAbsTurnSpeed ):
        
        self.batteryVoltage = batteryVoltage
        self.maxAbsMotorSpeed = maxAbsMotorSpeed
        self.maxAbsTurnSpeed = maxAbsTurnSpeed
        
#---------------------------------------------------------------------------------------------------
class MiniDriver():
    
    SERIAL_PORT_NAME = "/dev/ttyUSB0"
    BAUD_RATE = 57600
    FIRMWARE_MAIN_FILENAME = "mini_driver_firmware/mini_driver_firmware.ino"
    BOARD_MODEL = "atmega8"
    
    EXPECTED_FIRMWARE_INFO = FirmwareInfo( 0xAC, 0xED, 0, 1 )
    
    MAX_ABS_MOTOR_SPEED = 100
    MIN_PULSE_WIDTH = 200
    MAX_PULSE_WIDTH = 2800
    
    PRESET_MOTOR_SPEEDS = [     # Presets should be ordered by voltage from low to high
        PresetMotorSpeeds( 5.5, 80.0, 50.0 ),
        PresetMotorSpeeds( 7.5, 60.0, 30.0 )
    ]
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.scriptPath = os.path.dirname( __file__ )
        if self.scriptPath == "":
            self.scriptPath = "./"
        self.connection = None
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def connect( self, uploadIfInitialConnectionFails=True ):
        
        """Establishes a connection with the Mini Driver and confirms that it contains
           the correct version of the firmware. If not then the routine builds the
           firmware using Ino and uploads it to the Mini Driver"""
        
        self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
        firmwareInfo = self.connection.getFirmwareInfo()
        logging.info( "Read " + str( firmwareInfo ) )
        logging.info( "Expected " + str( self.__getExpectedFirmwareInfo() ) )
        
        if firmwareInfo != self.__getExpectedFirmwareInfo():
            
            self.connection.close()
            self.connection = None
            
            if uploadIfInitialConnectionFails:
            
                logging.info( "Unable to connect to correct firmware, uploading..." )
                uploadResult = ino_uploader.upload( self.__getFirmwareDir(), 
                    serialPortName=self.SERIAL_PORT_NAME, boardModel=self.BOARD_MODEL )
                
                if uploadResult == True:
                    
                    self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
                    firmwareInfo = self.connection.getFirmwareInfo()
                    logging.info( "Read " + str( firmwareInfo ) )
                    
                    if firmwareInfo != self.__getExpectedFirmwareInfo():
                
                        self.connection.close()
                        self.connection = None
            
        return self.isConnected()
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        if self.isConnected():
            self.connection.close()
            self.connection = None
    
    #-----------------------------------------------------------------------------------------------
    def isConnected( self ):
        
        return self.connection != None

    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if self.connection != None:
            self.connection.update()
        
    #-----------------------------------------------------------------------------------------------
    def getFirmwareInfo( self ):
        
        result = FirmwareInfo()
        
        if self.connection != None:
            result = self.connection.getFirmwareInfo()
    
        return result
    
    #-----------------------------------------------------------------------------------------------
    def setSensorConfiguration( self, sensorConfiguration ):
        
        if self.connection != None:
            self.connection.setSensorConfiguration( sensorConfiguration )
    
    #-----------------------------------------------------------------------------------------------
    def getSensorConfiguration( self ):
        
        """:return: The current SensorConfiguration of the Mini Driver 
           :rtype: SensorConfiguration"""
        
        result = SensorConfiguration()
        
        if self.connection != None:
            result = self.connection.getSensorConfiguration()
    
        return result
    
    #-----------------------------------------------------------------------------------------------
    def getBatteryVoltageReading( self ):
        
        """:return: A :py:class:`SensorReading` containing the most recent battery voltage read 
                    from the Mini Driver as a float
           :rtype: :py:class:`SensorReading`"""
        
        result = sensors.SensorReading( 0.0 )
        
        if self.connection != None:
            result = self.connection.getBatteryVoltageReading()
    
        return result
        
    #-----------------------------------------------------------------------------------------------
    def getDigitalReadings( self ):
        
        """:return: A :py:class:`SensorReading` containing the most recent set of digital readings 
                    read from the Mini Driver. The digital readings are returned as a byte with the 
                    bits corresponding to the digital readings from the following pins
                    
                    pin     A5 | A4 | A3 | A2 | A1 | A0 | D13 | D12
                    bit      7                                    0
                    
           :rtype: :py:class:`SensorReading`"""
        
        result = sensors.SensorReading( 0 )
        
        if self.connection != None:
            result = self.connection.getDigitalReadings()
    
        return result
        
    #-----------------------------------------------------------------------------------------------
    def getAnalogReadings( self ):
        
        """:return: A :py:class:`SensorReading` containing the most recent set of analog readings 
                    read from the Mini Driver. There are 6 analog pins that can be read on the Mini 
                    Driver and so the :py:class:`SensorReading` contains a list of 6 floats.
           :rtype: :py:class:`SensorReading`"""
        
        result = sensors.SensorReading( [0] * NUM_ANALOG_PINS )
        
        if self.connection != None:
            result = self.connection.getAnalogReadings()
    
        return result
        
    #-----------------------------------------------------------------------------------------------
    def getUltrasonicReading( self ):
        
        """:return: A :py:class:`SensorReading` containing the most recent ultrasonic distance 
                    reading from the Mini Driver. The distance is in centimetres and the maximum
                    range is MAX_ULTRASONIC_RANGE_CM. If it looks as if no ultrasonic sensor is 
                    attached then NO_ULTRASONIC_SENSOR_PRESENT will be set as the distance.
                    
           :rtype: :py:class:`SensorReading`"""
        
        result = sensors.SensorReading( 0 )
        
        if self.connection != None:
            result = self.connection.getUltrasonicReading()
    
        return result
        
    #-----------------------------------------------------------------------------------------------
    def getEncodersReading( self ):
        
        """:return: A :py:class:`SensorReading` containing the most recent reading from the 
                    Mini Driver encoders.
                    
           :rtype: :py:class:`SensorReading`"""
        
        result = sensors.SensorReading( (0, 0) )
        
        if self.connection != None:
            result = self.connection.getEncodersReading()
    
        return result
    
    #-----------------------------------------------------------------------------------------------
    def getPresetMotorSpeeds( self ):
        
        batteryVoltage = self.getBatteryVoltageReading().data
        maxAbsMotorSpeed = 0.0
        maxAbsTurnSpeed = 0.0
        
        for preset in self.PRESET_MOTOR_SPEEDS:
            
            maxAbsMotorSpeed = preset.maxAbsMotorSpeed
            maxAbsTurnSpeed = preset.maxAbsTurnSpeed
            
            if batteryVoltage <= preset.batteryVoltage:
                break
                
        return maxAbsMotorSpeed, maxAbsTurnSpeed
    
    #-----------------------------------------------------------------------------------------------
    def setOutputs( self, leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle ):
        
        SIGNED_TO_UNSIGNED_OFFSET = 128
        
        leftMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( int( leftMotorSpeed ), self.MAX_ABS_MOTOR_SPEED ) )
        leftMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        rightMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( int( rightMotorSpeed ), self.MAX_ABS_MOTOR_SPEED ) )
        rightMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        
        panAngle = max( 0, min( int( panAngle ), 180 ) )
        tiltAngle = max( 0, min( int( tiltAngle ), 180 ) )
        
        if self.connection != None:
            self.connection.setOutputs( leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle )
            
    #-----------------------------------------------------------------------------------------------
    def setPanServoLimits( self, servoMin, servoMax ):
     
        servoMin = max( self.MIN_PULSE_WIDTH, min( int( servoMin ), self.MAX_PULSE_WIDTH ) )
        servoMax = max( self.MIN_PULSE_WIDTH, min( int( servoMax ), self.MAX_PULSE_WIDTH ) )
        
        if self.connection != None:
            self.connection.setPanServoLimits( servoMin, servoMax )
            
    #-----------------------------------------------------------------------------------------------
    def setTiltServoLimits( self, servoMin, servoMax ):
     
        servoMin = max( self.MIN_PULSE_WIDTH, min( int( servoMin ), self.MAX_PULSE_WIDTH ) )
        servoMax = max( self.MIN_PULSE_WIDTH, min( int( servoMax ), self.MAX_PULSE_WIDTH ) )
        
        if self.connection != None:
            self.connection.setTiltServoLimits( servoMin, servoMax )
    
    #-----------------------------------------------------------------------------------------------
    def __getFirmwareDir( self ):
        
        firmwarePath = self.scriptPath + "/" + self.FIRMWARE_MAIN_FILENAME
        return os.path.dirname( firmwarePath )
    
    #-----------------------------------------------------------------------------------------------
    def __getExpectedFirmwareInfo( self ):
 
        idLow = None
        idHigh = None
        versionMajor = None
        versionMinor = None
        
        # Open up the firmware source file and search for the firmware Id
        firmwarePath = self.scriptPath + "/" + self.FIRMWARE_MAIN_FILENAME
        firmwareFile = open( firmwarePath )
        
        idRegEx = re.compile( "const[ ]*uint16_t[ ]*FIRMWARE_ID[ ]*=[ ]*(?P<firmwareId>\w*?)[ ]*;" )
        versionMajorRegEx = re.compile( "const[ ]*uint8_t[ ]*VERSION_MAJOR[ ]*=[ ]*(?P<versionMajor>\w*?)[ ]*;" )
        versionMinorRegEx = re.compile( "const[ ]*uint8_t[ ]*VERSION_MINOR[ ]*=[ ]*(?P<versionMinor>\w*?)[ ]*;" )
        
        for line in firmwareFile:
            
            match = idRegEx.search( line )
            if match != None:
                
                try:
                    firmwareId = int( match.groupdict()[ "firmwareId" ], 16 )
                    idHigh = (firmwareId >> 8) & 0xFF
                    idLow = firmwareId & 0xFF
                    
                except ValueError:
                    pass
                
            match = versionMajorRegEx.search( line )
            if match != None:
                
                try:
                    versionMajor = int( match.groupdict()[ "versionMajor" ] )
                    
                except ValueError:
                    pass
                
            match = versionMinorRegEx.search( line )
            if match != None:
                
                try:
                    versionMinor = int( match.groupdict()[ "versionMinor" ] )
                    
                except ValueError:
                    pass
        
        firmwareFile.close()
           
        if idHigh == None or idLow == None:
            
            raise Exception( "Unable to find expected firmware Id" )
        
        if versionMajor == None or versionMinor == None:
            
            raise Exception( "Unable to find expected firmware version" )
        
        return FirmwareInfo( idHigh, idLow, versionMajor, versionMinor )

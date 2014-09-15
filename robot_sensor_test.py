#! /usr/bin/env python

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

# To run this demo you first need to stop the robot web server (which will already
# be talking to the Mini Driver). To do this run
#
# sudo service robot_web_server stop
#

import time
import mini_driver
import logging

logging.basicConfig( level=logging.DEBUG )

#---------------------------------------------------------------------------------------------------
def byteToString( dataByte ):
    
    return "{7} {6} {5} {4} {3} {2} {1} {0}".format(
        (dataByte >> 0) & 1, (dataByte >> 1) & 1,
        (dataByte >> 2) & 1, (dataByte >> 3) & 1,
        (dataByte >> 4) & 1, (dataByte >> 5) & 1,
        (dataByte >> 6) & 1, (dataByte >> 7) & 1 )

#---------------------------------------------------------------------------------------------------
miniDriver = mini_driver.MiniDriver()
connected = miniDriver.connect()
print "connected =", connected

if connected:
    
    sensorConfiguration = mini_driver.SensorConfiguration(
        configD12=mini_driver.PIN_FUNC_ULTRASONIC_READ, 
        configD13=mini_driver.PIN_FUNC_DIGITAL_READ, 
        configA0=mini_driver.PIN_FUNC_ANALOG_READ, 
        configA1=mini_driver.PIN_FUNC_ANALOG_READ,
        configA2=mini_driver.PIN_FUNC_ANALOG_READ, 
        configA3=mini_driver.PIN_FUNC_DIGITAL_READ,
        configA4=mini_driver.PIN_FUNC_ANALOG_READ, 
        configA5=mini_driver.PIN_FUNC_ANALOG_READ,
        leftEncoderType=mini_driver.ENCODER_TYPE_QUADRATURE, 
        rightEncoderType=mini_driver.ENCODER_TYPE_QUADRATURE )
    
    try:
        while True:
            
            miniDriver.setSensorConfiguration( sensorConfiguration )
            
            miniDriver.update()
            
            # Print out the sensor readings
            print "Sensor timestamp:", miniDriver.getDigitalReadings().timestamp
            
            receivedSensorConfiguration = miniDriver.getSensorConfiguration()
            print "Sensor configuration ="
            print receivedSensorConfiguration
            
            print "Digital Readings:", byteToString( miniDriver.getDigitalReadings().data )
            print "Analog Readings:"
            print miniDriver.getAnalogReadings().data
            print "Ultrasonic:", miniDriver.getUltrasonicReading().data
            print "Encoders:", miniDriver.getEncodersReading().data
            print ""
            
            time.sleep( 0.1 )
    
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
    except Exception as e:
        
        print "Unhandled exception..."
        print e

miniDriver.disconnect()
del miniDriver
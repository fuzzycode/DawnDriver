
# This is the default Pi sensor reader used by the Pi camera robot web server. To implement your
# own sensor reader you should take a copy of this class and implement each of the routines. Then
# put the name of your module into the config file.

import time
import sensor_reading

#---------------------------------------------------------------------------------------------------
class PiSensorReader:

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):

        # TODO: Write any setup code, and initialise any state variables here
        pass

    #-----------------------------------------------------------------------------------------------
    def shutdown( self ):

        # TODO: Write any shutdown code here. i.e. if you created any threads or processes to
        # read sensor data asynchronously, then shut them down and clean up here.
        pass

    #-----------------------------------------------------------------------------------------------
    def readSensors( self ):

        sensorDict = {}
    
        # TODO: Write code to read from your sensors here. Put the readings into a SensorReading
        # instance so that the readings are timestamped and return all readings in a dictionary.
        
        # NOTE: This code should execute as quickly as possible because it will be called at a rate
        # of at least 100Hz. If reading the sensors attached to the Pi will take longer than this
        # then sensor reading should be done in a separate thread and this routine should be used
        # to just return the latest set of sensor readings.

        # Code for reading from the sensors could look like this
        # sensorValue = readFromSensor
        # sensorReading = sensor_reading.SensorReading( sensorValue, timestamp=time.time() )
        # sensorDict[ "myReading" ] = sensorReading

        return sensorDict
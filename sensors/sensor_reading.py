

#---------------------------------------------------------------------------------------------------
class SensorReading:

    """A very general purpose sensor reading class consisting of a timestamp and some data.
       Timestamps are currently just the system clock time at which a reading was taken"""

    #-----------------------------------------------------------------------------------------------
    def __init__( self, data, timestamp=0.0 ):

        """Constructor which can be provided with data read by the sensor, and optionally a timestamp

            :param var data: Data from the sensor reading, can be of arbitrary type
            :param float timestamp: The the system clock time at which the reading was taken"""

        self.data = data
        self.timestamp = timestamp
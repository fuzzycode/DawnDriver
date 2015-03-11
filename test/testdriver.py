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

import unittest
from unittest import TestCase
import time

import driver


class TestDriver(TestCase):
    config = driver.SensorConfiguration(
        configD12=driver.PIN_FUNC_ULTRASONIC_READ,
        configD13=driver.PIN_FUNC_DIGITAL_READ,
        configA0=driver.PIN_FUNC_ANALOG_READ,
        configA1=driver.PIN_FUNC_ANALOG_READ,
        configA2=driver.PIN_FUNC_ANALOG_READ,
        configA3=driver.PIN_FUNC_DIGITAL_READ,
        configA4=driver.PIN_FUNC_ANALOG_READ,
        configA5=driver.PIN_FUNC_ANALOG_READ,
        leftEncoderType=driver.ENCODER_TYPE_QUADRATURE,
        rightEncoderType=driver.ENCODER_TYPE_QUADRATURE )

    def setUp(self):
        self.driver = driver.MiniDriver()

    def tearDown(self):
        pass

    def test_sensors(self):
        connected = self.driver.connect()
        self.assertTrue(connected, "Connection should be successful")

        self.driver.setSensorConfiguration(self.config)

        for i in range(0, 10):
            self.driver.update()

            time.sleep(0.1)

if __name__ == "__main__":
    sys.exit(unittest.main())
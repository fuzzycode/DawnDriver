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

__description__ =  """
A stripped down version of the original Dagu Mini Driver from DawnRobotics. Only contains the actual driver
code to communicate with the Dagu Mini Driver board.

Added a setup.py file for easy installation and inclusion in other projects based on the driver.
"""

from setuptools import setup

__version__=(1,0,0)


if __name__ == "__main__":

    setup(
        name="DawnDriver",
        version='.'.join([str(d) for d in __version__]),
        author="Alan Broun(Dawn Robotics)",
        author_email='abroun@alanbroun.net',
        maintainer="Bjoern Larsson",
        maintainer_email="develop@bjornlarsson.net",
        description='A Dagu Mini Driver',
        long_description=__description__,
        url="",
        keywords="robotics firmware",
        install_requires=['ino'],
        test_suite='test',
        classifiers=[f.strip() for f in """
        Development Status :: 4 - Beta
        Intended Audience :: Developers
        Operating System :: POSIX :: Linux
        Programming Language :: Python
        Topic :: Software Development :: Libraries :: Python Modules
        Topic :: Utilities""".splitlines() if f.strip()],

        packages=['firmware'],
        package_dir={'firmware': 'firmware/'},
        package_data={'firmware': ['firmware/*']}
    )
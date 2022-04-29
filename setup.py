#!/usr/bin/python3

'''
This file is part of the SCUTTLEPy library (https://github.com/ansarid/scuttlepy).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import setuptools
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

install_requires = ['numpy',
                    'smbus2',
                    'pyyaml',
                    ]

setuptools.setup(
    name='scuttlepy',
    version='0.0.1',
    description='SCUTTLE Python Library',
    packages=setuptools.find_packages(),
    install_requires=install_requires,
    keywords=['scuttle', 'robot', 'python'],
    url='https://github.com/ansarid/scuttlepy',
    long_description=long_description,
    long_description_content_type='text/markdown'
)

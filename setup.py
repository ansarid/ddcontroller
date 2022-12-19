#!/usr/bin/env python3

'''
This file is part of the DDController library (https://github.com/ansarid/ddcontroller).
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

import os
import shutil
import filecmp
import setuptools
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

install_requires = [
                    'numpy',
                    'smbus2',
                    'ruamel.yaml',
                    'simple-pid',
                    'adafruit-circuitpython-ina219',
                    # 'as5048b'
                   ]

setuptools.setup(
    name='ddcontroller',
    version='0.0.1',
    description='Python controller for differential drive robots.',
    packages=setuptools.find_packages(),
    install_requires=install_requires,
    keywords=['python', 'robot', 'controller', 'differential drive', 'differential drive controller'],
    url='https://github.com/ansarid/ddcontroller',
    long_description=long_description,
    long_description_content_type='text/markdown'
)

if os.path.exists('/opt/ddcontroller'):
    pass
else:
    os.mkdir('/opt/ddcontroller')
    os.mkdir('/opt/ddcontroller/config')

if os.path.exists('/opt/ddcontroller/config/default.yaml') and filecmp.cmp(this_directory+'/config/default.yaml', '/opt/ddcontroller/config/default.yaml'):
    print('Config already exists and is up to date.')
else:
    shutil.copyfile(this_directory+'/config/default.yaml', '/opt/ddcontroller/config/default.yaml')
    print('Updated to latest config.')

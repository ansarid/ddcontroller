import pip
import importlib
import setuptools
from os import path
# read the contents of your README file
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

install_requires=['smbus2',
                  'fastlogging',
                 ]

setuptools.setup(
    name='scuttlepy',
    version='0.0.1',
    description='SCUTTLE Python Library',
    packages=setuptools.find_packages(),
    install_requires=install_requires,
    keywords=['scuttle','robot','python'],
    url='https://github.com/ansarid/scuttlepy',
    long_description=long_description,
    long_description_content_type='text/markdown'
)

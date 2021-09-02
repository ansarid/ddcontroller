import setuptools

# read the contents of your README file
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

install_requires=['smbus2',
                  'fastlogging',
                 ]

if detector.board.BEAGLEBONE_BLUE:
    install_requires.append('rcpy')
elif detector.board.any_raspberry_pi_40_pin:
    install_requires.append('RPi.GPIO')
elif detector.board.JETSON_NANO:
    install_requires.append('Jetson.GPIO')
    
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

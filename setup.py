import setuptools

# read the contents of your README file
from os import path
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

setuptools.setup(
    name='scuttlepy',
    version='0.0.1',
    description='SCUTTLE Python Library',
    packages=setuptools.find_packages(),
    install_requires=['spidev'],
    keywords=['scuttle','robot','python'],
    url='https://github.com/ansarid/scuttlepy',
    long_description=long_description,
    long_description_content_type='text/markdown'
)


import os
from glob import glob
from setuptools import setup

package_name = 'seahawk_rov'

setup(
    name=package_name,
    version='0.0.2',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # The `seahawk_rov` package doesn't actually contain any launch files. They're up a level in the main workspace.
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=[
        'setuptools',
        'RPi.GPIO',
        'Adafruit-Blinka',
        'adafruit-python-shell',
        'adafruit-circuitpython-bme280',
        'adafruit-circuitpython-bno08x',
        'adafruit-circuitpython-motorkit',
        'adafruit-circuitpython-servokit',
        'adafruit-circuitpython-typing',
        ],
    zip_safe=True,
    author='Ciaran Farley',
    author_email='ciaran@cturtle98.com',
    maintainer='Cabrillo Robotics Club',
    maintainer_email='cabrillorobotics@gmail.com',
    keywords=['MATE', 'ROV', '2023', 'Underwater', 'Robotics', 'Competition'],
    classifiers=[
        'Intended Audience :: Education',
        'License :: AGPLv3',
        'Programming Language :: Python',
        'Topic :: Education',
    ],
    description='package for deployment of the SeaHawk-ROV by Cabrillo College Robotics Club',
    license='AGPLv3',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'seahawk_rov = seahawk_rov:main'
        ],
    },
)
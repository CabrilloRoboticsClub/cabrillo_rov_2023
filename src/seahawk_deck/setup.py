import os
from glob import glob
from setuptools import setup

package_name = 'seahawk_deck'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.perspective'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maximus',
    maintainer_email='matera@lifealgorithmic.com',
    description='TODO: Package description',
    license='AGPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "thrust=seahawk_deck.thrust:main",
            "input_xbox_one=seahawk_deck.input_xbox_one:main",
            "rviz_markers=seahawk_deck.rviz_markers:main",
        ],
    },
)

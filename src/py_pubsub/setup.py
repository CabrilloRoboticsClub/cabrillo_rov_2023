from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert Garrett',
    maintainer_email='robertgarrett404@gmail.com',
    description='A toy package for learning the very basics of ROS packaging and the RCLPY API'
    license='APGLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher:main',
            'listener = py_pubsub.subscriber:main',
        ],
    },
)

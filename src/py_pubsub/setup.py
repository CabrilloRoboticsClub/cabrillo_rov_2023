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
    maintainer='dummy',
    maintainer_email='dummy@todo.todo',
    description='God why are there so many individual files that need to match but dont or cant'
    license='I said get out of my house. This is proprietary software (that I stole fair and square from the ROS docs)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher:main',
        ],
    },
)

from setuptools import setup

package_name = "our_examples"

setup(
    name=package_name,
    version="1.0.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name] ),
        ("share/" + package_name, ["package.xml"] )
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Garrett",
    maintainer_email="robertgarrett404@gmail.com",
    description="Example ROS Package providing a publisher and subscriber node.",
    license="AGPLv3",
    entry_points={
        "console_scripts": [
            "example_pub = our_examples.example_publisher.example_pub:main",
            "example_sub = our_examples.example_subscriber.example_sub:main",
        ]
    }
)
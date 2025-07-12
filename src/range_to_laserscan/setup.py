from setuptools import setup

package_name = 'range_to_laserscan'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Converts Range messages from ultrasonic sensors to LaserScan messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'range_to_laserscan_node = range_to_laserscan.range_to_laserscan_node:main',
        ],
    },
)

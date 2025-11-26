from setuptools import setup

package_name = 'raspberrypi_vitals'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CIDCO',
    maintainer_email='info@cidco.ca',
    description='ROS2 Raspberry Pi vitals publisher (port from ROS1).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raspberrypi_vitals_node = raspberrypi_vitals.raspberrypi_vitals_node:main',
        ],
    },
)

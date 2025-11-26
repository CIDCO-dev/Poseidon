from setuptools import setup

package_name = 'sonar_nmea_0183_tcp_client'

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
    description='ROS2 NMEA-0183 sonar TCP client.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'network_node = sonar_nmea_0183_tcp_client.network_node:main',
        ],
    },
)

from setuptools import setup

package_name = 'wifi_file_transfer_config'

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
    description='ROS2 WiFi file transfer configuration manager (nmcli).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_config_node = wifi_file_transfer_config.wifi_config_node:main',
        ],
    },
)

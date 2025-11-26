from setuptools import setup

package_name = 'power_management'

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
    description='ROS2 voltage monitor with graceful shutdown.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_management_node = power_management.power_management_node:main',
        ],
    },
)

from setuptools import setup

package_name = 'power_control'

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
    description='ROS2 GPIO power/LED control node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_control_node = power_control.power_control_node:main',
        ],
    },
)

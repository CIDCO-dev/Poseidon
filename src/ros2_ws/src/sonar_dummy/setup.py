from setuptools import setup

package_name = 'sonar_dummy'

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
    description='ROS2 dummy sonar publisher.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_dummy_node = sonar_dummy.sonar_dummy_node:main',
        ],
    },
)

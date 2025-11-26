from setuptools import setup

package_name = 'sonar_imagenex852_c'

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
    description='ROS2 port of Imagenex 852 sonar node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_imagenex852_node = sonar_imagenex852_c.sonar_imagenex852_node:main',
        ],
    },
)

from setuptools import setup

package_name = 'gnss_zed_f9p'

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
    description='ROS2 stub for ZED-F9P GNSS node.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_zed_f9p_node = gnss_zed_f9p.gnss_zed_f9p_node:main',
        ],
    },
)

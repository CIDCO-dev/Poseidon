from setuptools import setup

package_name = 'gnss_dummy'

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
    description='ROS2 dummy GNSS publisher.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_dummy_node = gnss_dummy.gnss_dummy_node:main',
        ],
    },
)

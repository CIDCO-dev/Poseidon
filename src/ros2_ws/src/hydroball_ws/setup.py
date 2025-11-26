from setuptools import setup

package_name = 'hydroball_ws'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='CIDCO',
    maintainer_email='info@cidco.ca',
    description='ROS2 Python nodes grouping config/data/files web sockets and diagnostics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'config_ws = hydroball_ws.config_ws:main',
            'data_ws = hydroball_ws.data_ws:main',
            'files_ws = hydroball_ws.files_ws:main',
            'diagnostics_ws = hydroball_ws.diagnostics_ws:main',
        ],
    },
)

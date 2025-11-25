from setuptools import setup

package_name = 'logger'

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
    description='ROS2 Python logger node skeleton mirroring the Poseidon logger service API.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logger_node = logger.logger_node:main',
            'logger_binary_node = logger.logger_binary_node:main',
            'logger_text_node = logger.logger_text_node:main',
        ],
    },
)

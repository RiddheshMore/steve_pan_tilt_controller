from setuptools import setup
import os
from glob import glob

package_name = 'steve_pan_tilt_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gokul Chenchani',
    maintainer_email='gchencha@uni-bonn.de',
    description='Pan-tilt controller for STEVE robot using Dynamixel XH430-W250 motors',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steve_pan_tilt_controller_node = steve_pan_tilt_controller.steve_pan_tilt_controller:main',
        ],
    },
)

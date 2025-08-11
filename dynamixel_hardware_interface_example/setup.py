from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'dynamixel_hardware_interface_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='Dynamixel Hardware Interface Example ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_xacros = dynamixel_hardware_interface_example.generate_xacros:main',
        ],
    },
)

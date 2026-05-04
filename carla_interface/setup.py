from setuptools import setup
import os
from glob import glob

package_name = 'carla_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.sensors',
        f'{package_name}.utils',
        f'{package_name}.environment',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 package to interface CARLA simulator with modular sensor and control support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_node = carla_interface.carla_node:main',
            'walker_node = carla_interface.walker_node:main',
        ],
    }
)

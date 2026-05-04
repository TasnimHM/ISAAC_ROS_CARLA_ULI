from setuptools import setup
import os
from glob import glob

package_name = 'carla_uli_isaacros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name,
            #   f'{package_name}.world',
              f'{package_name}.sensors',
            #   f'{package_name}.utils'
              ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'clock_synced_stereo_camera_node = carla_uli_isaacros.clock_synced_stereo_camera_node:main',
        'carla_uli_node = carla_uli_isaacros.carla_uli_node:main',
    ],
}

)

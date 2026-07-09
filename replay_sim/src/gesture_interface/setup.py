from setuptools import setup
import os
from glob import glob

package_name = 'gesture_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture_interface.launch.py']),
        ('share/' + package_name + '/config', ['config/gesture_params.yaml']),
        ('share/' + package_name + '/gestures', []),  # ← creates gestures folder on install
    ],
    install_requires=['setuptools', 'gesture_interface_msgs'],
    zip_safe=True,
    maintainer='Aswin Kumar',
    maintainer_email='aswin.kumar@gahanai.com',
    description='Gesture recording and playback interface using MoveIt 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gesture_manager_node = gesture_interface.gesture_manager_node:main',
        ],
    },
)
import os
from glob import glob
from setuptools import setup

package_name = 'rita_vision_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreyas',
    maintainer_email='shreyas148@gmail.com',
    description='Face detection node based on livestream_face_detect.py',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_tpu_node = rita_vision_control.yolo_tpu_node:main',
            'visual_servo_node = rita_vision_control.visual_servo_node:main',
            'moveit_commander_node = rita_vision_control.moveit_commander_node:main',
            'servo_commander_node = rita_vision_control.servo_commander_node:main',
            'yolo_webrtc_node = rita_vision_control.yolo_webrtc_node:main',
        ],
    },
)

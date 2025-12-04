from setuptools import setup
import os
from glob import glob

package_name = 'trash_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Team',
    maintainer_email='robotics@example.com',
    description='Autonomous trash bin robot with gesture control and navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_control = scripts.gesture_control:main',
            'mission_controller = scripts.mission_controller:main',
            'serial_bridge = scripts.serial_bridge:main',
        ],
    },
)

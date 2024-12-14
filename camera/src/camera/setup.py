from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'launch/'), glob("launch/*.launch.py")),
        (os.path.join('share', 'config/'), glob("config/*.yaml")),
        (os.path.join('share', 'rviz_cfg/'), glob("config/*.rviz")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergey',
    maintainer_email='sergey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pose_publisher = camera.position:main'
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'strategy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'config'), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timur Manshin',
    maintainer_email='TManshin@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy_node = strategy.strategy:main',
            'reinit_node = strategy.reinit:main',
            'reboot_node = strategy.reboot:main'
        ],
    },
)

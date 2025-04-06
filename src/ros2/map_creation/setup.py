from setuptools import find_packages, setup

package_name = 'map_creation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_creator_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='napalkov',
    maintainer_email='napalkov2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_creator = map_creation.map_creator:main',
        ],
    },
)

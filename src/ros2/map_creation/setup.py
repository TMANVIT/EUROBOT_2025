from setuptools import setup

package_name = 'map_creation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='napalkov',
    maintainer_email='napalkov2005@gmail.com',
    description='Map creation package for EUROBOT',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_creator = map_creation.map_creator:main',
        ],
    },
)
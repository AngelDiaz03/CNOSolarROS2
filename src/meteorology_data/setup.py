from setuptools import setup
import glob
import os

package_name = 'meteorology_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob(os.path.join('launch', '*.launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='manelonx@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "meteorology_node = meteorology_data.meteorology_node:main",
            "solar_park_node = meteorology_data.solar_park_node:main",
            "bridge_data_node = meteorology_data.bridge_data_node:main"
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ubicoders_hexdrone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #=============
        ('share/' + package_name, glob(os.path.join('launch', '*.launch.py')))
        #=============
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='anon@ubicoders.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hexdrone_ips_pub = ubicoders_hexdrone.node_ips_pub:main',
            'vdist_pub = ubicoders_hexdrone.node_vdist_pub:main',
            'auto_control = ubicoders_hexdrone.node_auto_control:main',
        ],
    },
)

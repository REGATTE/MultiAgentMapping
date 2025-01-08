from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config folder
        ('share/' + package_name + '/config', glob('config/*.yaml'))  # All YAML files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='regastation',
    maintainer_email='ashok.kumarj@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

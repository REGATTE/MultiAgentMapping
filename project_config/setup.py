from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project_config'

# Collect all YAML and config files, including subfolders
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Recursively include files from the config directory
for root, dirs, files in os.walk('config'):
    file_paths = [os.path.join(root, f) for f in files]  # Collect all files in the directory
    if file_paths:  # Add only if files exist
        dest = os.path.join('share', package_name, root)  # Destination path
        data_files.append((dest, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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

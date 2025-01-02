from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'drone_swarm_auto_setup_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files and subdirectories from created_resources
        *[(os.path.join('share', package_name, root), 
           [os.path.join(root, file) for file in files]) 
          for root, _, files in os.walk('created_resources')],
        # Include launch files from templates/launch
        ('share/' + package_name + '/templates/launch', 
            glob('templates/launch/*.py')),
        # Include the robots directory under templates/launch
        *[(os.path.join('share', package_name, root),
           [os.path.join(root, file) for file in files])
          for root, _, files in os.walk('templates/launch/robots')],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='basha',
    maintainer_email='bashar.ameen.ai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reset_swarm = drone_swarm_auto_setup_pkg.reset_swarm:main',
        ],
    },
)

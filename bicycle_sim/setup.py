from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bicycle_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARL Control Team',
    maintainer_email='arl@asu.edu.eg',
    description='Kinematic bicycle simulation plant and RViz bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = bicycle_sim.sim_node:main',
            'kinematic_bicycle = bicycle_sim.sim_node:main',
        ],
    },
)

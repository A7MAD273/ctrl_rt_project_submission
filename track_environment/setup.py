from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'track_environment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'tracks'), glob('tracks/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARL Control Team',
    maintainer_email='arl@asu.edu.eg',
    description='Racetrack geometry, path generator, and telemetry analyzer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_gen = track_environment.path_gen:main',
            'lap_analyzer = track_environment.lap_analyzer:main',
        ],
    },
)

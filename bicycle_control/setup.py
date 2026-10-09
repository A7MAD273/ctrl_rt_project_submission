from setuptools import find_packages, setup

package_name = 'bicycle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARL Control Team',
    maintainer_email='arl@asu.edu.eg',
    description='Longitudinal cruise control and lateral path tracking controllers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = bicycle_control.controller_node:main',
            'teleop_bridge = bicycle_control.teleop_bridge:main',
        ],
    },
)

from setuptools import setup
import glob
import os

package_name = 'planner'
nfop_planner = 'neural_field_optimal_planner'
collision_checker = 'neural_field_optimal_planner/collision_checker'
astar = 'neural_field_optimal_planner/astar'
utils = 'neural_field_optimal_planner/utils'

generated_map_mesh = 'generated_map_mesh'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, nfop_planner, collision_checker, astar, utils, generated_map_mesh ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iana',
    maintainer_email='iana.zhura@skoltech.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['call_planner = planner.call_planner:main',
        ],
    },
)

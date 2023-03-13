from setuptools import setup
import os
import glob

package_name = 'render'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh' ))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iana',
    maintainer_email='iana.zhura@skoltech.ru',
    description='Activate rendering after dataset collection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'call_inference = render.call_inference:main',
            'talker = render.bool_publisher:main',
        ],
    },
)

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'projet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sagnes Julien',
    maintainer_email='julien.sagnes@etu.sorbonne-universite.fr',
    description='Package of our project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_following = projet.line_following:main',
            'teleop = projet.mybot_teleop:main',
            'automatic_stop = projet.automatic_stop:main',
            'lds_distance = projet.lds_distance:main',
            'wall_follower = projet.wall_follower:main',
        ],
    },
)

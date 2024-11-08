from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'vision_atwork'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/vision.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juliogsabka',
    maintainer_email='cavalo.vendado@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = vision_atwork.service_member_function:main',
            'client = vision_atwork.client_member_function:main',
            'detections_organizer = vision_atwork.detections_organizer:main',
            'clean_obstacles = vision_atwork.clean_obstacles_file:main',
        ],
    },
)

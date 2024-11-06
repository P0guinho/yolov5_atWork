from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_atwork'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('yolo_atwork/configs/aruco_parameters.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='P0guinho',
    maintainer_email='cavalo.vendado@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws_recog_yolo = yolo_atwork.workspace_recognition:main',
            'ws_pose_estimate = yolo_atwork.workspace_pose_est:main',
            'obj_recog_yolo = yolo_atwork.object_recognition:main',
            'obj_pose_estimate = yolo_atwork.object_pose_est:main',
            'test = yolo_atwork.depth_test:main',
            'container_detection = yolo_atwork.container_detection:main'
        ],
    },
)

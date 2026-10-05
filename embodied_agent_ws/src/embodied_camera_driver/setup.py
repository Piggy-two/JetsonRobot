from glob import glob
import os

from setuptools import setup

package_name = 'embodied_camera_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piggy-two',
    maintainer_email='piggy@example.com',
    description='JetsonRobot Overlay 相机 Driver：时间戳重打 + TF 帧补发',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver = embodied_camera_driver.camera_driver:main',
        ],
    },
)

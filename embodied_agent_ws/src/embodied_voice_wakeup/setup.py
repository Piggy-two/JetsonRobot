from glob import glob
import os

from setuptools import setup

package_name = 'embodied_voice_wakeup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piggy-two',
    maintainer_email='piggy@example.com',
    description='JetsonRobot Overlay 语音唤醒（本地 KWS，不经厂商唤醒节点）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wakeup_node = embodied_voice_wakeup.wakeup_node:main',
        ],
    },
)

from glob import glob
import os

from setuptools import setup

package_name = 'embodied_vision_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piggy-two',
    maintainer_email='piggy@example.com',
    description='Vision Driver：画面里有没有某个东西、在哪一侧（不知道 ≠ 没有）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_driver = embodied_vision_driver.vision_driver:main',
        ],
    },
)

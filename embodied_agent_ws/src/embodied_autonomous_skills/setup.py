from glob import glob
import os

from setuptools import setup

package_name = 'embodied_autonomous_skills'

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
    description='JetsonRobot Autonomous Skill：advance_until_blocked（闭环，未真机验证）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_skills = embodied_autonomous_skills.autonomous_skills:main',
        ],
    },
)

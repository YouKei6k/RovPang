from glob import glob
from setuptools import setup

package_name = 'rovpang_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='정서인',
    description='RovPang mission manager and mock nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = rovpang_mission.mission_manager:main',
            'mock_robot = rovpang_mission.mock_robot:main',
            'mock_vision = rovpang_mission.mock_vision:main',
            'send_request = rovpang_mission.send_request:main',
        ],
    },
)

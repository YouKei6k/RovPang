from glob import glob
from setuptools import setup

package_name = 'rovpang_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.mock'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='정서인',
    description='RovPang delivery task manager, recipient auth and mock nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager = rovpang_manager.task_manager:main',
            'mock_robot = rovpang_manager.mock.mock_robot:main',
            'mock_vision = rovpang_manager.mock.mock_vision:main',
            'send_request = rovpang_manager.mock.send_request:main',
        ],
    },
)

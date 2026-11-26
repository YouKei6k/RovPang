from glob import glob
from setuptools import setup

package_name = 'rovpang_bringup'

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
    maintainer='YouKei6k',
    maintainer_email='bethe0000@gmail.com',
    description='RovPang 통합 실행',
    license='Apache-2.0',
)

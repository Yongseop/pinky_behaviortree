from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pinky_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 리소스 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config 파일 설치
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # rviz2
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='SLAM package for Pinky robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 여기에 실행 가능한 스크립트 추가 (필요 시)
        ],
    },
)

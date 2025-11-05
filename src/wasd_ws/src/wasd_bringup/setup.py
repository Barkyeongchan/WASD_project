from setuptools import setup
from glob import glob
import os

package_name = 'wasd_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 패키지 인덱스용 리소스
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # 🔽 launch/*.py 를 install/share/wasd_bringup/launch 로 설치
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wasd',
    maintainer_email='you@example.com',
    description='WASD project bringup package for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = wasd_bringup.teleop_keyboard:main',
            'wasd_goal_proxy = wasd_bringup.wasd_goal_proxy:main',
            # 지금은 실행할 노드 없음
        ],
    },
)
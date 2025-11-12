from setuptools import setup
import os
from glob import glob

package_name = 'wasd_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # wasd_bridge/ 디렉토리를 파이썬 패키지로 설치
    data_files=[
        # ament index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # config (points.yaml 설치)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wasd',
    maintainer_email='guysaint@naver.com',
    description='WASD bridge between UI and Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run wasd_bridge wasd_bridge
            'wasd_bridge = wasd_bridge.wasd_bridge_node:main',
        ],
    },
)
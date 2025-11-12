from setuptools import find_packages, setup

package_name = 'wasd_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/points.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wasd',
    maintainer_email='guysaint@naver.com',
    description='WASD Bridge node for UI-ROS communication',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'wasd_bridge = wasd_bridge.wasd_bridge_node:cli_main',
        ],
    },
)
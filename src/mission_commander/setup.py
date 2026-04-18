from setuptools import find_packages, setup

package_name = 'mission_commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='elijah',
    maintainer_email='elijah@gmail.com',
    description='ROS2 package for PX4 autonomous missions',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'box_delivery = mission_commander.box_delivery:main',
            'gate_passing = mission_commander.gate_passing:main',
            'path_following = mission_commander.path_following:main',
            'whiteboard_drawing = mission_commander.whiteboard_drawing:main',
            'attitude_step = mission_commander.Attitude_step:main'
        ],
    },
)

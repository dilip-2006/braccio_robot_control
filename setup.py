from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files (YAML, SRDF, rviz)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        # URDF / xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        # STL mesh files
        (os.path.join('share', package_name, 'stl'),
            glob('stl/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dk',
    maintainer_email='dk@todo.todo',
    description='Braccio robotic arm control – ROS2 Humble port of braccio_moveit_gazebo',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'braccio_commander = robot_control.braccio_commander:main',
            'joint_state_republisher = robot_control.joint_state_republisher:main',
            'cv_hand_control = robot_control.cv_hand_control:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'slam_to_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sgw',
    maintainer_email='sgw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		    'slam_to_vision = slam_to_vision.slam_to_vision_node:main',
            'offboard_velocity = slam_to_vision.offboard_velocity:main',
            'offboard_position = slam_to_vision.offboard_position:main',
            'nav2_cmd = slam_to_vision.cmdVelToRover:main'
        ],
    },
)

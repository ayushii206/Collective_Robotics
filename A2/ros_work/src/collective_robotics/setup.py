from setuptools import find_packages, setup

package_name = 'collective_robotics'

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
    maintainer='student',
    maintainer_email='kamrankhowaja@yahoo.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation= collective_robotics.task1:main',
            'obstacle_avoid= collective_robotics.obstacle_avoidance:main',
            'wall_follower= collective_robotics.wall_follower:main',
            'go_to_goal= collective_robotics.move_to_goal:main',
            'vacum_bot= collective_robotics.vaccum_robot:main',
        ],
    },
)

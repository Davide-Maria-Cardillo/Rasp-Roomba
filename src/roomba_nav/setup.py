from setuptools import find_packages, setup

package_name = 'roomba_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/roomba_nav/launch', ['launch/roomba_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dumbo',
    maintainer_email='dumbo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = roomba_nav.drive_node:main',
            'vel_serial_node = roomba_nav.vel_serial_node:main',
            'sensors_node = roomba_nav.sensors_node:main',
            'navigation_node = roomba_nav.navigation_node:main',
            'cmd_vel_bridge = roomba_nav.cmd_vel_bridge:main'
        ],
    },
)

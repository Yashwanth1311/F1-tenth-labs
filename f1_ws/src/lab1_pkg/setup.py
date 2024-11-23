from setuptools import find_packages, setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'rclpy', 'ackermann_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            ('share/' + package_name + '/launch',  # Include this line to specify the launch directory
             ['launch/lab1_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='yashwanth',
    maintainer_email='yashwanth@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = lab1_pkg.talker:main',
            'relay = lab1_pkg.relay:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'airsim_ros2_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_airsim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dimitris',
    maintainer_email='dimitrisgegas01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'airsim_twist_bridge = airsim_ros2_teleop.airsim_twist_bridge:main',
            'teleop_twist_keyboard = airsim_ros2_teleop.teleop_twist_keyboard:main',
        ],
    },
)

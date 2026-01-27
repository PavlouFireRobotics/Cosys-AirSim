from setuptools import find_packages, setup

package_name = 'bt_airsim_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bt_airsim_ros.launch.py']),

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
            'airsim_simple_actions = bt_airsim_ros.airsim_simple_actions:main',
        ],
    },
)

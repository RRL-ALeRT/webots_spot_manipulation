from glob import glob

from setuptools import find_packages, setup

package_name = 'webots_spot_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='m.kirsch@fh-aachen.de',
    description='Action server to move the manipulator end effector by a delta or to a TF frame',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_server = webots_spot_manipulation.manipulator_server:main',
            'manipulator_client = webots_spot_manipulation.manipulator_client:main',
            'gripper_server = webots_spot_manipulation.gripper_server:main',
            'gripper_client = webots_spot_manipulation.gripper_client:main',
        ],
    },
)

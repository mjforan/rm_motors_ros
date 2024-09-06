from setuptools import find_packages, setup
from glob import glob

package_name = 'rrbot_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('description/launch/*')),
        ('share/' + package_name + '/ros2_control/', glob('description/ros2_control/*')),
        ('share/' + package_name + '/urdf/', glob('description/urdf/*')),
        ('share/' + package_name + '/launch/', glob('bringup/launch/*')),
        ('share/' + package_name + '/config/', glob('bringup/config/*')),
        ('share/' + package_name + '/rrbot/urdf/', glob('rrbot/urdf/*')),
        ('share/' + package_name + '/rrbot/rviz/', glob('rrbot/rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mjforan',
    maintainer_email='matthewjforan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

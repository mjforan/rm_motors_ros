from setuptools import find_packages, setup
from glob import glob

package_name = 'gm6020_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf/', glob('urdf/*')),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
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

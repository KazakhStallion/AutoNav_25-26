from setuptools import setup
from setuptools import find_packages

package_name = 'autonav_automated_testing'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/automated_testing.launch.py']),
    ],
    install_requires=['setuptools', 'PyQt6'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@todo.com',
    description='Automated testing GUI for AutoNav project',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
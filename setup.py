from setuptools import find_packages, setup

package_name = 'roboguard_estop_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/estop_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Club Capra ETS',
    maintainer_email='capra@ens.etsmtl.ca',
    description='E-stop management package for RoboGuard (Capra ETS)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estop_manager = roboguard_estop_package.estop_manager:main',
            'estop_hardware_monitor = roboguard_estop_package.estop_hardware_monitor:main',
            'estop_heartbeat_unifier = roboguard_estop_package.estop_heartbeat_unifier:main',
            'roboguard_powersupply_toggler = roboguard_estop_package.roboguard_powersupply_toggler:main',
        ],
    },
)
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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Club Capra ETS',
    maintainer_email='capra@ens.etsmtl.ca',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboguard_estop_monitor = roboguard_estop_package.roboguard_estop_monitor:main',
            'roboguard_powersupply_modetoggler = roboguard_estop_package.roboguard_powersupply_modetoggler:main',
        ],
    },
)

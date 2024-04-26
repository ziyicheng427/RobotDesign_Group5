from setuptools import find_packages, setup

package_name = 'arm_test_pkg'

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
    maintainer='sfr2023-vm',
    maintainer_email='sfr2023-vm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        'publisher_control_node = '
        'arm_test_pkg.publisher_control_node:main',
        
        'subscriber_control_node = '
        'arm_test_pkg.subscriber_control_node:main',
        
        'flag_control_node = '
        'arm_test_pkg.flag_control_node:main',
        
        'flag_subscriber_node = '
        'arm_test_pkg.flag_subscriber_node:main',
        ],
    },
)

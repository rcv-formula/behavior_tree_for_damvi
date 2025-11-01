from setuptools import find_packages, setup

package_name = 'state_machine'

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
    maintainer='rcv',
    maintainer_email='dewfresh1@inha.edu',
    description='State machine of damvi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_main = state_machine.bt_main:main',
	    'health_monitor = state_machine.health_monitor:main',
	    'fake_path = state_machine.fake_path:main',
	    'state_machine_tester = state_machine.state_machine_tester:main',
        ],
    },
)

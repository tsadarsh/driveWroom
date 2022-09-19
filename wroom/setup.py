from setuptools import setup

package_name = 'wroom'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adarshts',
    maintainer_email='tsadarsh0707@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_drive = wroom.joy_to_drive:main',
            'sabertooth_serial_comm = wroom.sabertooth_serial_comm:main',
        ],
    },
)

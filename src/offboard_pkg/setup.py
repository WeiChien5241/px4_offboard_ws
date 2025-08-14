from setuptools import find_packages, setup

package_name = 'offboard_pkg'

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
    maintainer='weichien241',
    maintainer_email='weichien241@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_control = offboard_pkg.gui_control:main',
            'vel_ctrl = offboard_pkg.vel_ctrl:main',
            'mavsdk_gui_control = offboard_pkg.mavsdk_gui_control:main',
        ],
    },
)

from setuptools import setup

package_name = 'fleet_driver_mir'

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
    maintainer='Eric Dong',
    maintainer_email='eric.dongxx@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['fleet_driver_mir=fleet_driver_mir.fleet_driver_mir:main',
            'remove_mc=fleet_driver_mir.remove_move_coordinate:main',
            'mir_position_converter=fleet_driver_mir.report_positions_in_rmf_coord:main']
    },
)

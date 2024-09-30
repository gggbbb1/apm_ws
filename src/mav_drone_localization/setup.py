from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mav_drone_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.*')))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='xx@gg.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rome_ekf_wrapper = mav_drone_localization.rome_ekf_wrapper:main',
            'ekf_test_bench = mav_drone_localization.ekf_test_bench:main',
        ],
    },
    scripts=[
    'scripts/record_bag.sh',
    'scripts/play_bag.sh',

    ]
)

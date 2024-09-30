from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'of_poc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))

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
            'of_farneback_node = of_poc.of_farneback_node:main',
            'of_lucas_node = of_poc.of_lucas_node:main',
            'tf_handler = of_poc.tf_handler:main',
            'test_twist_vs_gz = of_poc.test_twist_vs_gz:main'
        ],
    },
)

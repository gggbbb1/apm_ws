from setuptools import find_packages, setup

package_name = 'apm_application'

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
    maintainer='user',
    maintainer_email='robo2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "od=apm_application.object_detection:main",
            "gimbal_control=apm_application.gimbal_controller:main",
            "tf_handler=apm_application.tf_broadcaster:main"
        ],
    },
)

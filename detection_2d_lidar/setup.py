from setuptools import setup

package_name = 'detection_2d_lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shengjian Chen, Steven Macenski',
    maintainer_email='csj15thu@gmail.com, stevenmacenski@gmail.com',
    description='2D LIDAR detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_2d_lidar_node = detection_2d_lidar.detection_2d_lidar_node:main'
        ],
    },
)

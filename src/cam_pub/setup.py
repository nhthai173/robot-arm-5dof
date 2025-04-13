from setuptools import find_packages, setup

package_name = 'cam_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='nht',
    maintainer_email='nht173@gmail.com',
    description='Publish camera images using ROS2 and Python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = cam_pub.pub:main',
        ],
    },
)

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision3D'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='lord.daniel.w@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub = vision3D.img_pub:main',                    
            'image_to_laser = vision3D.image_to_laser:main',            
            'obstacle_detection = vision3D.obstacle_detection:main',
        ],
    },
)

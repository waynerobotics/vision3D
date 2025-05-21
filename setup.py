from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision3D'

# Function to collect all files recursively
def collect_files_recursively(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(path, filename)
            paths.append(filepath)
    return paths

# Collect all model files recursively
model_files = collect_files_recursively('vision3D/finetuned_model')

# Create data_files list
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# Add all files from finetuned_model recursively, preserving directory structure
for filepath in model_files:
    # Get the relative path to maintain directory structure in the share folder
    relative_path = os.path.dirname(filepath.replace('vision3D/', ''))
    target_path = os.path.join('share', package_name, relative_path)
    data_files.append((target_path, [filepath]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
            'img_serve = vision3D.img_serve:main',
            'lane_segmentation_to_pointcloud = vison3D.lane_segmentation_to_pointcloud:main',
        ],
    },
)

from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpu_mag_fusion'

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
    install_requires=['setuptools','ahrs'],
    zip_safe=True,
    maintainer='soroush',
    maintainer_email='soroush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'orientation_publisher = mpu_mag_fusion.orientation_publisher:main',
        'madgwick_to_yaw = mpu_mag_fusion.madgwick_to_yaw:main',
        'catch_mag_data = mpu_mag_fusion.catch_mag_data:main',


    ],
},


)

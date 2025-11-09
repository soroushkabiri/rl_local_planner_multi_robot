from setuptools import find_packages, setup

package_name = 'rl_planner'

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
    maintainer='soroush',
    maintainer_email='soroush.kabiri.92@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [


        'observation_states = rl_planner.observation_states:main',
        'TD3 = rl_planner.TD3:main',
        'td3_actor_visualizer = rl_planner.td3_actor_visualizer:main',


        ],
    },
)

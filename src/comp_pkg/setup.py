from setuptools import find_packages, setup

package_name = 'comp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_yaw.launch.py']),
        #('share/' + package_name + '/launch', ['launch/combined.launch.py']),
        ('share/' + package_name + '/launch', ['launch/velocity_publishing.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nodes_before_nav2_slamtoolbox.launch.py']),


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
                    'imu_yaw_node = comp_pkg.imu_yaw_node:main',
                    'initial_align_node = comp_pkg.initial_allignment_node:main',
                    'test_node = comp_pkg.test_node:main',
                    'state_consensus_node = comp_pkg.state_consensus_node:main',
                    'pub_des_vel_node = comp_pkg.pub_des_vel_node:main',
                    'pub_des_vel_node_rl = comp_pkg.pub_des_vel_node_rl:main',

                    
                    'gazebo_to_odom = comp_pkg.gazebo_to_odom:main',
                    'gazebo_to_odom_rl = comp_pkg.gazebo_to_odom_rl:main',

                    'laser_relay = comp_pkg.laser_relay:main',

                    'repub_kinect = comp_pkg.repub_kinect:main',
                    'make_timestamp_rgb_for_rtab_odom = comp_pkg.make_timestamp_rgb_for_rtab_odom:main',
                    'make_timestamp_rgb_for_rtab_odom_sim = comp_pkg.make_timestamp_rgb_for_rtab_odom_sim:main',

                    'compute_closest_obstacle = comp_pkg.compute_closest_obstacle:main',
                    'compute_closest_obstacle_rl = comp_pkg.compute_closest_obstacle_rl:main',

                    'catch_waypoints = comp_pkg.catch_waypoints:main',

                    'fuzzy_planner = comp_pkg.fuzzy_planner:main',
                    'global_path_manager = comp_pkg.global_path_manager:main',
                    'bag_recorder = comp_pkg.bag_recorder:main',
                    'bag_plotter = comp_pkg.bag_plotter:main',
                    'consensus_tester = comp_pkg.consensus_tester:main',
                    'gazebo_reset_node = comp_pkg.gazebo_reset_node:main',

        ],
    },
)

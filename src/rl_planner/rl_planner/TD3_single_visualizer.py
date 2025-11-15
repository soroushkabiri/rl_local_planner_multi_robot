#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import tensorflow as tf
from tensorflow.keras import Model, layers
from datetime import datetime
import matplotlib.pyplot as plt
from collections import deque
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool, Float32
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
import threading
import os
import pandas as pd


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def clip_normalize_distance_to_goal(dist, max_range=15.0):
    """Clip distance to max_range and normalize to [0,1]."""
    return min(dist / max_range, 1.0)

def clip_normalize_distance(dist, max_range=1.0):
    """Clip distance to max_range and normalize to [0,1]."""
    return min(dist / max_range, 1.0)

def make_relative_state(state_uncomplete, current_wp,prev_action):
    """
    Convert raw observation vector into relative distances & angles.
    state_uncomplete = [rect_x, rect_y, leader_theta, closest_obs_x, closest_obs_y, goal_x, goal_y]
    current_wp       = [wp_x, wp_y]  # from your waypoint function
    """

    # unpack state
    rect_x, rect_y, leader_theta = state_uncomplete[0], state_uncomplete[1], state_uncomplete[2]
    closest_obs_x, closest_obs_y        = state_uncomplete[3], state_uncomplete[4]
    goal_x, goal_y = state_uncomplete[-2], state_uncomplete[-1]
    wp_x, wp_y      = current_wp[0], current_wp[1]
    v, w = prev_action[0], prev_action[1]


    # Distance to final goal
    dx_to_goal = goal_x - rect_x
    dy_to_goal = goal_y - rect_y
    dist_to_goal = np.sqrt(dx_to_goal**2 + dy_to_goal**2)
    dist_to_goal_norm = clip_normalize_distance_to_goal(dist_to_goal)

    # Distance to waypoint
    dx_wp = wp_x - rect_x
    dy_wp = wp_y - rect_y
    dist_to_wp = np.sqrt(dx_wp**2 + dy_wp**2)
    dist_to_wp_norm = clip_normalize_distance(dist_to_wp)

    # Angle to waypoint in world
    angle_to_wp_global = math.atan2(dy_wp, dx_wp)
    # Relative heading in robot frame
    angle_to_wp = normalize_angle(angle_to_wp_global - leader_theta)
    sin_angle_to_wp=math.sin(angle_to_wp)
    cos_angle_to_wp=math.cos(angle_to_wp)
    #angle_to_wp_norm = angle_to_wp / np.pi  # maps [-pi,pi] → [-1,1]

    # Distance to closest obstacle
    dx_obs = closest_obs_x - rect_x
    dy_obs = closest_obs_y - rect_y
    dist_to_obs = np.sqrt(dx_obs**2 + dy_obs**2)
    # Improved normalized distance (importance weighting)
    min_dist, max_dist = 1.35, 1.6
    improved_dist_to_obs = (max_dist - dist_to_obs) / (max_dist - min_dist)
    improved_dist_to_obs = np.clip(improved_dist_to_obs, 0.0, 1.0)

    # Angle to obstacle in world
    angle_to_obs_global = math.atan2(dy_obs, dx_obs)
    # Relative heading
    angle_to_obs = normalize_angle(angle_to_obs_global - leader_theta)
    sin_angle_to_obs=math.sin(angle_to_obs)
    cos_angle_to_obs=math.cos(angle_to_obs)
    # Normalize obstacle distance and angle
    #dist_to_obs_norm = clip_normalize_distance(dist_to_obs)
    #angle_to_obs_norm = angle_to_obs / np.pi

    # Leader theta normalized
    leader_theta_norm = normalize_angle(leader_theta) / np.pi

    # Final RL state vector
    rl_state = np.array([
        dist_to_goal_norm,
        dist_to_wp_norm,
        sin_angle_to_wp,
        cos_angle_to_wp,
        #angle_to_wp_norm,
        improved_dist_to_obs,
        sin_angle_to_obs,
        cos_angle_to_obs,
        #angle_to_obs_norm,
        #leader_theta_norm,
        v,
        w,
    ], dtype=np.float32)

    return rl_state


class GazeboResetClient():
    def __init__(self, node):
        self.node = node

        # Create clients for Gazebo control services
        self.cli_pause = self.node.create_client(Empty, '/pause_physics')
        self.cli_unpause = self.node.create_client(Empty, '/unpause_physics')
        self.cli_reset_sim = self.node.create_client(Empty, '/reset_simulation')

        # Wait for services to be available
        for cli, name in [
            (self.cli_pause, '/pause_physics'),
            (self.cli_unpause, '/unpause_physics'),
            (self.cli_reset_sim, '/reset_simulation')
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(f'Waiting for {name} service...')

        self.req = Empty.Request()

    def pause(self):
        self.node.get_logger().info('Pausing physics...')
        future = self.cli_pause.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Physics paused.')

    def unpause(self):
        self.node.get_logger().info('Unpausing physics...')
        future = self.cli_unpause.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Physics unpaused.')

    def reset_simulation(self):
        self.node.get_logger().info('Resetting full simulation (time + world)...')
        future = self.cli_reset_sim.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info('Simulation reset done!')

# class ReplayBuffer:
#     def __init__(self, max_size):
#         self.buffer = deque(maxlen=max_size)
#         self.max_size = max_size

#     def store(self, state, action, reward, next_state, done):
#         # Convert everything to float32 when storing
#         state = np.array(state, dtype=np.float32)
#         action = np.array(action, dtype=np.float32)
#         reward = np.float32(reward)  # Convert scalar to float32
#         next_state = np.array(next_state, dtype=np.float32)
#         done = np.float32(done)  # Convert boolean to float32
        
#         experience = (state, action, reward, next_state, done)
#         self.buffer.append(experience)

#     def sample_batch(self, batch_size):
#         if batch_size > len(self.buffer):
#             batch_size = len(self.buffer)

#         indices = np.random.choice(len(self.buffer), batch_size, replace=False)
#         states = []
#         actions = []
#         rewards = []
#         next_states = []
#         dones = []

#         for idx in indices:
#             s, a, r, s2, d = self.buffer[idx]
#             states.append(s)
#             actions.append(a)
#             rewards.append(r)
#             next_states.append(s2)
#             dones.append(d)

#         return {
#             's': np.array(states, dtype=np.float32),
#             'a': np.array(actions, dtype=np.float32),
#             'r': np.array(rewards, dtype=np.float32),
#             's2': np.array(next_states, dtype=np.float32),
#             'd': np.array(dones, dtype=np.float32)
#         }
    
#     def __len__(self):
#         return len(self.buffer)

class Actor(Model):
    def __init__(self, action_dim, action_max, hidden_sizes=(256,128,64,)):
        super().__init__()
        self.action_max = tf.constant(action_max, dtype=tf.float32)  # shape (2,)

        self.hidden_layers = [layers.Dense(h, activation='relu') for h in hidden_sizes]
        self.output_layer = layers.Dense(action_dim, activation='tanh')

    def call(self, state):
        x = state
        for lyr in self.hidden_layers:
            x = lyr(x)
                
        raw_action = self.output_layer(x)  # in [-1, 1]

        #return self.output_layer(x) * self.action_max
        return raw_action * tf.stop_gradient(self.action_max)


# class Critic(Model):
#     def __init__(self, hidden_sizes=(256,128,64,)):
#         super().__init__()
#         self.hidden_layers = [layers.Dense(h, activation='relu') for h in hidden_sizes]
#         self.output_layer = layers.Dense(1)

#     def call(self, inputs):
#         state, action = inputs
#         x = tf.concat([state, action], axis=-1)
#         for lyr in self.hidden_layers:
#             x = lyr(x)
#         return tf.squeeze(self.output_layer(x), axis=1)
#         #return self.output_layer(x)   # shape (batch, 1)


# ROS2 TD3 AGENT NODE
class TD3AgentNode(Node):
    def __init__(self, hidden_sizes=(256,128,64,), replay_size=int(5e3), mu_lr=1e-3, q_lr=1e-3,
        gamma=0.99, decay=0.995, batch_size=64, action_noise=0.1, target_noise=0.2,
        noise_clip=0.5, policy_delay=2,max_episode_length=1500):
        super().__init__("td3_agent")
        
        self.resume_training = True



        # # Define directory for saving training history
        # self.history_dir = os.path.expanduser('~/ros_for_project_1/articulate_robot/td3_history_single')
        # os.makedirs(self.history_dir, exist_ok=True)

        # # Define full path for training log
        # self.history_file = os.path.join(self.history_dir, "td3_training_history.xlsx")

        # Initialize empty DataFrame with columns
        # self.history_columns = ["total_it", "episode", "episode_return", "actor_loss", "critic1_loss", "critic2_loss"]
        # self.training_history = pd.DataFrame(columns=self.history_columns)


        # save network weights
        self.save_dir = os.path.expanduser("~/ros_for_project_1/articulate_robot/td3_weights_single")
        os.makedirs(self.save_dir, exist_ok=True)

        # Continue training flag
        self.resume_training = True  # set False if you want fresh training

        # Directory containing final weights
        self.load_dir = os.path.join(self.save_dir, "episode_final")

        # Gazebo reset helper
        self.gz_reset = GazeboResetClient(self)

        self.gamma = gamma
        self.decay = decay
        self.batch_size = batch_size
        self.action_noise = action_noise
        self.target_noise = target_noise                # TD3-specific: noise added to target actions
        self.noise_clip = noise_clip                    # TD3-specific: clipping of target noise
        self.policy_delay = policy_delay                # TD3-specific: delayed policy updates
        self.max_episode_length = max_episode_length

        self.last_obs = None
        
        #extract environment dimensions
        self.num_states = 9
        self.num_actions = 2  # linear and angular velocity
        self.max_v_leader=0.15
        self.max_w_leader=0.3
        self.action_max=np.array([self.max_v_leader, self.max_w_leader], dtype=np.float32)  # max linear and angular velocity

        # Subscribe to observation topic
        self.create_subscription(Float32MultiArray, '/observation_state', self.observation_callback, 10)
        self.create_subscription(Path, "/RL_path", self.path_callback, 10)
        # Save downsampled waypoints
        self.waypoints = []  # list of (x, y)
        self.current_wp_idx = 0

        # Publish to robot cmd_vel
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_fuzzy", 10)
       
        # Create networks
        self.actor = Actor(self.num_actions, self.action_max, hidden_sizes)


        # Build networks (initialize weights)
        dummy_state = tf.zeros([1, self.num_states])
        dummy_action = tf.zeros([1, self.num_actions])
        self.actor(dummy_state)


        ### Load weights if continuing training
        if self.resume_training and os.path.exists(self.load_dir):
            try:
                self.actor.load_weights(os.path.join(self.load_dir, "actor.h5"))

                self.get_logger().info("✅ Loaded previous weights — continuing training!")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to load previous weights: {e}")
                self.get_logger().warn("Starting training from scratch.")

        else:
            self.get_logger().info("ℹ️ No previous weights found — starting from scratch.")

        
        # Create optimizers
        self.actor_optimizer = tf.keras.optimizers.Adam(mu_lr)


        self.get_logger().info(f"Actor variables: {len(self.actor.trainable_variables)}")


        self.actor.summary()


        
        # Initialize step counter for delayed policy updates
        self.total_it = 0



    #catch the whole waypoints
    def path_callback(self, msg: Path):
        """Store downsampled path and its order."""
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        #self.get_logger().info(f"Saved {len(self.waypoints)} downsampled waypoints.")

    def update_waypoint(self,uncomplete_state):
        rect_obj_x, rect_obj_y= uncomplete_state[0], uncomplete_state[1]
        current_wp=self.waypoints[self.current_wp_idx]
        distance=math.hypot(current_wp[0]-rect_obj_x,current_wp[1]-rect_obj_y)

        waypoint_changed = False

        # check if the rect obj distance to current waypoint is closer than 0.3 meter then go to the next waypoint
        if distance<=0.3:
            self.current_wp_idx=min(self.current_wp_idx+1,len(self.waypoints)-1 )
            waypoint_changed = True

        return np.array(self.waypoints[self.current_wp_idx], dtype=np.float32), waypoint_changed


    def reward_calculator(self,uncomplete_states,current_wp):
        reward=0

        rect_obj_x, rect_obj_y=uncomplete_states[0],uncomplete_states[1]
        final_goal_x, final_goal_y=uncomplete_states[-2], uncomplete_states[-1]
        closest_obstacle_x, closest_obstacle_y=uncomplete_states[3], uncomplete_states[4]
        leader_heading = uncomplete_states[2]  # yaw angle (rad)

        distance_to_goal=math.hypot(rect_obj_x-final_goal_x, rect_obj_y-final_goal_y)
        distance_to_obstacle=math.hypot(rect_obj_x-closest_obstacle_x,rect_obj_y-closest_obstacle_y)
        distance_to_waypoint=math.hypot(rect_obj_x-current_wp[0],rect_obj_y-current_wp[1])

        angle_to_wp = math.atan2(current_wp[1] - rect_obj_y, current_wp[0] - rect_obj_x)

        # Normalize angle to [-pi, pi]
        angle_to_wp = (angle_to_wp + math.pi) % (2 * math.pi) - math.pi

        leader_heading = (leader_heading + math.pi) % (2 * math.pi) - math.pi

        # --- Heading error (difference) in range [-pi, pi] ---
        heading_error = angle_to_wp - leader_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
        
        # stronger penalty for large angular error
        if abs(heading_error)>1.0:
            reward -= 0.1 * abs(heading_error)

        #if distance_to_goal <=0.3:
        #    reward+=20
        if distance_to_obstacle<=1.35:
            reward-=20
        if distance_to_waypoint<=0.3:
            reward+=3
        # negative reward for every time step
        reward-=0.01
        # progress reward
        if self.prev_distance_to_wp is not None:
            progress = self.prev_distance_to_wp - distance_to_waypoint  # >0 means improvement
            if progress > 0:
                reward += 1.0 * progress   # strong reward for moving closer
            else:
                reward += 4.0 * progress   # stronger punishment for moving away (progress is negative)

        # Update previous distance for next step
        self.prev_distance_to_wp = distance_to_waypoint

        if distance_to_goal<=0.3 or distance_to_obstacle<=1.35:
            Termination=True
        else :
            Termination=False

        return reward, Termination

    # publish the action of rl agent
    def send_action_to_robot(self, action):
        # action = [v, w]
        msg = Twist()
        #if float(action[0])>0:
        msg.linear.x = float(action[0])
        #else:
        #    msg.linear.x=0.0       
        msg.angular.z = float(action[1])
        self.cmd_pub.publish(msg)


    # function for reseting environment at start of every episode
    def reset_environment(self):
        self.get_logger().info("Resetting environment...")
        self.gz_reset.pause()
        time.sleep(4)
        self.gz_reset.reset_simulation()
        time.sleep(3)
        self.gz_reset.unpause()
        time.sleep(3)

    def get_action(self, s, noise_scale):
        a = self.actor(tf.convert_to_tensor(s.reshape(1,-1), dtype=tf.float32))
        a = a.numpy()[0]
        a += noise_scale * np.random.randn(self.num_actions)
        return np.clip(a, -self.action_max, self.action_max)
    


    def train(self, num_episodes):


        for episode in range(num_episodes):
            self.get_logger().info(f" episode number: {episode+1}")
            self.current_wp_idx = 0

            self.reset_environment()
            self.send_action_to_robot(np.array([0,0]))
            # Run automatic alignment only once per episode
            time.sleep(4.0)
            # Spin a few times to ensure callbacks update self.last_obs
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.01)
            #rclpy.spin_once(self)  # **keep callbacks alive**
            self.prev_distance_to_wp = None
            state_uncomplete=self.last_obs
            current_waypoint, wp_changed=self.update_waypoint(state_uncomplete)
            #state_reward = np.concatenate([current_waypoint, state_uncomplete], axis=0)  # shape (19,)
            prev_action=np.array([0,0])
            state_rl = make_relative_state(state_uncomplete, current_waypoint,prev_action)
            
            self.get_logger().info(f"new_states: {state_rl}, its shape is: {state_rl.shape}")

            episode_return, episode_length=0, 0
            
            #state, episode_return, episode_length = self.env.reset()[0], 0, 0
            done = False

            self.prev_distance_to_wp = math.hypot(
                state_uncomplete[0] - current_waypoint[0],
                state_uncomplete[1] - current_waypoint[1])

            # ---- Per-episode temporary storage for logging ----

            while not (done or episode_length == self.max_episode_length):
                
                #rclpy.spin_once(self)  # **keep callbacks alive**
                

                action = self.get_action(state_rl, self.action_noise)





                # # Use agent's actions only after buffer has enough samples
                # if len(self.replay_buffer) >= self.replay_buffer.max_size: #self.batch_size:
                #     action = self.get_action(state_rl, self.action_noise)
                # else:
                #     #action = self.env.action_space.sample()
                #     action = np.random.uniform(low=-self.action_max, high=self.action_max)

                time.sleep(0.01)

                for _ in range(3):
                    rclpy.spin_once(self, timeout_sec=0.01)

                #save previous action to use later in states
                prev_action=action
                # Take action in environment
                self.send_action_to_robot(action)
                # wait till the action implemented on environment
                time.sleep(0.1)
                self.send_action_to_robot(np.array([0,0]))

                # read the uncomplete state after implementing action to calculate reward

                #rclpy.spin_once(self)  # **keep callbacks alive**

                for _ in range(3):
                    rclpy.spin_once(self, timeout_sec=0.01)

                next_state_uncomplete=self.last_obs
                # calculate reward 
                reward ,done=self.reward_calculator(next_state_uncomplete,current_waypoint)
                #self.get_logger().info(f"next_state_uncomplete in main is: {next_state_uncomplete}")

                # find the new waypoint
                current_waypoint, wp_changed=self.update_waypoint(next_state_uncomplete)
                if wp_changed:
                    # Reset previous waypoint distance when waypoint changes
                    self.prev_distance_to_wp = math.hypot(
                        state_uncomplete[0] - current_waypoint[0],
                        state_uncomplete[1] - current_waypoint[1])
                # complete state (with the waypoint)
                #next_state_reward=np.concatenate([current_waypoint, next_state_uncomplete], axis=0)  # shape (17,)
            
                next_state_rl = make_relative_state(next_state_uncomplete, current_waypoint,prev_action)
                self.get_logger().info(f"states: {next_state_rl}")
                episode_return += reward
                self.get_logger().info(f"return is {episode_return}")
                episode_length += 1
                
                


                # Update state
                state_rl = next_state_rl
                


            if episode>0:
            #if (episode + 1) % 2 == 0:
                self.get_logger().info(
                    f"Episode: {episode + 1:4d} | "
                    f"Score: {(episode_return)} | "

    )

        return 

    def observation_callback(self, msg):
        """Store latest observation — do NOT start RL here."""
        self.last_obs = np.array(msg.data, dtype=np.float32)

        #self.num_states=self.last_obs.shape[0]
        #self.get_logger().info(f"Received observation of shape: {self.last_obs.shape}")
        #self.get_logger().info(f" shape: {self.num_states}")

def main():
    rclpy.init()
    node = TD3AgentNode()
    node.train(1000)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




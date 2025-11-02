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
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
import threading

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

class ReplayBuffer:
    def __init__(self, max_size):
        self.buffer = deque(maxlen=max_size)
        self.max_size = max_size

    def store(self, state, action, reward, next_state, done):
        # Convert everything to float32 when storing
        state = np.array(state, dtype=np.float32)
        action = np.array(action, dtype=np.float32)
        reward = np.float32(reward)  # Convert scalar to float32
        next_state = np.array(next_state, dtype=np.float32)
        done = np.float32(done)  # Convert boolean to float32
        
        experience = (state, action, reward, next_state, done)
        self.buffer.append(experience)

    def sample_batch(self, batch_size):
        if batch_size > len(self.buffer):
            batch_size = len(self.buffer)

        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        states = []
        actions = []
        rewards = []
        next_states = []
        dones = []

        for idx in indices:
            s, a, r, s2, d = self.buffer[idx]
            states.append(s)
            actions.append(a)
            rewards.append(r)
            next_states.append(s2)
            dones.append(d)

        return {
            's': np.array(states, dtype=np.float32),
            'a': np.array(actions, dtype=np.float32),
            'r': np.array(rewards, dtype=np.float32),
            's2': np.array(next_states, dtype=np.float32),
            'd': np.array(dones, dtype=np.float32)
        }
    
    def __len__(self):
        return len(self.buffer)

class Actor(Model):
    def __init__(self, action_dim, action_max, hidden_sizes=(300,)):
        super().__init__()
        self.action_max = tf.constant(action_max, dtype=tf.float32)  # shape (2,)

        self.hidden_layers = [layers.Dense(h, activation='relu') for h in hidden_sizes]
        self.output_layer = layers.Dense(action_dim, activation='tanh')

    def call(self, state):
        x = state
        for lyr in self.hidden_layers:
            x = lyr(x)
        return self.output_layer(x) * self.action_max

class Critic(Model):
    def __init__(self, hidden_sizes=(300,)):
        super().__init__()
        self.hidden_layers = [layers.Dense(h, activation='relu') for h in hidden_sizes]
        self.output_layer = layers.Dense(1)

    def call(self, inputs):
        state, action = inputs
        x = tf.concat([state, action], axis=-1)
        for lyr in self.hidden_layers:
            x = lyr(x)
        return tf.squeeze(self.output_layer(x), axis=1)

# ROS2 TD3 AGENT NODE
class TD3AgentNode(Node):
    def __init__(self, hidden_sizes=(300,), replay_size=int(1e2), mu_lr=1e-3, q_lr=1e-3,
        gamma=0.99, decay=0.995, batch_size=100, action_noise=0.1, target_noise=0.2,
        noise_clip=0.5, policy_delay=2,max_episode_length=400):
        super().__init__("td3_agent")

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

        self.max_v_leader=0.5
        self.max_w_leader=1.0

        self.last_obs = None
        
        #extract environment dimensions
        self.num_states = 19
        self.num_actions = 2  # linear and angular velocity
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
        self.critic1 = Critic(hidden_sizes)
        self.critic2 = Critic(hidden_sizes)
        self.target_actor = Actor(self.num_actions, self.action_max, hidden_sizes)
        self.target_critic1 = Critic(hidden_sizes)
        self.target_critic2 = Critic(hidden_sizes)

        # Build networks (initialize weights)
        dummy_state = tf.zeros([1, self.num_states])
        dummy_action = tf.zeros([1, self.num_actions])
        self.actor(dummy_state)
        self.critic1([dummy_state, dummy_action])
        self.critic2([dummy_state, dummy_action])
        self.target_actor(dummy_state)
        self.target_critic1([dummy_state, dummy_action])
        self.target_critic2([dummy_state, dummy_action])

        # Copy weights to target networks
        self.target_actor.set_weights(self.actor.get_weights())
        self.target_critic1.set_weights(self.critic1.get_weights())
        self.target_critic2.set_weights(self.critic2.get_weights())
        
        # Create optimizers
        self.actor_optimizer = tf.keras.optimizers.Adam(mu_lr)
        self.critic1_optimizer = tf.keras.optimizers.Adam(q_lr)
        self.critic2_optimizer = tf.keras.optimizers.Adam(q_lr)

        # Create replay buffer
        self.replay_buffer = ReplayBuffer(replay_size)
        
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

        # check if the rect obj distance to current waypoint is closer than 0.3 meter then go to the next waypoint
        if distance<=0.3:
            self.current_wp_idx=min(self.current_wp_idx+1,len(self.waypoints)-1 )

        return np.array(self.waypoints[self.current_wp_idx], dtype=np.float32)


    def reward_calculator(self,uncomplete_states,current_wp):

        reward=0

        rect_obj_x, rect_obj_y=uncomplete_states[0],uncomplete_states[1]
        final_goal_x, final_goal_y=uncomplete_states[-2], uncomplete_states[-1]
        closest_obstacle_x, closest_obstacle_y=uncomplete_states[3], uncomplete_states[4]

        distance_to_goal=math.hypot(rect_obj_x-final_goal_x, rect_obj_y-final_goal_y)
        distance_to_obstacle=math.hypot(rect_obj_x-closest_obstacle_x,rect_obj_y-closest_obstacle_y)
        distance_to_waypoint=math.hypot(rect_obj_x-current_wp[0],rect_obj_y-current_wp[1])
                
        #self.get_logger().info(f"next_state_uncomplete in reward calculator is: {uncomplete_states}")

        self.get_logger().info(f"distance to obstacle: {distance_to_obstacle}")

        if distance_to_goal <=0.3:
            reward+=20
        if distance_to_obstacle<=1.2:
            reward-=20
        if distance_to_waypoint<=0.3:
            reward+=1
        # negative reward for every time step
        reward-=0.03
        # progress reward
        reward += -(distance_to_waypoint) * 0.05

        if distance_to_goal<=0.3 or distance_to_obstacle<=1.2:
            Termination=True
        else :
            Termination=False

        return reward, Termination

    # publish the action of rl agent
    def send_action_to_robot(self, action):
        # action = [v, w]
        msg = Twist()
        msg.linear.x = float(action[0])
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
        time.sleep(2)

    def get_action(self, s, noise_scale):
        a = self.actor(tf.convert_to_tensor(s.reshape(1,-1), dtype=tf.float32))
        a = a.numpy()[0]
        a += noise_scale * np.random.randn(self.num_actions)
        return np.clip(a, -self.action_max, self.action_max)
    
    @tf.function
    def update(self, batch):
        states = tf.convert_to_tensor(batch['s'], dtype=tf.float32)
        states_next = tf.convert_to_tensor(batch['s2'], dtype=tf.float32)
        actions = tf.convert_to_tensor(batch['a'], dtype=tf.float32)
        rewards = tf.convert_to_tensor(batch['r'], dtype=tf.float32)
        dones = tf.convert_to_tensor(batch['d'], dtype=tf.float32)
        
        # Add noise to target actions
        noise = tf.random.normal(tf.shape(actions), stddev=self.target_noise)
        noise = tf.clip_by_value(noise, -self.noise_clip, self.noise_clip)
        
        target_actions = self.target_actor(states_next)
        target_actions = tf.clip_by_value(
            target_actions + noise,
            -self.action_max,
            self.action_max)
        
        # Get minimum Q-value between two critics
        target_q1 = self.target_critic1([states_next, target_actions])
        target_q2 = self.target_critic2([states_next, target_actions])
        target_q = tf.minimum(target_q1, target_q2)
        
        # Q targets
        q_target = rewards + self.gamma * (1 - dones) * target_q
        
        # Update first critic
        with tf.GradientTape() as tape:
            q1 = self.critic1([states, actions])
            critic1_loss = tf.reduce_mean((q1 - q_target)**2)
        
        critic1_gradients = tape.gradient(critic1_loss, self.critic1.trainable_variables)
        self.critic1_optimizer.apply_gradients(
            zip(critic1_gradients, self.critic1.trainable_variables)
        )
        
        # Update second critic
        with tf.GradientTape() as tape:
            q2 = self.critic2([states, actions])
            critic2_loss = tf.reduce_mean((q2 - q_target)**2)
        
        critic2_gradients = tape.gradient(critic2_loss, self.critic2.trainable_variables)
        self.critic2_optimizer.apply_gradients(
            zip(critic2_gradients, self.critic2.trainable_variables)
        )
        
        # Delayed policy updates
        if self.total_it % self.policy_delay == 0:
            # Update actor
            with tf.GradientTape() as tape:
                actor_actions = self.actor(states)
                actor_loss = -tf.reduce_mean(self.critic1([states, actor_actions]))
            
            actor_gradients = tape.gradient(actor_loss, self.actor.trainable_variables)
            self.actor_optimizer.apply_gradients(
                zip(actor_gradients, self.actor.trainable_variables)
            )
            
            # Update target networks
            self.update_target_networks()
        else:
            actor_loss = tf.constant(0.0)
        
        return critic1_loss, critic2_loss, actor_loss

    def update_target_networks(self):
        # Update target networks using soft update
        for target, main in zip(self.target_actor.variables, self.actor.variables):
            target.assign(self.decay * target + (1 - self.decay) * main)
        
        for target, main in zip(self.target_critic1.variables, self.critic1.variables):
            target.assign(self.decay * target + (1 - self.decay) * main)
            
        for target, main in zip(self.target_critic2.variables, self.critic2.variables):
            target.assign(self.decay * target + (1 - self.decay) * main)

    def train(self, num_episodes):
        returns = []
        # test_returns = []
        critic1_losses = []
        critic2_losses = []
        actor_losses = []

        self.get_logger().info(f"Using random actions for the initial {self.replay_buffer.max_size} steps...")

        for episode in range(num_episodes):
            
            self.current_wp_idx = 0

            self.reset_environment()
            self.send_action_to_robot(np.array([0,0]))

            time.sleep(8.0)

            state_uncomplete=self.last_obs
            current_waypoint=self.update_waypoint(state_uncomplete)
            state = np.concatenate([current_waypoint, state_uncomplete], axis=0)  # shape (19,)
            episode_return, episode_length=0, 0
            
            #state, episode_return, episode_length = self.env.reset()[0], 0, 0
            done = False

            while not (done or episode_length == self.max_episode_length):
                
                rclpy.spin_once(self)  # **keep callbacks alive**

                
                self.get_logger().info(f"episode_length  is {episode_length}")

                # Use agent's actions only after buffer has enough samples
                if len(self.replay_buffer) >= self.replay_buffer.max_size: #self.batch_size:
                    action = self.get_action(state, self.action_noise)
                else:
                    #action = self.env.action_space.sample()
                    action = np.random.uniform(low=-self.action_max, high=self.action_max)

                time.sleep(0.01)

                # Take action in environment
                self.send_action_to_robot(action)
                # wait till the action implemented on environment
                time.sleep(0.1)
                # read the uncomplete state after implementing action to calculate reward
                next_state_uncomplete=self.last_obs
                # calculate reward 
                reward ,done=self.reward_calculator(next_state_uncomplete,current_waypoint)
                #self.get_logger().info(f"next_state_uncomplete in main is: {next_state_uncomplete}")

                # find the new waypoint
                current_waypoint=self.update_waypoint(next_state_uncomplete)
                # complete state (with the waypoint)
                next_state=np.concatenate([current_waypoint, next_state_uncomplete], axis=0)  # shape (17,)

                #next_state, reward, done, _, _ = self.env.step(action)
                
                episode_return += reward

                self.get_logger().info(f"return is {episode_return}")
                self.get_logger().info(f"done is {done}")


                episode_length += 1
                
                # Store transition
                done_store = False if episode_length == self.max_episode_length else done
                self.replay_buffer.store(state, action, reward, next_state, done_store)
                
                if len(self.replay_buffer) == self.replay_buffer.max_size-1: #self.batch_size:
                    self.get_logger().info("Memory full. Performing agent actions from now on.")

                
                # Update state
                state = next_state
                
                # Update networks if buffer has enough samples
                if len(self.replay_buffer) >= self.batch_size:
                    batch = self.replay_buffer.sample_batch(self.batch_size)
                    critic1_loss, critic2_loss, actor_loss = self.update(batch)
                    critic1_losses.append(critic1_loss.numpy())
                    critic2_losses.append(critic2_loss.numpy())
                    actor_losses.append(actor_loss.numpy())
                    self.total_it += 1
            

                            
            if (episode + 1) % 10 == 0:
                self.get_logger().info(
                    f"Episode: {episode + 1:4d} | "
                    f"Score: {int(episode_return):5d} | "
                    f"Memory: {len(self.replay_buffer):5d} | "
                    f"Actor Loss: {actor_loss.numpy():.2f} | "
                    f"Critic 1 Loss: {critic1_loss.numpy():.2f} | "
                    f"Critic 2 Loss: {critic2_loss.numpy():.2f}"
    )
                
            returns.append(episode_return)
        
        return returns, critic1_losses, critic2_losses, actor_losses

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




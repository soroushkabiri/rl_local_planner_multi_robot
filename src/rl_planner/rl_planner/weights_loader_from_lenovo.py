#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
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

# Import your Actor and Critic classes here
#from rl_planner.networks import Actor, Critic  



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


class Critic(Model):
    def __init__(self, hidden_sizes=(256,128,64,)):
        super().__init__()
        self.hidden_layers = [layers.Dense(h, activation='relu') for h in hidden_sizes]
        self.output_layer = layers.Dense(1)

    def call(self, inputs):
        state, action = inputs
        x = tf.concat([state, action], axis=-1)
        for lyr in self.hidden_layers:
            x = lyr(x)
        return tf.squeeze(self.output_layer(x), axis=1)
        #return self.output_layer(x)   # shape (batch, 1)

class TD3WeightsLoader(Node):
    def __init__(self):
        super().__init__("td3_weights_loader")

        # Dimensions
        self.num_states = 9
        self.num_actions = 2
        self.hidden_sizes = (256, 128, 64)
        self.max_v_leader = 0.15
        self.max_w_leader = 0.3
        self.action_max = np.array([self.max_v_leader, self.max_w_leader], dtype=np.float32)

        # Directories
        self.save_dir = os.path.expanduser("~/rl_planner/rl_local_planner_multi_robot/td3_weights_single")
        self.load_dir = os.path.join(self.save_dir, "episode_final")

        # Create networks
        self.actor = Actor(self.num_actions, self.action_max, self.hidden_sizes)
        self.critic1 = Critic(self.hidden_sizes)
        self.critic2 = Critic(self.hidden_sizes)
        self.target_actor = Actor(self.num_actions, self.action_max, self.hidden_sizes)
        self.target_critic1 = Critic(self.hidden_sizes)
        self.target_critic2 = Critic(self.hidden_sizes)

        # Build networks (required before loading old weights)
        dummy_state = tf.zeros([1, self.num_states])
        dummy_action = tf.zeros([1, self.num_actions])
        self.actor(dummy_state)
        self.critic1([dummy_state, dummy_action])
        self.critic2([dummy_state, dummy_action])
        self.target_actor(dummy_state)
        self.target_critic1([dummy_state, dummy_action])
        self.target_critic2([dummy_state, dummy_action])

        # Try loading weights
        self.load_weights()

    def load_weights(self):
        if os.path.exists(self.load_dir):
            try:
                self.actor.load_weights(os.path.join(self.load_dir, "actor.weights.h5"))
                self.critic1.load_weights(os.path.join(self.load_dir, "critic1.weights.h5"))
                self.critic2.load_weights(os.path.join(self.load_dir, "critic2.weights.h5"))
                self.target_actor.load_weights(os.path.join(self.load_dir, "target_actor.weights.h5"))
                self.target_critic1.load_weights(os.path.join(self.load_dir, "target_critic1.weights.h5"))
                self.target_critic2.load_weights(os.path.join(self.load_dir, "target_critic2.weights.h5"))
                self.get_logger().info("✅ All weights loaded successfully!")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to load some weights: {e}")
                self.get_logger().warn("Weights not compatible. Training must start from scratch.")
        else:
            self.get_logger().info("ℹ️ No previous weights found. Starting from scratch.")

def main(args=None):
    rclpy.init(args=args)
    node = TD3WeightsLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

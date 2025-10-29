import numpy as np
import tensorflow as tf
from tensorflow.keras import Model, layers
import gymnasium as gym
from datetime import datetime
import matplotlib.pyplot as plt
from collections import deque



class Actor(Model):
    def __init__(self, action_dim, action_max, hidden_sizes=(300,)):
        super(Actor, self).__init__()
        self.action_max = action_max
        self.hidden_layers = []
        
        # Create hidden layers
        for h in hidden_sizes:
            self.hidden_layers.append(layers.Dense(h, activation='relu'))
            
        # Output layer
        self.output_layer = layers.Dense(action_dim, activation='tanh')
        
    def call(self, state):
        x = state
        for layer in self.hidden_layers:
            x = layer(x)
        return self.action_max * self.output_layer(x)
    



class Critic(Model):
    def __init__(self, hidden_sizes=(300,)):
        super(Critic, self).__init__()
        self.hidden_layers = []
        
        # Create hidden layers
        for h in hidden_sizes:
            self.hidden_layers.append(layers.Dense(h, activation='relu'))
            
        # Output layer (Q-value)
        self.output_layer = layers.Dense(1)
        
    def call(self, inputs):
        state, action = inputs
        x = tf.concat([state, action], axis=-1)
        for layer in self.hidden_layers:
            x = layer(x)
        return tf.squeeze(self.output_layer(x), axis=1)
    



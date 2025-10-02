#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
# --- IEEE plotting style ---
plt.rcParams.update({
    "font.family": "serif",      # Times/Serif font
    "font.size": 12,              # base font
    "axes.titlesize": 13,
    "axes.labelsize": 13,
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11
})

class BagPlotter(Node):
    def __init__(self):
        super().__init__('bag_plotter')

        # --- Path to your bag file ---
        bag_path = "map_11/map_11_0.db3"
        self.get_logger().info(f"Loading bag: {bag_path}")

        # Open bag database
        con = sqlite3.connect(bag_path)
        cur = con.cursor()

        # --- Get topics ---
        topics = cur.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_map = {name: (id, type) for id, name, type in topics}
        self.get_logger().info(f"Topics in bag: {list(self.topic_map.keys())}")

        # --- Global time zero (earliest message in bag) ---
        cur.execute("SELECT MIN(timestamp) FROM messages")
        t0_ns = cur.fetchone()[0]
        self.global_t0 = t0_ns / 1e9  # ns → s
        self.get_logger().info(f"Global start time: {self.global_t0:.3f} s (epoch)")

        # --- Create one figure with 2 subplots (stacked vertically) ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True)

        # --- Subplot 1: leader cmd_vel vs follower v_hat ---
        leader_topic = "/robot0_0/cmd_vel"
        follower_vhat_topics = [
            "/robot0_1/v_hat",
            "/robot1_0/v_hat",
            "/robot1_1/v_hat",
        ]

        # Leader (Twist)
        df_leader = self.load_twist(cur, leader_topic)
        if not df_leader.empty:
            ax1.plot(df_leader["time"], df_leader["linear_x"], "k--", 
                    label="Leader desired V")

        # Followers (Float32)
        for i, topic in enumerate(follower_vhat_topics, start=1):
            df = self.load_float(cur, topic)
            if not df.empty:
                ax1.plot(df["time"], df["value"], label=f"Follower {i} estimated V")

        ax1.set_ylabel("Velocity [m/s]")
        ax1.set_title("Leader desired V vs Followers estimated V")
        ax1.grid(True)
        ax1.legend()

        # --- Subplot 2: yaw comparison ---
        yaw_leader_topic = "/robot0_0/yaw_deg"  # already in degrees
        yaw_hat_topics = [
            "/robot0_1/yaw_hat",
            "/robot1_0/yaw_hat",
            "/robot1_1/yaw_hat",
        ]

        df_leader = self.load_float(cur, yaw_leader_topic, radians=False, wrap360=True)
        if not df_leader.empty:
            ax2.plot(df_leader["time"], df_leader["value"], "k--", 
                    label=r"Leader desired $\theta$")

        for i, topic in enumerate(yaw_hat_topics, start=1):
            df = self.load_float(cur, topic, radians=True, wrap360=True)  # rad→deg then wrap
            if not df.empty:
                ax2.plot(df["time"], df["value"], label=rf"Follower {i} estimated $\theta$")

        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel(r"Yaw [deg] (0–360)")
        ax2.set_title(r"Leader desired $\theta$ vs Followers estimated $\theta$")
        ax2.grid(True)
        ax2.legend()

        plt.tight_layout()
        plt.show()


        # Shutdown after plotting
        rclpy.shutdown()

    def load_twist(self, cur, topic_name):
        """Load geometry_msgs/Twist messages and align to global t0"""
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "linear_x"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=?",
            (topic_id,)
        ).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Twist)
            time_s = (t / 1e9) - self.global_t0
            data.append([time_s, msg.linear.x])
        return pd.DataFrame(data, columns=["time", "linear_x"])

    def load_float(self, cur, topic_name, radians=False, wrap360=False):
        """Load std_msgs/Float32 messages and align to global t0"""
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "value"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=?",
            (topic_id,)
        ).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Float32)
            val = msg.data
            if radians:
                val = math.degrees(val)
            if wrap360:
                val = (val + 360) % 360
            time_s = (t / 1e9) - self.global_t0
            data.append([time_s, val])
        return pd.DataFrame(data, columns=["time", "value"])


def main(args=None):
    rclpy.init(args=args)
    BagPlotter()
    rclpy.spin(rclpy.get_global_executor())


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2025 - PREDICTIVE OBSTACLE AVOIDANCE ROS WRAPPER
=========================================================

Bu ROS wrapper, Predictive Obstacle Avoidance System'i ROS ekosistemi ile entegre eder.
Trajectory prediction ve dynamic path planning sağlar.

Features:
- Trajectory prediction (3-5 second lookahead)
- Dynamic obstacle avoidance
- Real-time collision risk assessment
- Path planning optimization
- Multi-sensor obstacle detection

Author: TEKNOFEST Takımı
Date: 30 Ağustos 2025
"""

import rospy
import numpy as np
import threading
from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

# Predictive Obstacle Avoidance System import
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from predictive_obstacle_avoidance import PredictiveObstacleAvoidance

class ObstacleAvoidanceROSNode:
    """Predictive Obstacle Avoidance System için ROS Wrapper"""
    
    def __init__(self):
        """ROS node ve Obstacle Avoidance System'i başlatır"""
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        rospy.loginfo("🎯 Obstacle Avoidance ROS Node başlatılıyor...")
        
        # Obstacle Avoidance System oluştur
        self.avoidance_system = PredictiveObstacleAvoidance()
        
        # Data storage
        self.robot_data = {
            'position': np.array([0.0, 0.0, 0.0]),
            'velocity': np.array([0.0, 0.0, 0.0]),
            'obstacles': [],
            'goal_position': np.array([0.0, 0.0, 0.0]),
            'current_path': []
        }
        
        # Avoidance status
        self.avoidance_active = False
        self.collision_risk = 0.0
        self.predicted_trajectory = []
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup ROS Publishers & Subscribers
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()
        
        rospy.loginfo("✅ Obstacle Avoidance ROS Node hazır!")
    
    def setup_publishers(self):
        """ROS publisher'larını kurar"""
        self.avoidance_status_pub = rospy.Publisher(
            '/obstacle_avoidance/status', String, queue_size=10
        )
        self.collision_risk_pub = rospy.Publisher(
            '/obstacle_avoidance/collision_risk', Float32, queue_size=10
        )
        self.avoidance_cmd_vel_pub = rospy.Publisher(
            '/obstacle_avoidance/cmd_vel', Twist, queue_size=1
        )
        self.predicted_path_pub = rospy.Publisher(
            '/obstacle_avoidance/predicted_path', Path, queue_size=10
        )
        self.obstacles_marker_pub = rospy.Publisher(
            '/obstacle_avoidance/obstacles_marker', MarkerArray, queue_size=10
        )
        self.avoidance_active_pub = rospy.Publisher(
            '/obstacle_avoidance/active', Bool, queue_size=10
        )
        
        rospy.loginfo("📡 Obstacle Avoidance publishers kuruldu")
    
    def setup_subscribers(self):
        """ROS subscriber'ları kurar"""
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1
        )
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1
        )
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1
        )
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, self.path_callback, queue_size=1
        )
        
        rospy.loginfo("📡 Obstacle Avoidance subscribers kuruldu")
    
    def setup_timers(self):
        """Timer'ları kurar"""
        self.avoidance_timer = rospy.Timer(
            rospy.Duration(0.05), self.avoidance_callback  # 20Hz
        )
        self.prediction_timer = rospy.Timer(
            rospy.Duration(0.1), self.prediction_callback  # 10Hz
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(0.2), self.status_callback  # 5Hz
        )
        
        rospy.loginfo("⏰ Obstacle Avoidance timers kuruldu")
    
    def lidar_callback(self, msg):
        """LiDAR verisini işler ve obstacles tespit eder"""
        with self.data_lock:
            ranges = np.array(msg.ranges)
            ranges[np.isinf(ranges)] = msg.range_max
            ranges[np.isnan(ranges)] = 0.0
            
            # Obstacle detection
            obstacles = []
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            
            for i, range_val in enumerate(ranges):
                if 0.1 < range_val < 2.0:  # Obstacle detection range
                    angle = angle_min + i * angle_increment
                    
                    # Cartesian coordinates
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    
                    obstacles.append([x, y, 0.0])
            
            self.robot_data['obstacles'] = obstacles
    
    def odom_callback(self, msg):
        """Odometry verisini işler"""
        with self.data_lock:
            self.robot_data['position'] = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.robot_data['velocity'] = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z
            ])
    
    def goal_callback(self, msg):
        """Goal callback"""
        with self.data_lock:
            self.robot_data['goal_position'] = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
    
    def cmd_vel_callback(self, msg):
        """Command velocity callback"""
        # Current cmd_vel'i obstacle avoidance için kullan
        pass
    
    def path_callback(self, msg):
        """Path callback"""
        with self.data_lock:
            path_points = []
            for pose in msg.poses:
                point = [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ]
                path_points.append(point)
            self.robot_data['current_path'] = path_points
    
    def avoidance_callback(self, event):
        """Ana obstacle avoidance callback (20Hz)"""
        try:
            with self.data_lock:
                robot_data_copy = self.robot_data.copy()
            
            if len(robot_data_copy['obstacles']) > 0:
                self.perform_obstacle_avoidance(robot_data_copy)
            else:
                self.avoidance_active = False
            
        except Exception as e:
            rospy.logerr(f"❌ Obstacle avoidance hatası: {e}")
    
    def prediction_callback(self, event):
        """Trajectory prediction callback (10Hz)"""
        try:
            with self.data_lock:
                robot_data_copy = self.robot_data.copy()
            
            # Trajectory prediction yap
            self.predict_trajectory(robot_data_copy)
            
            # Collision risk hesapla
            self.calculate_collision_risk(robot_data_copy)
            
        except Exception as e:
            rospy.logerr(f"❌ Trajectory prediction hatası: {e}")
    
    def status_callback(self, event):
        """Status callback (5Hz)"""
        try:
            self.publish_status_messages()
            self.publish_visualization()
            
        except Exception as e:
            rospy.logerr(f"❌ Status callback hatası: {e}")
    
    def perform_obstacle_avoidance(self, robot_data):
        """Obstacle avoidance yapar"""
        try:
            # Avoidance system'e veri gönder
            self.avoidance_system.update_robot_state(
                robot_data['position'],
                robot_data['velocity']
            )
            
            self.avoidance_system.update_obstacles(
                np.array(robot_data['obstacles']) if robot_data['obstacles'] else np.array([])
            )
            
            self.avoidance_system.update_goal(
                robot_data['goal_position']
            )
            
            # Avoidance command hesapla
            avoidance_result = self.avoidance_system.calculate_avoidance_command()
            
            if avoidance_result and avoidance_result.get('avoidance_needed', False):
                self.avoidance_active = True
                
                # Avoidance cmd_vel yayınla
                cmd_vel = avoidance_result.get('cmd_vel', [0.0, 0.0, 0.0])
                self.publish_avoidance_cmd_vel(cmd_vel)
            else:
                self.avoidance_active = False
                
        except Exception as e:
            rospy.logerr(f"❌ Obstacle avoidance performance hatası: {e}")
    
    def predict_trajectory(self, robot_data):
        """Robot trajectory'sini predict eder"""
        try:
            # 3-5 saniye lookahead
            prediction_time = 3.0
            dt = 0.1
            steps = int(prediction_time / dt)
            
            # Current state
            pos = robot_data['position'].copy()
            vel = robot_data['velocity'].copy()
            
            trajectory = [pos.copy()]
            
            # Trajectory prediction
            for i in range(steps):
                # Simple linear prediction (gerçek implementation daha complex olabilir)
                pos[0] += vel[0] * dt
                pos[1] += vel[1] * dt
                pos[2] += vel[2] * dt  # Angular velocity integration
                
                trajectory.append(pos.copy())
            
            self.predicted_trajectory = trajectory
            
        except Exception as e:
            rospy.logerr(f"❌ Trajectory prediction hatası: {e}")
    
    def calculate_collision_risk(self, robot_data):
        """Collision risk hesaplar"""
        try:
            if not self.predicted_trajectory or not robot_data['obstacles']:
                self.collision_risk = 0.0
                return
            
            min_distance = float('inf')
            
            # Her trajectory point için en yakın obstacle mesafesi
            for trajectory_point in self.predicted_trajectory:
                for obstacle in robot_data['obstacles']:
                    distance = np.linalg.norm(
                        np.array(trajectory_point[:2]) - np.array(obstacle[:2])
                    )
                    min_distance = min(min_distance, distance)
            
            # Risk calculation (0.0 - 1.0)
            if min_distance < 0.2:  # Critical distance
                self.collision_risk = 1.0
            elif min_distance < 0.5:  # Warning distance
                self.collision_risk = 1.0 - (min_distance - 0.2) / 0.3
            else:
                self.collision_risk = 0.0
            
        except Exception as e:
            rospy.logerr(f"❌ Collision risk calculation hatası: {e}")
            self.collision_risk = 0.0
    
    def publish_avoidance_cmd_vel(self, cmd_vel):
        """Avoidance cmd_vel yayınlar"""
        twist_msg = Twist()
        twist_msg.linear.x = cmd_vel[0]
        twist_msg.linear.y = cmd_vel[1]
        twist_msg.angular.z = cmd_vel[2]
        
        self.avoidance_cmd_vel_pub.publish(twist_msg)
    
    def publish_status_messages(self):
        """Status mesajlarını yayınlar"""
        # Avoidance status
        status_msg = String()
        obstacle_count = len(self.robot_data.get('obstacles', []))
        status_msg.data = f"Active: {self.avoidance_active}, " \
                         f"Risk: {self.collision_risk:.3f}, " \
                         f"Obstacles: {obstacle_count}"
        self.avoidance_status_pub.publish(status_msg)
        
        # Collision risk
        risk_msg = Float32()
        risk_msg.data = float(self.collision_risk)
        self.collision_risk_pub.publish(risk_msg)
        
        # Avoidance active
        active_msg = Bool()
        active_msg.data = self.avoidance_active
        self.avoidance_active_pub.publish(active_msg)
    
    def publish_visualization(self):
        """Visualization markers yayınlar"""
        try:
            # Predicted trajectory
            if self.predicted_trajectory:
                path_msg = Path()
                path_msg.header.frame_id = "base_link"
                path_msg.header.stamp = rospy.Time.now()
                
                for point in self.predicted_trajectory:
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = point[2]
                    path_msg.poses.append(pose)
                
                self.predicted_path_pub.publish(path_msg)
            
            # Obstacle markers
            with self.data_lock:
                obstacles = self.robot_data.get('obstacles', [])
            
            if obstacles:
                marker_array = MarkerArray()
                
                for i, obstacle in enumerate(obstacles):
                    marker = Marker()
                    marker.header.frame_id = "base_link"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacles"
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = obstacle[0]
                    marker.pose.position.y = obstacle[1]
                    marker.pose.position.z = obstacle[2]
                    
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    
                    # Risk-based color
                    if self.collision_risk > 0.7:
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    elif self.collision_risk > 0.3:
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    
                    marker.color.a = 0.8
                    marker.lifetime = rospy.Duration(0.5)
                    
                    marker_array.markers.append(marker)
                
                self.obstacles_marker_pub.publish(marker_array)
                
        except Exception as e:
            rospy.logerr(f"❌ Visualization hatası: {e}")
    
    def shutdown_handler(self):
        """Node kapanma işlemleri"""
        rospy.loginfo("🎯 Obstacle Avoidance Node kapatılıyor...")
        self.avoidance_system.stop_avoidance()
        rospy.loginfo("✅ Obstacle Avoidance Node kapatıldı")

def main():
    """Ana program"""
    try:
        avoidance_node = ObstacleAvoidanceROSNode()
        rospy.on_shutdown(avoidance_node.shutdown_handler)
        
        rospy.loginfo("🚀 Obstacle Avoidance ROS Node çalışıyor...")
        rospy.loginfo("📊 Topics: /scan, /odom → /obstacle_avoidance/*")
        rospy.loginfo("🎯 Predictive Avoidance ACTIVE!")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🎯 Obstacle Avoidance Node durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Obstacle Avoidance Node hatası: {e}")

if __name__ == '__main__':
    main()

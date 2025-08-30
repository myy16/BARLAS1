#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2025 - SAFETY MONITOR ROS WRAPPER
===========================================

Bu ROS wrapper, Safety Monitor System'i ROS ekosistemi ile entegre eder.
Tüm güvenlik kurallarını real-time izler ve acil durumda sistemi durdurur.

Features:
- 10 güvenlik kuralı real-time monitoring
- 5-level safety system (Safe → Emergency)
- Emergency stop capability
- ROS topic integration
- Safety status broadcasting

Author: TEKNOFEST Takımı
Date: 30 Ağustos 2025
"""

import rospy
import numpy as np
import threading
import time
from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

# Safety Monitor System import
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from safety_monitor_system import SafetyMonitorSystem

class SafetyMonitorROSNode:
    """
    Safety Monitor System için ROS Wrapper
    
    Bu class, Safety Monitor System'i ROS topics ile entegre eder.
    Sensör verilerini alır, güvenlik analizi yapar ve sonuçları yayınlar.
    """
    
    def __init__(self):
        """ROS node ve Safety Monitor System'i başlatır"""
        # ROS Node başlat
        rospy.init_node('safety_monitor_node', anonymous=True)
        rospy.loginfo("🔥 Safety Monitor ROS Node başlatılıyor...")
        
        # Safety Monitor System oluştur
        self.safety_system = SafetyMonitorSystem()
        
        # Data storage
        self.current_data = {
            'lidar_data': None,
            'imu_data': None,
            'odometry_data': None,
            'cmd_vel': None,
            'battery_voltage': None,
            'temperature': None
        }
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Emergency stop flag
        self.emergency_stop_active = False
        
        # Setup ROS Publishers
        self.setup_publishers()
        
        # Setup ROS Subscribers  
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        rospy.loginfo("✅ Safety Monitor ROS Node hazır!")
    
    def setup_publishers(self):
        """ROS publisher'larını kurar"""
        # Safety status publisher
        self.safety_status_pub = rospy.Publisher(
            '/safety_monitor/status', String, queue_size=10
        )
        
        # Safety level publisher
        self.safety_level_pub = rospy.Publisher(
            '/safety_monitor/level', Int32, queue_size=10
        )
        
        # Emergency stop publisher
        self.emergency_stop_pub = rospy.Publisher(
            '/safety_monitor/emergency_stop', Bool, queue_size=10
        )
        
        # Safety score publisher
        self.safety_score_pub = rospy.Publisher(
            '/safety_monitor/score', Float32, queue_size=10
        )
        
        # Violation count publisher
        self.violation_pub = rospy.Publisher(
            '/safety_monitor/violations', Int32, queue_size=10
        )
        
        # Emergency cmd_vel override
        self.cmd_vel_override_pub = rospy.Publisher(
            '/cmd_vel_safe', Twist, queue_size=1
        )
        
        rospy.loginfo("📡 Safety Monitor publishers kuruldu")
    
    def setup_subscribers(self):
        """ROS subscriber'ları kurar"""
        # LiDAR data subscriber
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback, queue_size=1
        )
        
        # IMU data subscriber
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, self.imu_callback, queue_size=1
        )
        
        # Odometry subscriber
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1
        )
        
        # Command velocity subscriber
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1
        )
        
        # Battery voltage subscriber
        self.battery_sub = rospy.Subscriber(
            '/battery_voltage', Float32, self.battery_callback, queue_size=1
        )
        
        # Temperature subscriber
        self.temp_sub = rospy.Subscriber(
            '/temperature', Float32, self.temperature_callback, queue_size=1
        )
        
        rospy.loginfo("📡 Safety Monitor subscribers kuruldu")
    
    def setup_timers(self):
        """Timer'ları kurar"""
        # Safety monitoring timer (20Hz - kritik sistem)
        self.monitor_timer = rospy.Timer(
            rospy.Duration(0.05), self.monitor_callback
        )
        
        # Status publishing timer (5Hz)
        self.status_timer = rospy.Timer(
            rospy.Duration(0.2), self.status_callback
        )
        
        rospy.loginfo("⏰ Safety Monitor timers kuruldu")
    
    def lidar_callback(self, msg):
        """LiDAR verisini işler"""
        with self.data_lock:
            # LaserScan'i numpy array'e çevir
            ranges = np.array(msg.ranges)
            # Inf değerlerini max range ile değiştir
            ranges[np.isinf(ranges)] = msg.range_max
            # NaN değerlerini 0 ile değiştir
            ranges[np.isnan(ranges)] = 0.0
            
            self.current_data['lidar_data'] = {
                'ranges': ranges,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def imu_callback(self, msg):
        """IMU verisini işler"""
        with self.data_lock:
            self.current_data['imu_data'] = {
                'orientation': [
                    msg.orientation.x, msg.orientation.y,
                    msg.orientation.z, msg.orientation.w
                ],
                'angular_velocity': [
                    msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z
                ],
                'linear_acceleration': [
                    msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ],
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def odom_callback(self, msg):
        """Odometry verisini işler"""
        with self.data_lock:
            self.current_data['odometry_data'] = {
                'position': [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ],
                'orientation': [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ],
                'linear_velocity': [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ],
                'angular_velocity': [
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z
                ],
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def cmd_vel_callback(self, msg):
        """Command velocity verisini işler"""
        with self.data_lock:
            self.current_data['cmd_vel'] = {
                'linear': [msg.linear.x, msg.linear.y, msg.linear.z],
                'angular': [msg.angular.x, msg.angular.y, msg.angular.z],
                'timestamp': rospy.Time.now().to_sec()
            }
    
    def battery_callback(self, msg):
        """Battery voltage verisini işler"""
        with self.data_lock:
            self.current_data['battery_voltage'] = {
                'voltage': msg.data,
                'timestamp': rospy.Time.now().to_sec()
            }
    
    def temperature_callback(self, msg):
        """Temperature verisini işler"""
        with self.data_lock:
            self.current_data['temperature'] = {
                'temperature': msg.data,
                'timestamp': rospy.Time.now().to_sec()
            }
    
    def monitor_callback(self, event):
        """Ana safety monitoring callback (20Hz)"""
        try:
            with self.data_lock:
                # Verileri kopyala (thread safety için)
                data_copy = self.current_data.copy()
            
            # Safety monitoring yap
            if self.has_required_data(data_copy):
                self.perform_safety_check(data_copy)
            
        except Exception as e:
            rospy.logerr(f"❌ Safety monitoring hatası: {e}")
    
    def has_required_data(self, data):
        """Minimum gerekli verilerin olup olmadığını kontrol eder"""
        required_sensors = ['lidar_data', 'imu_data']
        return all(data[sensor] is not None for sensor in required_sensors)
    
    def perform_safety_check(self, data):
        """Safety check yapar ve sonuçları işler"""
        try:
            # Safety system'e veri gönder
            if data['lidar_data']:
                self.safety_system.update_lidar_data(
                    data['lidar_data']['ranges'],
                    data['lidar_data']['timestamp']
                )
            
            if data['imu_data']:
                self.safety_system.update_imu_data(
                    np.array(data['imu_data']['linear_acceleration']),
                    np.array(data['imu_data']['angular_velocity']),
                    data['imu_data']['timestamp']
                )
            
            if data['odometry_data']:
                self.safety_system.update_odometry_data(
                    np.array(data['odometry_data']['linear_velocity']),
                    np.array(data['odometry_data']['angular_velocity']),
                    data['odometry_data']['timestamp']
                )
            
            if data['cmd_vel']:
                self.safety_system.update_control_data(
                    np.array(data['cmd_vel']['linear']),
                    np.array(data['cmd_vel']['angular']),
                    data['cmd_vel']['timestamp']
                )
            
            if data['battery_voltage']:
                self.safety_system.update_battery_voltage(
                    data['battery_voltage']['voltage'],
                    data['battery_voltage']['timestamp']
                )
            
            if data['temperature']:
                self.safety_system.update_temperature(
                    data['temperature']['temperature'],
                    data['temperature']['timestamp']
                )
            
            # Safety check yap
            safety_result = self.safety_system.check_safety()
            
            # Emergency durumu kontrol et
            self.handle_safety_result(safety_result, data)
            
        except Exception as e:
            rospy.logerr(f"❌ Safety check hatası: {e}")
    
    def handle_safety_result(self, safety_result, data):
        """Safety check sonucunu işler"""
        current_level = safety_result.get('safety_level', 0)
        violations = safety_result.get('active_violations', [])
        
        # Emergency stop kontrolü (Level 4 = Emergency)
        if current_level >= 4:
            if not self.emergency_stop_active:
                self.activate_emergency_stop()
        else:
            if self.emergency_stop_active:
                self.deactivate_emergency_stop()
        
        # Cmd_vel override (güvenli hızla sınırla)
        if data['cmd_vel'] and current_level >= 2:  # Caution level
            self.publish_safe_cmd_vel(data['cmd_vel'], current_level)
    
    def activate_emergency_stop(self):
        """Emergency stop'u aktif eder"""
        self.emergency_stop_active = True
        
        # Emergency stop mesajı yayınla
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
        
        # Durma komutu gönder
        stop_cmd = Twist()  # Tüm değerler 0
        self.cmd_vel_override_pub.publish(stop_cmd)
        
        rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED! 🚨")
    
    def deactivate_emergency_stop(self):
        """Emergency stop'u deaktif eder"""
        self.emergency_stop_active = False
        
        # Emergency stop mesajı yayınla
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)
        
        rospy.loginfo("✅ Emergency stop deactivated")
    
    def publish_safe_cmd_vel(self, cmd_vel, safety_level):
        """Güvenlik seviyesine göre cmd_vel'i sınırlar"""
        safe_cmd = Twist()
        
        # Güvenlik seviyesine göre hız sınırları
        speed_limits = {
            0: 1.0,    # Safe - full speed
            1: 0.8,    # Caution - 80% speed
            2: 0.5,    # Warning - 50% speed
            3: 0.2,    # Critical - 20% speed
            4: 0.0     # Emergency - stop
        }
        
        limit = speed_limits.get(safety_level, 0.0)
        
        # Linear velocity sınırla
        safe_cmd.linear.x = max(min(cmd_vel['linear'][0], limit), -limit)
        safe_cmd.linear.y = max(min(cmd_vel['linear'][1], limit), -limit)
        
        # Angular velocity sınırla (yarısı)
        angular_limit = limit * 0.5
        safe_cmd.angular.z = max(min(cmd_vel['angular'][2], angular_limit), -angular_limit)
        
        self.cmd_vel_override_pub.publish(safe_cmd)
    
    def status_callback(self, event):
        """Status yayınlama callback (5Hz)"""
        try:
            # Safety system durumunu al
            safety_status = self.safety_system.get_safety_status()
            
            # Status mesajlarını yayınla
            self.publish_status_messages(safety_status)
            
        except Exception as e:
            rospy.logerr(f"❌ Status callback hatası: {e}")
    
    def publish_status_messages(self, status):
        """Status mesajlarını ROS topic'lere yayınlar"""
        # Safety status string
        status_msg = String()
        status_msg.data = f"Level: {status.get('safety_level', 0)}, " \
                         f"Score: {status.get('safety_score', 0.0):.3f}, " \
                         f"Violations: {len(status.get('active_violations', []))}"
        self.safety_status_pub.publish(status_msg)
        
        # Safety level
        level_msg = Int32()
        level_msg.data = status.get('safety_level', 0)
        self.safety_level_pub.publish(level_msg)
        
        # Safety score
        score_msg = Float32()
        score_msg.data = float(status.get('safety_score', 0.0))
        self.safety_score_pub.publish(score_msg)
        
        # Violation count
        violation_msg = Int32()
        violation_msg.data = len(status.get('active_violations', []))
        self.violation_pub.publish(violation_msg)
    
    def shutdown_handler(self):
        """Node kapanma işlemleri"""
        rospy.loginfo("🔥 Safety Monitor Node kapatılıyor...")
        
        # Emergency stop deaktif et
        if self.emergency_stop_active:
            self.deactivate_emergency_stop()
        
        # Safety system'i kapat
        self.safety_system.stop_monitoring()
        
        rospy.loginfo("✅ Safety Monitor Node kapatıldı")

def main():
    """Ana program"""
    try:
        # Safety Monitor ROS Node başlat
        safety_node = SafetyMonitorROSNode()
        
        # Shutdown handler kaydet
        rospy.on_shutdown(safety_node.shutdown_handler)
        
        rospy.loginfo("🚀 Safety Monitor ROS Node çalışıyor...")
        rospy.loginfo("📊 Topics:")
        rospy.loginfo("  - Input: /scan, /imu/data, /odom, /cmd_vel")
        rospy.loginfo("  - Output: /safety_monitor/status, /cmd_vel_safe")
        rospy.loginfo("🔥 Emergency stop capability ACTIVE!")
        
        # ROS spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🔥 Safety Monitor Node durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Safety Monitor Node hatası: {e}")

if __name__ == '__main__':
    main()

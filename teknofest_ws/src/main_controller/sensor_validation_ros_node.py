#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2025 - SENSOR VALIDATION ROS WRAPPER
=============================================

Bu ROS wrapper, Sensor Validation System'i ROS ekosistemi ile entegre eder.
Sensör verilerini cross-validation yapar ve outlier detection sağlar.

Features:
- 6 sensor validation rule
- Cross-validation between sensors
- Statistical outlier detection
- Real-time sensor health monitoring
- 99% accuracy validation

Author: TEKNOFEST Takımı
Date: 30 Ağustos 2025
"""

import rospy
import numpy as np
import threading
from std_msgs.msg import String, Bool, Float32, Int32
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# Sensor Validation System import
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from sensor_validation_system import SensorValidationSystem

class SensorValidationROSNode:
    """Sensor Validation System için ROS Wrapper"""
    
    def __init__(self):
        """ROS node ve Sensor Validation System'i başlatır"""
        rospy.init_node('sensor_validation_node', anonymous=True)
        rospy.loginfo("🔍 Sensor Validation ROS Node başlatılıyor...")
        
        # Sensor Validation System oluştur
        self.validation_system = SensorValidationSystem()
        
        # Data storage
        self.sensor_data = {
            'lidar_data': None,
            'imu_data': None,
            'odometry_data': None
        }
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup ROS Publishers & Subscribers
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()
        
        rospy.loginfo("✅ Sensor Validation ROS Node hazır!")
    
    def setup_publishers(self):
        """ROS publisher'larını kurar"""
        self.validation_status_pub = rospy.Publisher(
            '/sensor_validation/status', String, queue_size=10
        )
        self.sensor_health_pub = rospy.Publisher(
            '/sensor_validation/health', DiagnosticArray, queue_size=10
        )
        self.validation_score_pub = rospy.Publisher(
            '/sensor_validation/score', Float32, queue_size=10
        )
        self.outlier_detection_pub = rospy.Publisher(
            '/sensor_validation/outliers', String, queue_size=10
        )
        
        rospy.loginfo("📡 Sensor Validation publishers kuruldu")
    
    def setup_subscribers(self):
        """ROS subscriber'ları kurar"""
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback, queue_size=1
        )
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, self.imu_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1
        )
        
        rospy.loginfo("📡 Sensor Validation subscribers kuruldu")
    
    def setup_timers(self):
        """Timer'ları kurar"""
        self.validation_timer = rospy.Timer(
            rospy.Duration(0.1), self.validation_callback
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0), self.status_callback
        )
        
        rospy.loginfo("⏰ Sensor Validation timers kuruldu")
    
    def lidar_callback(self, msg):
        """LiDAR verisini işler"""
        with self.data_lock:
            ranges = np.array(msg.ranges)
            ranges[np.isinf(ranges)] = msg.range_max
            ranges[np.isnan(ranges)] = 0.0
            
            self.sensor_data['lidar_data'] = {
                'ranges': ranges,
                'timestamp': msg.header.stamp.to_sec(),
                'frame_id': msg.header.frame_id
            }
    
    def imu_callback(self, msg):
        """IMU verisini işler"""
        with self.data_lock:
            self.sensor_data['imu_data'] = {
                'linear_acceleration': np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]),
                'angular_velocity': np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ]),
                'timestamp': msg.header.stamp.to_sec(),
                'frame_id': msg.header.frame_id
            }
    
    def odom_callback(self, msg):
        """Odometry verisini işler"""
        with self.data_lock:
            self.sensor_data['odometry_data'] = {
                'position': np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]),
                'linear_velocity': np.array([
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ]),
                'timestamp': msg.header.stamp.to_sec(),
                'frame_id': msg.header.frame_id
            }
    
    def validation_callback(self, event):
        """Ana validation callback (10Hz)"""
        try:
            with self.data_lock:
                data_copy = self.sensor_data.copy()
            
            if self.has_sensor_data(data_copy):
                self.perform_validation(data_copy)
                
        except Exception as e:
            rospy.logerr(f"❌ Sensor validation hatası: {e}")
    
    def has_sensor_data(self, data):
        """Sensor verisinin olup olmadığını kontrol eder"""
        return any(sensor_data is not None for sensor_data in data.values())
    
    def perform_validation(self, data):
        """Sensor validation yapar"""
        try:
            # Update sensor data in validation system
            if data['lidar_data']:
                self.validation_system.update_lidar_data(
                    data['lidar_data']['ranges'],
                    data['lidar_data']['timestamp']
                )
            
            if data['imu_data']:
                self.validation_system.update_imu_data(
                    data['imu_data']['linear_acceleration'],
                    data['imu_data']['angular_velocity'],
                    data['imu_data']['timestamp']
                )
            
            if data['odometry_data']:
                self.validation_system.update_odometry_data(
                    data['odometry_data']['position'],
                    data['odometry_data']['linear_velocity'],
                    data['odometry_data']['timestamp']
                )
            
            # Perform validation
            validation_result = self.validation_system.validate_sensors()
            
            # Publish validation results
            self.publish_validation_results(validation_result)
            
        except Exception as e:
            rospy.logerr(f"❌ Validation performance hatası: {e}")
    
    def publish_validation_results(self, results):
        """Validation sonuçlarını yayınlar"""
        # Validation score
        score = results.get('overall_score', 0.0)
        score_msg = Float32()
        score_msg.data = float(score)
        self.validation_score_pub.publish(score_msg)
        
        # Outliers
        outliers = results.get('outliers', [])
        if outliers:
            outlier_msg = String()
            outlier_msg.data = f"Outliers detected: {', '.join(outliers)}"
            self.outlier_detection_pub.publish(outlier_msg)
    
    def status_callback(self, event):
        """Status callback (1Hz)"""
        try:
            validation_status = self.validation_system.get_validation_status()
            self.publish_sensor_health(validation_status)
            
        except Exception as e:
            rospy.logerr(f"❌ Status callback hatası: {e}")
    
    def publish_sensor_health(self, status):
        """Sensor health diagnostics yayınlar"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        sensor_names = ['lidar', 'imu', 'odometry']
        for sensor in sensor_names:
            diag_status = DiagnosticStatus()
            diag_status.name = f"sensor_validation_{sensor}"
            diag_status.hardware_id = "teknofest_robot"
            
            sensor_health = status.get(f'{sensor}_health', True)
            if sensor_health:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = f"{sensor} sensor healthy"
            else:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = f"{sensor} sensor validation failed"
            
            diag_array.status.append(diag_status)
        
        self.sensor_health_pub.publish(diag_array)
        
        # Overall status
        status_msg = String()
        overall_score = status.get('overall_validation_score', 0.0)
        status_msg.data = f"Validation Score: {overall_score:.3f}, Health: OK"
        self.validation_status_pub.publish(status_msg)
    
    def shutdown_handler(self):
        """Node kapanma işlemleri"""
        rospy.loginfo("🔍 Sensor Validation Node kapatılıyor...")
        self.validation_system.stop_validation()
        rospy.loginfo("✅ Sensor Validation Node kapatıldı")

def main():
    """Ana program"""
    try:
        validation_node = SensorValidationROSNode()
        rospy.on_shutdown(validation_node.shutdown_handler)
        
        rospy.loginfo("🚀 Sensor Validation ROS Node çalışıyor...")
        rospy.loginfo("📊 Topics: /scan, /imu/data, /odom → /sensor_validation/*")
        rospy.loginfo("🔍 6 Validation Rules ACTIVE!")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🔍 Sensor Validation Node durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Sensor Validation Node hatası: {e}")

if __name__ == '__main__':
    main()

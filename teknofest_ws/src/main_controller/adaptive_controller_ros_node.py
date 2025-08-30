#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2025 - ADAPTIVE CONTROLLER ROS WRAPPER
===============================================

Bu ROS wrapper, Adaptive Stage Controller System'i ROS ekosistemi ile entegre eder.
Farklı aşamalarda (exploration, mapping, navigation, competition) optimum kontrol sağlar.

Features:
- 4 adaptive control stage (Exploration, Mapping, Navigation, Competition)
- Dynamic parameter adjustment
- Performance optimization
- Stage transition automation
- Real-time control adaptation

Author: TEKNOFEST Takımı
Date: 30 Ağustos 2025
"""

import rospy
import numpy as np
import threading
import time
from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatusArray

# Adaptive Stage Controller import
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from adaptive_stage_controller import AdaptiveStageController

class AdaptiveControllerROSNode:
    """
    Adaptive Stage Controller System için ROS Wrapper
    
    Bu class, Adaptive Controller'ı ROS topics ile entegre eder.
    Farklı aşamalarda optimum kontrol parametreleri sağlar.
    """
    
    def __init__(self):
        """ROS node ve Adaptive Controller System'i başlatır"""
        # ROS Node başlat
        rospy.init_node('adaptive_controller_node', anonymous=True)
        rospy.loginfo("🎮 Adaptive Controller ROS Node başlatılıyor...")
        
        # Adaptive Stage Controller oluştur
        self.adaptive_controller = AdaptiveStageController()
        
        # Current stage tracking
        self.current_stage = "exploration"  # Default stage
        self.stage_start_time = time.time()
        self.auto_stage_transition = True
        
        # Performance tracking
        self.performance_metrics = {
            'exploration': {'efficiency': 0.0, 'coverage': 0.0, 'time': 0.0},
            'mapping': {'accuracy': 0.0, 'completeness': 0.0, 'time': 0.0},
            'navigation': {'success_rate': 0.0, 'path_length': 0.0, 'time': 0.0},
            'competition': {'task_completion': 0.0, 'efficiency': 0.0, 'time': 0.0}
        }
        
        # Control parameters
        self.current_params = {
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'lookahead_distance': 1.0,
            'obstacle_threshold': 0.3,
            'path_tolerance': 0.2
        }
        
        # Data storage
        self.robot_data = {
            'position': [0.0, 0.0, 0.0],
            'velocity': [0.0, 0.0, 0.0],
            'goal_position': [0.0, 0.0, 0.0],
            'path_length': 0.0,
            'obstacles_detected': 0,
            'mapping_coverage': 0.0,
            'navigation_success': True
        }
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup ROS Publishers
        self.setup_publishers()
        
        # Setup ROS Subscribers  
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        rospy.loginfo("✅ Adaptive Controller ROS Node hazır!")
    
    def setup_publishers(self):
        """ROS publisher'larını kurar"""
        # Current stage publisher
        self.current_stage_pub = rospy.Publisher(
            '/adaptive_controller/current_stage', String, queue_size=10
        )
        
        # Control parameters publisher
        self.control_params_pub = rospy.Publisher(
            '/adaptive_controller/parameters', String, queue_size=10
        )
        
        # Performance metrics publisher
        self.performance_pub = rospy.Publisher(
            '/adaptive_controller/performance', String, queue_size=10
        )
        
        # Stage transition publisher
        self.stage_transition_pub = rospy.Publisher(
            '/adaptive_controller/stage_transition', String, queue_size=10
        )
        
        # Optimized cmd_vel publisher
        self.cmd_vel_optimized_pub = rospy.Publisher(
            '/cmd_vel_optimized', Twist, queue_size=1
        )
        
        # Stage recommendation publisher
        self.stage_recommendation_pub = rospy.Publisher(
            '/adaptive_controller/stage_recommendation', String, queue_size=10
        )
        
        # Adaptation status publisher
        self.adaptation_status_pub = rospy.Publisher(
            '/adaptive_controller/adaptation_status', String, queue_size=10
        )
        
        rospy.loginfo("📡 Adaptive Controller publishers kuruldu")
    
    def setup_subscribers(self):
        """ROS subscriber'ları kurar"""
        # Robot pose subscriber
        self.pose_sub = rospy.Subscriber(
            '/robot_pose', PoseStamped, self.pose_callback, queue_size=1
        )
        
        # Odometry subscriber
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback, queue_size=1
        )
        
        # LiDAR data subscriber
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback, queue_size=1
        )
        
        # Goal subscriber
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1
        )
        
        # Navigation status subscriber
        self.nav_status_sub = rospy.Subscriber(
            '/move_base/status', GoalStatusArray, self.nav_status_callback, queue_size=1
        )
        
        # Path subscriber
        self.path_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, self.path_callback, queue_size=1
        )
        
        # Map coverage subscriber (custom topic)
        self.coverage_sub = rospy.Subscriber(
            '/map_coverage', Float32, self.coverage_callback, queue_size=1
        )
        
        # Manual stage change subscriber
        self.manual_stage_sub = rospy.Subscriber(
            '/adaptive_controller/set_stage', String, self.manual_stage_callback, queue_size=1
        )
        
        # Safety monitor subscriber
        self.safety_sub = rospy.Subscriber(
            '/safety_monitor/level', Int32, self.safety_callback, queue_size=1
        )
        
        # Recovery status subscriber
        self.recovery_sub = rospy.Subscriber(
            '/recovery/active', Bool, self.recovery_callback, queue_size=1
        )
        
        rospy.loginfo("📡 Adaptive Controller subscribers kuruldu")
    
    def setup_timers(self):
        """Timer'ları kurar"""
        # Adaptation timer (5Hz)
        self.adaptation_timer = rospy.Timer(
            rospy.Duration(0.2), self.adaptation_callback
        )
        
        # Stage monitoring timer (1Hz)
        self.stage_monitor_timer = rospy.Timer(
            rospy.Duration(1.0), self.stage_monitor_callback
        )
        
        # Performance update timer (0.5Hz)
        self.performance_timer = rospy.Timer(
            rospy.Duration(2.0), self.performance_callback
        )
        
        rospy.loginfo("⏰ Adaptive Controller timers kuruldu")
    
    def pose_callback(self, msg):
        """Robot pose callback"""
        with self.data_lock:
            self.robot_data['position'] = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
    
    def odom_callback(self, msg):
        """Odometry callback"""
        with self.data_lock:
            self.robot_data['velocity'] = [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z
            ]
    
    def lidar_callback(self, msg):
        """LiDAR callback"""
        with self.data_lock:
            # Obstacle detection
            ranges = np.array(msg.ranges)
            valid_ranges = ranges[~np.isinf(ranges) & ~np.isnan(ranges)]
            
            if len(valid_ranges) > 0:
                obstacles = np.sum(valid_ranges < self.current_params['obstacle_threshold'])
                self.robot_data['obstacles_detected'] = obstacles
    
    def goal_callback(self, msg):
        """Goal callback"""
        with self.data_lock:
            self.robot_data['goal_position'] = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
    
    def nav_status_callback(self, msg):
        """Navigation status callback"""
        with self.data_lock:
            # Navigation success tracking
            if msg.status_list:
                latest_status = msg.status_list[-1]
                if latest_status.status == 3:  # SUCCEEDED
                    self.robot_data['navigation_success'] = True
                elif latest_status.status in [4, 5]:  # ABORTED, REJECTED
                    self.robot_data['navigation_success'] = False
    
    def path_callback(self, msg):
        """Path callback"""
        with self.data_lock:
            # Path length calculation
            if len(msg.poses) > 1:
                path_length = 0.0
                for i in range(1, len(msg.poses)):
                    p1 = msg.poses[i-1].pose.position
                    p2 = msg.poses[i].pose.position
                    dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
                    path_length += dist
                self.robot_data['path_length'] = path_length
    
    def coverage_callback(self, msg):
        """Map coverage callback"""
        with self.data_lock:
            self.robot_data['mapping_coverage'] = msg.data
    
    def manual_stage_callback(self, msg):
        """Manual stage change callback"""
        requested_stage = msg.data.lower()
        valid_stages = ['exploration', 'mapping', 'navigation', 'competition']
        
        if requested_stage in valid_stages:
            rospy.loginfo(f"🎮 Manual stage change: {self.current_stage} → {requested_stage}")
            self.change_stage(requested_stage, manual=True)
        else:
            rospy.logwarn(f"⚠️ Invalid stage requested: {requested_stage}")
    
    def safety_callback(self, msg):
        """Safety level callback"""
        safety_level = msg.data
        
        # Safety durumuna göre kontrol parametrelerini ayarla
        if safety_level >= 3:  # Critical level
            with self.data_lock:
                self.current_params['max_linear_vel'] = 0.1
                self.current_params['max_angular_vel'] = 0.2
        elif safety_level >= 2:  # Warning level
            with self.data_lock:
                self.current_params['max_linear_vel'] = 0.3
                self.current_params['max_angular_vel'] = 0.5
    
    def recovery_callback(self, msg):
        """Recovery active callback"""
        if msg.data:  # Recovery active
            # Recovery sırasında conservative parameters
            with self.data_lock:
                self.current_params['max_linear_vel'] = 0.2
                self.current_params['max_angular_vel'] = 0.3
                self.current_params['obstacle_threshold'] = 0.5
    
    def adaptation_callback(self, event):
        """Ana adaptation callback (5Hz)"""
        try:
            with self.data_lock:
                robot_data_copy = self.robot_data.copy()
            
            # Current stage için optimization yap
            self.optimize_parameters(robot_data_copy)
            
            # Optimized cmd_vel publish et (eğer varsa)
            self.publish_optimized_control()
            
        except Exception as e:
            rospy.logerr(f"❌ Adaptation callback hatası: {e}")
    
    def stage_monitor_callback(self, event):
        """Stage monitoring callback (1Hz)"""
        try:
            # Auto stage transition enabled ise
            if self.auto_stage_transition:
                self.check_stage_transition()
            
            # Performance metrics güncelle
            self.update_performance_metrics()
            
        except Exception as e:
            rospy.logerr(f"❌ Stage monitor callback hatası: {e}")
    
    def performance_callback(self, event):
        """Performance publishing callback (0.5Hz)"""
        try:
            # Performance metrics'leri yayınla
            self.publish_performance_metrics()
            
            # Adaptation status yayınla
            self.publish_adaptation_status()
            
        except Exception as e:
            rospy.logerr(f"❌ Performance callback hatası: {e}")
    
    def optimize_parameters(self, robot_data):
        """Current stage için parametreleri optimize eder"""
        try:
            # Adaptive controller'dan optimum parametreleri al
            if self.current_stage == "exploration":
                params = self.adaptive_controller.get_exploration_parameters(
                    coverage=robot_data.get('mapping_coverage', 0.0),
                    obstacles=robot_data.get('obstacles_detected', 0),
                    velocity=robot_data.get('velocity', [0.0, 0.0, 0.0])
                )
            elif self.current_stage == "mapping":
                params = self.adaptive_controller.get_mapping_parameters(
                    coverage=robot_data.get('mapping_coverage', 0.0),
                    accuracy_requirement=0.95
                )
            elif self.current_stage == "navigation":
                params = self.adaptive_controller.get_navigation_parameters(
                    path_length=robot_data.get('path_length', 0.0),
                    obstacles=robot_data.get('obstacles_detected', 0),
                    success_rate=1.0 if robot_data.get('navigation_success', False) else 0.0
                )
            elif self.current_stage == "competition":
                params = self.adaptive_controller.get_competition_parameters(
                    time_pressure=0.8,
                    task_complexity=0.7
                )
            else:
                params = {}
            
            # Parametreleri güncelle
            with self.data_lock:
                self.current_params.update(params)
            
        except Exception as e:
            rospy.logerr(f"❌ Parameter optimization hatası: {e}")
    
    def check_stage_transition(self):
        """Stage transition gerekip gerekmediğini kontrol eder"""
        stage_time = time.time() - self.stage_start_time
        
        with self.data_lock:
            coverage = self.robot_data.get('mapping_coverage', 0.0)
            nav_success = self.robot_data.get('navigation_success', False)
        
        # Stage transition logic
        new_stage = None
        
        if self.current_stage == "exploration":
            # Exploration → Mapping (yeterli keşif yapıldı)
            if coverage > 0.3 or stage_time > 300:  # 5 dakika
                new_stage = "mapping"
        
        elif self.current_stage == "mapping":
            # Mapping → Navigation (harita tamamlandı)
            if coverage > 0.8 or stage_time > 600:  # 10 dakika
                new_stage = "navigation"
        
        elif self.current_stage == "navigation":
            # Navigation → Competition (navigation test edildi)
            if nav_success and stage_time > 120:  # 2 dakika
                new_stage = "competition"
        
        # Stage transition yap
        if new_stage:
            self.change_stage(new_stage, manual=False)
    
    def change_stage(self, new_stage, manual=False):
        """Stage değiştirir"""
        old_stage = self.current_stage
        self.current_stage = new_stage
        self.stage_start_time = time.time()
        
        # Adaptive controller'a bildir
        try:
            self.adaptive_controller.set_current_stage(new_stage)
        except Exception as e:
            rospy.logerr(f"❌ Stage change hatası: {e}")
        
        # Stage transition mesajı yayınla
        transition_msg = String()
        transition_type = "MANUAL" if manual else "AUTO"
        transition_msg.data = f"{transition_type}: {old_stage} → {new_stage}"
        self.stage_transition_pub.publish(transition_msg)
        
        rospy.loginfo(f"🎮 Stage changed: {old_stage} → {new_stage} ({transition_type})")
    
    def update_performance_metrics(self):
        """Performance metrics'leri günceller"""
        stage_time = time.time() - self.stage_start_time
        
        with self.data_lock:
            if self.current_stage == "exploration":
                self.performance_metrics['exploration'] = {
                    'efficiency': min(self.robot_data.get('mapping_coverage', 0.0) / max(stage_time/60, 1), 1.0),
                    'coverage': self.robot_data.get('mapping_coverage', 0.0),
                    'time': stage_time
                }
            elif self.current_stage == "mapping":
                self.performance_metrics['mapping'] = {
                    'accuracy': 0.95,  # Assumption
                    'completeness': self.robot_data.get('mapping_coverage', 0.0),
                    'time': stage_time
                }
            elif self.current_stage == "navigation":
                self.performance_metrics['navigation'] = {
                    'success_rate': 1.0 if self.robot_data.get('navigation_success', False) else 0.0,
                    'path_length': self.robot_data.get('path_length', 0.0),
                    'time': stage_time
                }
            elif self.current_stage == "competition":
                # Competition performance (placeholder)
                self.performance_metrics['competition'] = {
                    'task_completion': 0.8,  # Assumption
                    'efficiency': 0.9,
                    'time': stage_time
                }
    
    def publish_optimized_control(self):
        """Optimized control commands yayınlar"""
        # Bu method, mevcut cmd_vel'i alıp optimize edebilir
        # Şu an için placeholder - gerçek implementation'da
        # /cmd_vel topic'ini dinleyip optimize edebilir
        pass
    
    def publish_performance_metrics(self):
        """Performance metrics'leri yayınlar"""
        # Current stage performance
        current_perf = self.performance_metrics.get(self.current_stage, {})
        
        perf_msg = String()
        perf_msg.data = f"Stage: {self.current_stage}, " \
                       f"Metrics: {', '.join([f'{k}:{v:.3f}' for k, v in current_perf.items()])}"
        self.performance_pub.publish(perf_msg)
        
        # Control parameters
        params_msg = String()
        params_msg.data = f"Params: {', '.join([f'{k}:{v:.3f}' for k, v in self.current_params.items()])}"
        self.control_params_pub.publish(params_msg)
        
        # Current stage
        stage_msg = String()
        stage_msg.data = self.current_stage
        self.current_stage_pub.publish(stage_msg)
    
    def publish_adaptation_status(self):
        """Adaptation status'u yayınlar"""
        stage_time = time.time() - self.stage_start_time
        
        status_msg = String()
        status_msg.data = f"Stage: {self.current_stage} ({stage_time:.1f}s), " \
                         f"Auto-transition: {self.auto_stage_transition}, " \
                         f"Parameters optimized"
        self.adaptation_status_pub.publish(status_msg)
    
    def shutdown_handler(self):
        """Node kapanma işlemleri"""
        rospy.loginfo("🎮 Adaptive Controller Node kapatılıyor...")
        
        # Adaptive controller'ı kapat
        try:
            self.adaptive_controller.stop_adaptation()
        except:
            pass
        
        rospy.loginfo("✅ Adaptive Controller Node kapatıldı")

def main():
    """Ana program"""
    try:
        # Adaptive Controller ROS Node başlat
        controller_node = AdaptiveControllerROSNode()
        
        # Shutdown handler kaydet
        rospy.on_shutdown(controller_node.shutdown_handler)
        
        rospy.loginfo("🚀 Adaptive Controller ROS Node çalışıyor...")
        rospy.loginfo("📊 Topics:")
        rospy.loginfo("  - Input: /odom, /scan, /move_base/status, /robot_pose")
        rospy.loginfo("  - Output: /adaptive_controller/current_stage, /cmd_vel_optimized")
        rospy.loginfo("🎮 4 Adaptive Stages READY!")
        rospy.loginfo(f"🎯 Current Stage: {controller_node.current_stage}")
        
        # ROS spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🎮 Adaptive Controller Node durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Adaptive Controller Node hatası: {e}")

if __name__ == '__main__':
    main()

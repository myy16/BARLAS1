#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2025 - RECOVERY AUTOMATION ROS WRAPPER
==============================================

Bu ROS wrapper, Recovery Automation System'i ROS ekosistemi ile entegre eder.
Sistem hatalarını otomatik tespit eder ve 7 farklı recovery planı uygular.

Features:
- 7 recovery plan (Navigation, Sensor, Communication, Power, Hardware, Software, Emergency)
- 8 recovery action automatic execution
- Real-time failure detection
- Recovery status broadcasting
- System health monitoring

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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# Recovery Automation System import
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from recovery_automation_system import RecoveryAutomationSystem

class RecoveryAutomationROSNode:
    """
    Recovery Automation System için ROS Wrapper
    
    Bu class, Recovery Automation System'i ROS topics ile entegre eder.
    Sistem hatalarını tespit eder, recovery plan'ları çalıştırır.
    """
    
    def __init__(self):
        """ROS node ve Recovery Automation System'i başlatır"""
        # ROS Node başlat
        rospy.init_node('recovery_automation_node', anonymous=True)
        rospy.loginfo("🔧 Recovery Automation ROS Node başlatılıyor...")
        
        # Recovery Automation System oluştur
        self.recovery_system = RecoveryAutomationSystem()
        
        # System status tracking
        self.system_status = {
            'navigation_system': {'healthy': True, 'last_update': time.time()},
            'sensor_system': {'healthy': True, 'last_update': time.time()},
            'communication_system': {'healthy': True, 'last_update': time.time()},
            'power_system': {'healthy': True, 'last_update': time.time()},
            'hardware_system': {'healthy': True, 'last_update': time.time()},
            'software_system': {'healthy': True, 'last_update': time.time()}
        }
        
        # Recovery state tracking
        self.recovery_active = False
        self.current_recovery_plan = None
        self.recovery_start_time = None
        
        # Data storage
        self.sensor_data = {
            'lidar_healthy': True,
            'imu_healthy': True,
            'odom_healthy': True,
            'camera_healthy': True,
            'safety_healthy': True
        }
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Setup ROS Publishers
        self.setup_publishers()
        
        # Setup ROS Subscribers  
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        rospy.loginfo("✅ Recovery Automation ROS Node hazır!")
    
    def setup_publishers(self):
        """ROS publisher'larını kurar"""
        # Recovery status publisher
        self.recovery_status_pub = rospy.Publisher(
            '/recovery/status', String, queue_size=10
        )
        
        # Active recovery plan publisher
        self.active_plan_pub = rospy.Publisher(
            '/recovery/active_plan', String, queue_size=10
        )
        
        # Recovery action publisher
        self.recovery_action_pub = rospy.Publisher(
            '/recovery/action', String, queue_size=10
        )
        
        # System health publisher
        self.system_health_pub = rospy.Publisher(
            '/recovery/system_health', DiagnosticArray, queue_size=10
        )
        
        # Recovery success rate publisher
        self.success_rate_pub = rospy.Publisher(
            '/recovery/success_rate', Float32, queue_size=10
        )
        
        # Recovery active flag publisher
        self.recovery_active_pub = rospy.Publisher(
            '/recovery/active', Bool, queue_size=10
        )
        
        # Emergency recovery publisher
        self.emergency_recovery_pub = rospy.Publisher(
            '/recovery/emergency', Bool, queue_size=1
        )
        
        rospy.loginfo("📡 Recovery Automation publishers kuruldu")
    
    def setup_subscribers(self):
        """ROS subscriber'ları kurar"""
        # Safety monitor status subscriber
        self.safety_status_sub = rospy.Subscriber(
            '/safety_monitor/status', String, self.safety_status_callback, queue_size=1
        )
        
        # Navigation status subscriber
        self.nav_status_sub = rospy.Subscriber(
            '/move_base/status', String, self.navigation_status_callback, queue_size=1
        )
        
        # LiDAR health subscriber
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_health_callback, queue_size=1
        )
        
        # IMU health subscriber
        self.imu_sub = rospy.Subscriber(
            '/imu/data', Imu, self.imu_health_callback, queue_size=1
        )
        
        # Odometry health subscriber
        self.odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.odom_health_callback, queue_size=1
        )
        
        # Battery voltage subscriber
        self.battery_sub = rospy.Subscriber(
            '/battery_voltage', Float32, self.battery_callback, queue_size=1
        )
        
        # Temperature subscriber
        self.temp_sub = rospy.Subscriber(
            '/temperature', Float32, self.temperature_callback, queue_size=1
        )
        
        # System diagnostics subscriber
        self.diagnostics_sub = rospy.Subscriber(
            '/diagnostics', DiagnosticArray, self.diagnostics_callback, queue_size=1
        )
        
        rospy.loginfo("📡 Recovery Automation subscribers kuruldu")
    
    def setup_timers(self):
        """Timer'ları kurar"""
        # Recovery monitoring timer (10Hz)
        self.monitor_timer = rospy.Timer(
            rospy.Duration(0.1), self.monitor_callback
        )
        
        # Status publishing timer (2Hz)
        self.status_timer = rospy.Timer(
            rospy.Duration(0.5), self.status_callback
        )
        
        # System health check timer (1Hz)
        self.health_timer = rospy.Timer(
            rospy.Duration(1.0), self.health_callback
        )
        
        rospy.loginfo("⏰ Recovery Automation timers kuruldu")
    
    def safety_status_callback(self, msg):
        """Safety monitor status callback"""
        with self.data_lock:
            # Safety system health kontrol
            if "Emergency" in msg.data or "Critical" in msg.data:
                self.system_status['software_system']['healthy'] = False
            else:
                self.system_status['software_system']['healthy'] = True
            
            self.system_status['software_system']['last_update'] = time.time()
    
    def navigation_status_callback(self, msg):
        """Navigation system status callback"""
        with self.data_lock:
            # Navigation health kontrolü
            if "SUCCEEDED" in msg.data or "ACTIVE" in msg.data:
                self.system_status['navigation_system']['healthy'] = True
            else:
                self.system_status['navigation_system']['healthy'] = False
            
            self.system_status['navigation_system']['last_update'] = time.time()
    
    def lidar_health_callback(self, msg):
        """LiDAR health check callback"""
        with self.data_lock:
            current_time = time.time()
            
            # LiDAR data quality kontrolü
            ranges = np.array(msg.ranges)
            valid_ranges = ranges[~np.isinf(ranges) & ~np.isnan(ranges)]
            
            if len(valid_ranges) > len(ranges) * 0.5:  # En az %50 valid data
                self.sensor_data['lidar_healthy'] = True
            else:
                self.sensor_data['lidar_healthy'] = False
            
            self.system_status['sensor_system']['last_update'] = current_time
    
    def imu_health_callback(self, msg):
        """IMU health check callback"""
        with self.data_lock:
            current_time = time.time()
            
            # IMU data validity kontrolü
            acc_norm = np.sqrt(
                msg.linear_acceleration.x**2 + 
                msg.linear_acceleration.y**2 + 
                msg.linear_acceleration.z**2
            )
            
            # Normal gravity range kontrolü (8-12 m/s²)
            if 8.0 < acc_norm < 12.0:
                self.sensor_data['imu_healthy'] = True
            else:
                self.sensor_data['imu_healthy'] = False
            
            self.system_status['sensor_system']['last_update'] = current_time
    
    def odom_health_callback(self, msg):
        """Odometry health check callback"""
        with self.data_lock:
            current_time = time.time()
            
            # Odometry data sanity check
            vel_norm = np.sqrt(
                msg.twist.twist.linear.x**2 + 
                msg.twist.twist.linear.y**2
            )
            
            # Reasonable velocity range (0-5 m/s)
            if vel_norm < 5.0:
                self.sensor_data['odom_healthy'] = True
            else:
                self.sensor_data['odom_healthy'] = False
            
            self.system_status['sensor_system']['last_update'] = current_time
    
    def battery_callback(self, msg):
        """Battery status callback"""
        with self.data_lock:
            # Battery health kontrolü
            if msg.data > 10.0:  # Minimum battery voltage
                self.system_status['power_system']['healthy'] = True
            else:
                self.system_status['power_system']['healthy'] = False
            
            self.system_status['power_system']['last_update'] = time.time()
    
    def temperature_callback(self, msg):
        """Temperature status callback"""
        with self.data_lock:
            # Temperature health kontrolü (0-60°C)
            if 0 < msg.data < 60:
                self.system_status['hardware_system']['healthy'] = True
            else:
                self.system_status['hardware_system']['healthy'] = False
            
            self.system_status['hardware_system']['last_update'] = time.time()
    
    def diagnostics_callback(self, msg):
        """System diagnostics callback"""
        with self.data_lock:
            # Diagnostic messages analizi
            error_count = 0
            for status in msg.status:
                if status.level == DiagnosticStatus.ERROR:
                    error_count += 1
            
            # Communication system health
            if error_count < 3:  # Maximum 2 error tolerable
                self.system_status['communication_system']['healthy'] = True
            else:
                self.system_status['communication_system']['healthy'] = False
            
            self.system_status['communication_system']['last_update'] = time.time()
    
    def monitor_callback(self, event):
        """Ana recovery monitoring callback (10Hz)"""
        try:
            with self.data_lock:
                # System durumunu kopyala
                status_copy = self.system_status.copy()
                sensor_copy = self.sensor_data.copy()
            
            # Failure detection
            failures = self.detect_failures(status_copy, sensor_copy)
            
            # Recovery ihtiyacı var mı kontrol et
            if failures and not self.recovery_active:
                self.start_recovery(failures)
            elif not failures and self.recovery_active:
                self.complete_recovery()
            
            # Active recovery monitoring
            if self.recovery_active:
                self.monitor_recovery_progress()
            
        except Exception as e:
            rospy.logerr(f"❌ Recovery monitoring hatası: {e}")
    
    def detect_failures(self, status, sensors):
        """Sistem hatalarını tespit eder"""
        failures = []
        current_time = time.time()
        
        # Timeout kontrolü (5 saniye)
        timeout_threshold = 5.0
        
        # System status kontrolü
        for system_name, system_info in status.items():
            if not system_info['healthy']:
                failures.append(f"{system_name}_unhealthy")
            
            # Timeout kontrolü
            if current_time - system_info['last_update'] > timeout_threshold:
                failures.append(f"{system_name}_timeout")
        
        # Sensor health kontrolü
        for sensor_name, sensor_healthy in sensors.items():
            if not sensor_healthy:
                failures.append(f"{sensor_name}_failure")
        
        return failures
    
    def start_recovery(self, failures):
        """Recovery sürecini başlatır"""
        self.recovery_active = True
        self.recovery_start_time = time.time()
        
        # Recovery plan seç
        recovery_plan = self.select_recovery_plan(failures)
        self.current_recovery_plan = recovery_plan
        
        rospy.logwarn(f"🔧 Recovery başlatıldı: {recovery_plan}")
        rospy.logwarn(f"🔧 Tespit edilen hatalar: {failures}")
        
        # Recovery system'e bildir
        try:
            if recovery_plan == "navigation_recovery":
                self.recovery_system.execute_navigation_recovery()
            elif recovery_plan == "sensor_recovery":
                self.recovery_system.execute_sensor_recovery()
            elif recovery_plan == "communication_recovery":
                self.recovery_system.execute_communication_recovery()
            elif recovery_plan == "power_recovery":
                self.recovery_system.execute_power_recovery()
            elif recovery_plan == "hardware_recovery":
                self.recovery_system.execute_hardware_recovery()
            elif recovery_plan == "software_recovery":
                self.recovery_system.execute_software_recovery()
            elif recovery_plan == "emergency_recovery":
                self.recovery_system.execute_emergency_recovery()
                # Emergency recovery mesajı
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_recovery_pub.publish(emergency_msg)
        
        except Exception as e:
            rospy.logerr(f"❌ Recovery execution hatası: {e}")
        
        # Recovery action mesajı yayınla
        action_msg = String()
        action_msg.data = f"Started {recovery_plan} for failures: {', '.join(failures)}"
        self.recovery_action_pub.publish(action_msg)
    
    def select_recovery_plan(self, failures):
        """Hatalara göre recovery plan seçer"""
        # Critical failures - Emergency recovery
        emergency_keywords = ['emergency', 'critical', 'power_system_unhealthy']
        if any(keyword in ' '.join(failures) for keyword in emergency_keywords):
            return "emergency_recovery"
        
        # Navigation failures
        navigation_keywords = ['navigation', 'move_base', 'path']
        if any(keyword in ' '.join(failures) for keyword in navigation_keywords):
            return "navigation_recovery"
        
        # Sensor failures
        sensor_keywords = ['sensor', 'lidar', 'imu', 'odom', 'camera']
        if any(keyword in ' '.join(failures) for keyword in sensor_keywords):
            return "sensor_recovery"
        
        # Communication failures
        comm_keywords = ['communication', 'timeout', 'connection']
        if any(keyword in ' '.join(failures) for keyword in comm_keywords):
            return "communication_recovery"
        
        # Power failures
        power_keywords = ['power', 'battery', 'voltage']
        if any(keyword in ' '.join(failures) for keyword in power_keywords):
            return "power_recovery"
        
        # Hardware failures
        hardware_keywords = ['hardware', 'temperature', 'motor']
        if any(keyword in ' '.join(failures) for keyword in hardware_keywords):
            return "hardware_recovery"
        
        # Default: software recovery
        return "software_recovery"
    
    def monitor_recovery_progress(self):
        """Recovery progress'ini izler"""
        if not self.recovery_active:
            return
        
        # Recovery timeout kontrolü (30 saniye)
        recovery_time = time.time() - self.recovery_start_time
        if recovery_time > 30.0:
            rospy.logwarn("⚠️ Recovery timeout, escalating to emergency recovery")
            self.current_recovery_plan = "emergency_recovery"
            self.recovery_system.execute_emergency_recovery()
    
    def complete_recovery(self):
        """Recovery tamamlandığını işler"""
        if not self.recovery_active:
            return
        
        recovery_time = time.time() - self.recovery_start_time
        
        rospy.loginfo(f"✅ Recovery tamamlandı: {self.current_recovery_plan}")
        rospy.loginfo(f"✅ Recovery süresi: {recovery_time:.2f} saniye")
        
        # Recovery durumunu sıfırla
        self.recovery_active = False
        self.current_recovery_plan = None
        self.recovery_start_time = None
        
        # Recovery tamamlandı mesajı
        action_msg = String()
        action_msg.data = f"Recovery completed successfully in {recovery_time:.2f}s"
        self.recovery_action_pub.publish(action_msg)
        
        # Emergency recovery'den çık
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_recovery_pub.publish(emergency_msg)
    
    def status_callback(self, event):
        """Status yayınlama callback (2Hz)"""
        try:
            # Recovery status yayınla
            self.publish_recovery_status()
            
        except Exception as e:
            rospy.logerr(f"❌ Recovery status callback hatası: {e}")
    
    def health_callback(self, event):
        """System health callback (1Hz)"""
        try:
            # System health diagnostics yayınla
            self.publish_system_health()
            
        except Exception as e:
            rospy.logerr(f"❌ Health callback hatası: {e}")
    
    def publish_recovery_status(self):
        """Recovery status mesajlarını yayınlar"""
        # Recovery status
        status_msg = String()
        if self.recovery_active:
            recovery_time = time.time() - self.recovery_start_time
            status_msg.data = f"ACTIVE: {self.current_recovery_plan} ({recovery_time:.1f}s)"
        else:
            status_msg.data = "IDLE"
        self.recovery_status_pub.publish(status_msg)
        
        # Active plan
        plan_msg = String()
        plan_msg.data = self.current_recovery_plan if self.current_recovery_plan else "none"
        self.active_plan_pub.publish(plan_msg)
        
        # Recovery active flag
        active_msg = Bool()
        active_msg.data = self.recovery_active
        self.recovery_active_pub.publish(active_msg)
        
        # Success rate (recovery system'den al)
        try:
            success_rate = self.recovery_system.get_recovery_statistics().get('success_rate', 0.0)
            rate_msg = Float32()
            rate_msg.data = float(success_rate)
            self.success_rate_pub.publish(rate_msg)
        except:
            pass
    
    def publish_system_health(self):
        """System health diagnostics yayınlar"""
        with self.data_lock:
            status_copy = self.system_status.copy()
            sensor_copy = self.sensor_data.copy()
        
        # Diagnostic array oluştur
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        # System health status'leri ekle
        for system_name, system_info in status_copy.items():
            status = DiagnosticStatus()
            status.name = system_name
            status.hardware_id = "teknofest_robot"
            
            if system_info['healthy']:
                status.level = DiagnosticStatus.OK
                status.message = "System healthy"
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "System unhealthy"
            
            # Timing info ekle
            age = time.time() - system_info['last_update']
            status.values.append(
                {
                    'key': 'last_update_age',
                    'value': f"{age:.1f}s"
                }
            )
            
            diag_array.status.append(status)
        
        # Sensor health status'leri ekle
        for sensor_name, sensor_healthy in sensor_copy.items():
            status = DiagnosticStatus()
            status.name = sensor_name
            status.hardware_id = "teknofest_robot"
            
            if sensor_healthy:
                status.level = DiagnosticStatus.OK
                status.message = "Sensor healthy"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "Sensor degraded"
            
            diag_array.status.append(status)
        
        self.system_health_pub.publish(diag_array)
    
    def shutdown_handler(self):
        """Node kapanma işlemleri"""
        rospy.loginfo("🔧 Recovery Automation Node kapatılıyor...")
        
        # Active recovery'yi durdur
        if self.recovery_active:
            rospy.loginfo("🔧 Active recovery durdurulluyor...")
            self.recovery_active = False
        
        # Recovery system'i kapat
        self.recovery_system.stop_monitoring()
        
        rospy.loginfo("✅ Recovery Automation Node kapatıldı")

def main():
    """Ana program"""
    try:
        # Recovery Automation ROS Node başlat
        recovery_node = RecoveryAutomationROSNode()
        
        # Shutdown handler kaydet
        rospy.on_shutdown(recovery_node.shutdown_handler)
        
        rospy.loginfo("🚀 Recovery Automation ROS Node çalışıyor...")
        rospy.loginfo("📊 Topics:")
        rospy.loginfo("  - Input: /safety_monitor/status, /move_base/status, /diagnostics")
        rospy.loginfo("  - Output: /recovery/status, /recovery/action, /recovery/emergency")
        rospy.loginfo("🔧 7 Recovery Plans READY!")
        
        # ROS spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🔧 Recovery Automation Node durduruldu")
    except Exception as e:
        rospy.logerr(f"❌ Recovery Automation Node hatası: {e}")

if __name__ == '__main__':
    main()

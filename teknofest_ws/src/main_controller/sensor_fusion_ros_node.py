#!/usr/bin/env python3
"""
🔬 SENSOR FUSION ROS WRAPPER
Sensor Fusion System'i ROS node'u haline getirir
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray

# Mevcut sensor fusion system'i import et
from sensor_fusion_system import SensorFusionSystem

class SensorFusionROSNode:
    """
    ROS Wrapper for Sensor Fusion System
    Python class'ını ROS node'u haline getirir
    """
    
    def __init__(self):
        # ROS node başlat
        rospy.init_node('sensor_fusion_node', anonymous=True)
        rospy.loginfo("🔬 Sensor Fusion ROS Node başlatıldı")
        
        # Mevcut Python class'ını oluştur
        self.fusion_system = SensorFusionSystem()
        
        # ROS Subscribers (Sensör verilerini dinle)
        self.lidar_sub = rospy.Subscriber(
            '/scan', 
            LaserScan, 
            self.lidar_callback, 
            queue_size=10
        )
        
        self.imu_sub = rospy.Subscriber(
            '/imu/data', 
            Imu, 
            self.imu_callback, 
            queue_size=10
        )
        
        self.odom_sub = rospy.Subscriber(
            '/odom', 
            Odometry, 
            self.encoder_callback, 
            queue_size=10
        )
        
        # ROS Publishers (İşlenmiş veriyi yayınla)
        self.fused_pose_pub = rospy.Publisher(
            '/sensor_fusion/pose', 
            PoseStamped, 
            queue_size=10
        )
        
        self.fusion_confidence_pub = rospy.Publisher(
            '/sensor_fusion/confidence', 
            Float32MultiArray, 
            queue_size=10
        )
        
        # Timer (Periyodik çalışma)
        self.fusion_timer = rospy.Timer(
            rospy.Duration(0.1),  # 10Hz
            self.fusion_update_callback
        )
        
        # Başlatma
        self.fusion_system.start()
        rospy.loginfo("✅ Sensor Fusion System ROS wrapper hazır")
    
    def lidar_callback(self, msg):
        """LiDAR verisi geldiğinde çağrılır"""
        try:
            # ROS LaserScan mesajını numpy array'e çevir
            ranges = np.array(msg.ranges)
            
            # Geçersiz değerleri filtrele
            valid_ranges = ranges[np.isfinite(ranges)]
            
            # Ortalama mesafe hesapla (basit örnek)
            if len(valid_ranges) > 0:
                avg_distance = np.mean(valid_ranges)
                
                # Position estimate (basitleştirilmiş)
                lidar_measurement = np.array([
                    avg_distance * np.cos(0),  # x
                    avg_distance * np.sin(0)   # y
                ])
                
                # Mevcut Python class'ına gönder
                # NOT: Bu metod yok, eklenmesi gerekiyor
                # self.fusion_system.update_lidar(lidar_measurement)
                
        except Exception as e:
            rospy.logerr(f"❌ LiDAR callback hatası: {e}")
    
    def imu_callback(self, msg):
        """IMU verisi geldiğinde çağrılır"""
        try:
            # ROS Imu mesajından orientation al
            orientation = msg.orientation
            
            # Quaternion'dan yaw açısını hesapla
            import tf.transformations as tf_trans
            (roll, pitch, yaw) = tf_trans.euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            
            # IMU measurement oluştur
            imu_measurement = np.array([yaw])
            
            # Mevcut Python class'ına gönder
            # self.fusion_system.update_imu(imu_measurement)
            
        except Exception as e:
            rospy.logerr(f"❌ IMU callback hatası: {e}")
    
    def encoder_callback(self, msg):
        """Encoder/Odometry verisi geldiğinde çağrılır"""
        try:
            # Linear velocity al
            linear_velocity = msg.twist.twist.linear.x
            
            # Encoder measurement oluştur
            encoder_measurement = np.array([linear_velocity])
            
            # Mevcut Python class'ına gönder
            # self.fusion_system.update_encoder(encoder_measurement)
            
        except Exception as e:
            rospy.logerr(f"❌ Encoder callback hatası: {e}")
    
    def fusion_update_callback(self, event):
        """10Hz'de fusion sonuçlarını yayınla"""
        try:
            # Python class'ından sonuç al
            fusion_status = self.fusion_system.get_fusion_status()
            
            # ROS PoseStamped mesajı oluştur
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"
            
            # Fusion sonucunu pose'a çevir (örnek)
            pose_msg.pose.position.x = fusion_status.get('estimated_position', {}).get('x', 0.0)
            pose_msg.pose.position.y = fusion_status.get('estimated_position', {}).get('y', 0.0)
            pose_msg.pose.position.z = 0.0
            
            # Publish et
            self.fused_pose_pub.publish(pose_msg)
            
            # Confidence bilgisi yayınla
            confidence_msg = Float32MultiArray()
            confidence_msg.data = [
                fusion_status.get('fusion_confidence', 0.0),
                float(fusion_status.get('active_sensors', 0)),
                float(fusion_status.get('obstacles_detected', 0))
            ]
            self.fusion_confidence_pub.publish(confidence_msg)
            
        except Exception as e:
            rospy.logerr(f"❌ Fusion update hatası: {e}")
    
    def shutdown_hook(self):
        """Node kapatılırken çağrılır"""
        rospy.loginfo("🛑 Sensor Fusion Node kapatılıyor...")
        self.fusion_system.stop()

def main():
    try:
        # ROS wrapper node'unu başlat
        fusion_node = SensorFusionROSNode()
        
        # Shutdown hook ekle
        rospy.on_shutdown(fusion_node.shutdown_hook)
        
        # ROS spin (sürekli çalış)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Node başlatma hatası: {e}")

if __name__ == '__main__':
    main()

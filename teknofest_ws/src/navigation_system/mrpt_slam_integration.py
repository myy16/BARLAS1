#!/usr/bin/env python3
"""
TEKNOFEST MRPT RBPF SLAM Integration
my_robot'daki MRPT SLAM kodlarını teknofest sistemine entegre eder
"""

import rospy
import tf2_ros
import numpy as np
import threading
import time
from typing import Optional, Tuple, List

# ROS messages
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose

class MRPTSLAMIntegration:
    """
    MRPT RBPF SLAM teknofest entegrasyonu
    """
    
    def __init__(self):
        """
        MRPT SLAM entegrasyon başlatıcı
        """
        # ROS node
        rospy.init_node('mrpt_slam_integration', anonymous=True)
        
        # Parameters
        self.global_frame = rospy.get_param('~global_frame_id', 'map')
        self.odom_frame = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame = rospy.get_param('~base_frame_id', 'base_link')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        
        # MRPT SLAM process connection
        self.mrpt_slam_running = False
        self.current_pose = None
        self.current_map = None
        self.particle_count = 200  # my_robot config'den
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.pose_pub = rospy.Publisher('/slam_pose', PoseStamped, queue_size=1)
        self.map_pub = rospy.Publisher('/slam_map', OccupancyGrid, queue_size=1)
        self.particles_pub = rospy.Publisher('/slam_particles', PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # MRPT SLAM node monitoring
        rospy.Subscriber('/mrpt_rbpf_slam/pose', PoseStamped, self.mrpt_pose_callback)
        rospy.Subscriber('/mrpt_rbpf_slam/map', OccupancyGrid, self.mrpt_map_callback)
        
        # Threading
        self.lock = threading.Lock()
        self.monitoring_thread = None
        self.is_running = False
        
        rospy.loginfo("🗺️ MRPT SLAM Integration başlatıldı")
        rospy.loginfo(f"📍 Global frame: {self.global_frame}")
        rospy.loginfo(f"🎯 Base frame: {self.base_frame}")
        rospy.loginfo(f"📡 Scan topic: {self.scan_topic}")
    
    def start_slam(self):
        """
        MRPT SLAM sürecini başlat
        """
        try:
            # MRPT SLAM node'unun çalışıp çalışmadığını kontrol et
            slam_topics = rospy.get_published_topics()
            mrpt_running = any('/mrpt_rbpf_slam' in topic[0] for topic in slam_topics)
            
            if mrpt_running:
                rospy.loginfo("✅ MRPT SLAM node zaten çalışıyor")
                self.mrpt_slam_running = True
            else:
                rospy.logwarn("⚠️ MRPT SLAM node çalışmıyor")
                rospy.loginfo("💡 Şu komutu çalıştırın:")
                rospy.loginfo("roslaunch my_robot rbpf_slam.launch")
                self.mrpt_slam_running = False
            
            # Monitoring thread başlat
            self.is_running = True
            self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
            self.monitoring_thread.start()
            
            return self.mrpt_slam_running
            
        except Exception as e:
            rospy.logerr(f"❌ SLAM başlatma hatası: {e}")
            return False
    
    def stop_slam(self):
        """
        MRPT SLAM sürecini durdur
        """
        with self.lock:
            self.is_running = False
            self.mrpt_slam_running = False
        
        rospy.loginfo("🛑 MRPT SLAM integration durdu")
    
    def get_current_pose(self) -> Optional[PoseStamped]:
        """
        Güncel robot pose'unu döndür
        
        Returns:
            PoseStamped: Robot pozisyonu (map frame'de)
        """
        with self.lock:
            return self.current_pose
    
    def get_current_map(self) -> Optional[OccupancyGrid]:
        """
        Güncel haritayı döndür
        
        Returns:
            OccupancyGrid: SLAM haritası
        """
        with self.lock:
            return self.current_map
    
    def is_slam_ready(self) -> bool:
        """
        SLAM sisteminin hazır olup olmadığını kontrol et
        
        Returns:
            bool: Hazır durumu
        """
        return (self.mrpt_slam_running and 
                self.current_pose is not None and 
                self.current_map is not None)
    
    def navigate_to_point(self, x: float, y: float) -> bool:
        """
        Belirtilen noktaya navigasyon komutu gönder
        
        Args:
            x: Hedef X koordinatı (map frame)
            y: Hedef Y koordinatı (map frame)
            
        Returns:
            bool: Navigasyon başlatma durumu
        """
        if not self.is_slam_ready():
            rospy.logwarn("⚠️ SLAM sistemi hazır değil - navigasyon yapılamaz")
            return False
        
        try:
            # Goal pose oluştur
            goal = PoseStamped()
            goal.header.frame_id = self.global_frame
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0  # Yön önemli değil şimdilik
            
            # Navigation stack'e goal gönder (eğer varsa)
            # Bu kısım move_base ile entegre edilecek
            rospy.loginfo(f"🎯 Navigasyon hedefi: ({x:.2f}, {y:.2f})")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Navigasyon hatası: {e}")
            return False
    
    def get_map_info(self) -> dict:
        """
        Harita bilgilerini döndür
        
        Returns:
            dict: Harita bilgileri
        """
        if self.current_map is None:
            return {}
        
        info = self.current_map.info
        return {
            'resolution': info.resolution,
            'width': info.width,
            'height': info.height,
            'origin_x': info.origin.position.x,
            'origin_y': info.origin.position.y,
            'map_size_m': (info.width * info.resolution, info.height * info.resolution)
        }
    
    def scan_callback(self, msg: LaserScan):
        """
        LiDAR scan callback
        
        Args:
            msg: LaserScan mesajı
        """
        # MRPT SLAM'e veri akışını logla
        if hasattr(self, '_last_scan_log'):
            if time.time() - self._last_scan_log > 5.0:  # 5 saniyede bir log
                rospy.logdebug(f"📡 Scan verisi: {len(msg.ranges)} nokta")
                self._last_scan_log = time.time()
        else:
            self._last_scan_log = time.time()
    
    def imu_callback(self, msg: Imu):
        """
        IMU callback
        
        Args:
            msg: IMU mesajı
        """
        # IMU verilerini validate et
        if (not np.isfinite(msg.angular_velocity.x) or
            not np.isfinite(msg.linear_acceleration.x)):
            rospy.logwarn("⚠️ IMU verilerinde NaN değer tespit edildi")
    
    def odom_callback(self, msg: Odometry):
        """
        Odometry callback
        
        Args:
            msg: Odometry mesajı
        """
        # Odometry verilerini validate et
        linear_vel = np.sqrt(msg.twist.twist.linear.x**2 + 
                           msg.twist.twist.linear.y**2)
        
        if linear_vel > 5.0:  # 5 m/s üzeri anormal
            rospy.logwarn(f"⚠️ Anormal hız tespit edildi: {linear_vel:.2f} m/s")
    
    def mrpt_pose_callback(self, msg: PoseStamped):
        """
        MRPT SLAM pose callback
        
        Args:
            msg: MRPT'den gelen pose
        """
        with self.lock:
            self.current_pose = msg
        
        # Kendi pose publisher'ımıza da gönder
        self.pose_pub.publish(msg)
        
        # Log periodic pose updates
        if hasattr(self, '_last_pose_log'):
            if time.time() - self._last_pose_log > 2.0:  # 2 saniyede bir
                x, y = msg.pose.position.x, msg.pose.position.y
                rospy.loginfo(f"🤖 Robot pozisyonu: ({x:.2f}, {y:.2f})")
                self._last_pose_log = time.time()
        else:
            self._last_pose_log = time.time()
    
    def mrpt_map_callback(self, msg: OccupancyGrid):
        """
        MRPT SLAM map callback
        
        Args:
            msg: MRPT'den gelen harita
        """
        with self.lock:
            self.current_map = msg
        
        # Kendi map publisher'ımıza da gönder
        self.map_pub.publish(msg)
        
        # Map statistics
        occupied_cells = sum(1 for cell in msg.data if cell > 50)  # %50+ occupied
        free_cells = sum(1 for cell in msg.data if cell >= 0 and cell <= 25)  # %25- free
        
        rospy.loginfo_throttle(10, f"🗺️ Harita: {occupied_cells} dolu, {free_cells} boş hücre")
    
    def _monitoring_loop(self):
        """
        MRPT SLAM izleme döngüsü
        """
        rospy.loginfo("👁️ MRPT SLAM monitoring başladı")
        
        while self.is_running and not rospy.is_shutdown():
            try:
                # SLAM node durumunu kontrol et
                current_time = rospy.Time.now()
                
                # Pose timeout kontrolü
                if (self.current_pose and 
                    (current_time - self.current_pose.header.stamp).to_sec() > 10.0):
                    rospy.logwarn("⚠️ SLAM pose timeout - 10 saniyedir güncelleme yok")
                
                # Map timeout kontrolü  
                if (self.current_map and
                    (current_time - self.current_map.header.stamp).to_sec() > 30.0):
                    rospy.logwarn("⚠️ SLAM map timeout - 30 saniyedir güncelleme yok")
                
                # SLAM node health check
                if not self.check_slam_health():
                    rospy.logwarn("⚠️ MRPT SLAM node sağlık sorunu tespit edildi")
                
            except Exception as e:
                rospy.logerr(f"❌ Monitoring hatası: {e}")
            
            time.sleep(5.0)  # 5 saniyede bir kontrol
    
    def check_slam_health(self) -> bool:
        """
        SLAM node sağlık kontrolü
        
        Returns:
            bool: Sağlık durumu
        """
        try:
            # Topic'lerin var olup olmadığını kontrol et
            topics = rospy.get_published_topics()
            slam_topics = [t[0] for t in topics if '/mrpt_rbpf_slam' in t[0]]
            
            required_topics = ['/mrpt_rbpf_slam/pose', '/mrpt_rbpf_slam/map']
            missing_topics = [t for t in required_topics if t not in slam_topics]
            
            if missing_topics:
                rospy.logwarn(f"⚠️ Eksik SLAM topic'leri: {missing_topics}")
                return False
            
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ Health check hatası: {e}")
            return False
    
    def shutdown(self):
        """
        Temizlik işlemleri
        """
        rospy.loginfo("🧹 MRPT SLAM integration kapatılıyor...")
        self.stop_slam()

def main():
    """
    Ana fonksiyon
    """
    try:
        # MRPT SLAM integration oluştur
        mrpt_integration = MRPTSLAMIntegration()
        
        # SLAM'i başlat
        if mrpt_integration.start_slam():
            rospy.loginfo("✅ MRPT SLAM integration başlatıldı")
            
            # Test navigation
            rospy.loginfo("⏱️ 10 saniye bekleyip test navigation yapılacak...")
            rospy.sleep(10.0)
            
            if mrpt_integration.is_slam_ready():
                rospy.loginfo("🧪 Test navigation: (2, 0) noktasına git")
                mrpt_integration.navigate_to_point(2.0, 0.0)
                
                rospy.sleep(5.0)
                
                rospy.loginfo("🧪 Test navigation: (0, 2) noktasına git") 
                mrpt_integration.navigate_to_point(0.0, 2.0)
            
            # ROS spin
            rospy.spin()
            
        else:
            rospy.logerr("❌ MRPT SLAM başlatılamadı")
    
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupted")
    
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Keyboard interrupt")
    
    finally:
        if 'mrpt_integration' in locals():
            mrpt_integration.shutdown()

if __name__ == "__main__":
    main()

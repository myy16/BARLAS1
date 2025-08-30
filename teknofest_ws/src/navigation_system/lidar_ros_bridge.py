#!/usr/bin/env python3
"""
TEKNOFEST LiDAR ROS Bridge Sistemi
RabbitMQ'dan LiDAR verilerini alıp ROS LaserScan topic'ine yayınlar
SLAM ve navigasyon için temel veri kaynağı
"""

import rospy
import pika
import json
import math
import numpy as np
import threading
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

class LidarROSBridge:
    """
    LiDAR verilerini RabbitMQ'dan alıp ROS'a aktaran bridge sistemi
    """
    
    def __init__(self):
        """
        LiDAR ROS Bridge başlatıcı
        """
        # ROS node başlat
        rospy.init_node('lidar_ros_bridge', anonymous=True)
        
        # RabbitMQ parametreleri
        self.rabbitmq_host = rospy.get_param('~rabbitmq_host', '192.168.1.4')
        self.rabbitmq_user = rospy.get_param('~rabbitmq_user', 'barlas')
        self.rabbitmq_pass = rospy.get_param('~rabbitmq_pass', 'barlas')
        self.rabbitmq_vhost = rospy.get_param('~rabbitmq_vhost', '/sensor_data')
        self.queue_name = rospy.get_param('~queue_name', 'lidar')
        
        # ROS parametreleri
        self.laser_topic = rospy.get_param('~laser_topic', '/scan')
        self.frame_id = rospy.get_param('~frame_id', 'laser_frame')
        
        # LiDAR parametreleri
        self.angle_min = rospy.get_param('~angle_min', -math.pi)
        self.angle_max = rospy.get_param('~angle_max', math.pi)
        self.angle_increment = rospy.get_param('~angle_increment', 0.0087)  # ~0.5°
        self.range_min = rospy.get_param('~range_min', 0.1)
        self.range_max = rospy.get_param('~range_max', 12.0)
        self.scan_time = rospy.get_param('~scan_time', 0.1)
        
        # ROS publisher
        self.laser_pub = rospy.Publisher(self.laser_topic, LaserScan, queue_size=10)
        
        # RabbitMQ bağlantısı
        self.connection = None
        self.channel = None
        self.is_connected = False
        
        # Threading
        self.running = False
        self.consumer_thread = None
        
        # İstatistikler
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.error_count = 0
        
        rospy.loginfo("🗺️ LiDAR ROS Bridge başlatıldı")
        rospy.loginfo(f"📡 RabbitMQ: {self.rabbitmq_host} - Queue: {self.queue_name}")
        rospy.loginfo(f"📊 ROS Topic: {self.laser_topic}")
    
    def setup_rabbitmq(self) -> bool:
        """
        RabbitMQ bağlantısını kur
        
        Returns:
            bool: Bağlantı durumu
        """
        try:
            # Bağlantı parametreleri
            credentials = pika.PlainCredentials(self.rabbitmq_user, self.rabbitmq_pass)
            connection_params = pika.ConnectionParameters(
                host=self.rabbitmq_host,
                virtual_host=self.rabbitmq_vhost,
                credentials=credentials,
                heartbeat=30,
                blocked_connection_timeout=60
            )
            
            # Bağlantı kur
            self.connection = pika.BlockingConnection(connection_params)
            self.channel = self.connection.channel()
            
            # Queue'yu declare et
            self.channel.queue_declare(queue=self.queue_name, durable=True)
            
            # Consumer callback ayarla
            self.channel.basic_consume(
                queue=self.queue_name,
                on_message_callback=self.lidar_callback,
                auto_ack=True
            )
            
            self.is_connected = True
            rospy.loginfo("✅ RabbitMQ bağlantısı başarılı")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ RabbitMQ bağlantı hatası: {e}")
            return False
    
    def lidar_callback(self, ch, method, properties, body):
        """
        RabbitMQ'dan gelen LiDAR verilerini işle
        
        Args:
            ch: RabbitMQ channel
            method: Delivery method
            properties: Message properties
            body: JSON veri
        """
        try:
            # JSON veriyi parse et
            data = json.loads(body)
            
            # LaserScan mesajı oluştur
            laser_msg = LaserScan()
            
            # Header ayarla
            laser_msg.header.stamp = rospy.Time.now()
            laser_msg.header.frame_id = self.frame_id
            
            # Config verilerini al
            config = data.get('config', {})
            points = data.get('points', [])
            
            # Açı parametrelerini güncelle
            laser_msg.angle_min = config.get('min_angle', self.angle_min)
            laser_msg.angle_max = config.get('max_angle', self.angle_max)
            laser_msg.angle_increment = config.get('angle_increment', self.angle_increment)
            
            # Zaman parametreleri
            laser_msg.scan_time = config.get('scan_time', self.scan_time)
            laser_msg.time_increment = config.get('time_increment', 0.0)
            
            # Menzil parametreleri
            laser_msg.range_min = config.get('min_range', self.range_min)
            laser_msg.range_max = config.get('max_range', self.range_max)
            
            # Points verilerini ranges array'ine dönüştür
            if points:
                ranges = self._process_lidar_points(points, laser_msg)
                laser_msg.ranges = ranges
            else:
                # Boş scan
                num_readings = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment)
                laser_msg.ranges = [float('inf')] * num_readings
            
            # Intensities boş bırak (şimdilik)
            laser_msg.intensities = []
            
            # ROS topic'ine yayınla
            self.laser_pub.publish(laser_msg)
            
            # İstatistikleri güncelle
            self.scan_count += 1
            current_time = time.time()
            
            if self.scan_count % 50 == 0:  # Her 50 scan'de bir log
                scan_rate = 50.0 / (current_time - self.last_scan_time)
                rospy.loginfo(f"📊 LiDAR: {len(points)} nokta, {scan_rate:.1f} Hz, Toplam: {self.scan_count}")
                self.last_scan_time = current_time
                
        except json.JSONDecodeError as e:
            self.error_count += 1
            rospy.logerr(f"❌ JSON parse hatası: {e}")
        except Exception as e:
            self.error_count += 1
            rospy.logerr(f"❌ LiDAR callback hatası: {e}")
    
    def _process_lidar_points(self, points, laser_msg):
        """
        LiDAR point verilerini LaserScan ranges formatına çevir
        
        Args:
            points: [(açı, mesafe), ...] formatında point listesi
            laser_msg: LaserScan mesajı (parametreler için)
            
        Returns:
            list: Ranges array'i
        """
        # Açı ve mesafe verilerini ayır
        angles = [point[0] for point in points]
        ranges = [point[1] for point in points]
        
        # Açı aralığını hesapla
        num_readings = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment)
        result_ranges = [float('inf')] * num_readings
        
        # Her point'i doğru index'e yerleştir
        for angle, range_val in zip(angles, ranges):
            # Açıyı normalize et
            if angle < laser_msg.angle_min or angle > laser_msg.angle_max:
                continue
                
            # Index hesapla
            index = int((angle - laser_msg.angle_min) / laser_msg.angle_increment)
            
            if 0 <= index < num_readings:
                # Range değerini kontrol et
                if laser_msg.range_min <= range_val <= laser_msg.range_max:
                    result_ranges[index] = range_val
                else:
                    result_ranges[index] = float('inf')
        
        return result_ranges
    
    def start_consuming(self):
        """
        RabbitMQ consumer'ı başlat
        """
        if not self.is_connected:
            rospy.logerr("❌ RabbitMQ bağlı değil!")
            return
        
        self.running = True
        self.consumer_thread = threading.Thread(target=self._consumer_loop, daemon=True)
        self.consumer_thread.start()
        
        rospy.loginfo("🚀 LiDAR data consuming başlatıldı")
    
    def _consumer_loop(self):
        """
        RabbitMQ consumer döngüsü
        """
        try:
            rospy.loginfo("📡 LiDAR verisi bekleniyor...")
            self.channel.start_consuming()
            
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Consumer durduruluyor...")
            self.stop_consuming()
        except Exception as e:
            rospy.logerr(f"❌ Consumer loop hatası: {e}")
            self.running = False
    
    def stop_consuming(self):
        """
        Consumer'ı durdur
        """
        self.running = False
        
        if self.channel and not self.channel.is_closed:
            self.channel.stop_consuming()
        
        if self.consumer_thread and self.consumer_thread.is_alive():
            self.consumer_thread.join(timeout=2.0)
        
        if self.connection and not self.connection.is_closed:
            self.connection.close()
        
        self.is_connected = False
        rospy.loginfo("🛑 LiDAR bridge durduruldu")
    
    def run(self):
        """
        Ana çalıştırma döngüsü
        """
        # RabbitMQ bağlantısını kur
        if not self.setup_rabbitmq():
            rospy.logerr("❌ RabbitMQ bağlantısı kurulamadı!")
            return
        
        # Consumer'ı başlat
        self.start_consuming()
        
        # Ana döngü
        rate = rospy.Rate(1)  # 1 Hz status
        
        try:
            while not rospy.is_shutdown() and self.running:
                # Bağlantı durumunu kontrol et
                if not self.is_connected:
                    rospy.logwarn("⚠️ RabbitMQ bağlantısı kesildi, yeniden bağlanmaya çalışılıyor...")
                    if self.setup_rabbitmq():
                        self.start_consuming()
                
                # Hata sayısı yüksekse uyar
                if self.error_count > 10:
                    rospy.logwarn(f"⚠️ Çok fazla hata: {self.error_count}")
                    self.error_count = 0
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 LiDAR bridge durduruluyor...")
            
        finally:
            self.stop_consuming()

def main():
    """
    Ana fonksiyon
    """
    try:
        bridge = LidarROSBridge()
        bridge.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupted")
    except Exception as e:
        rospy.logerr(f"❌ LiDAR bridge hatası: {e}")

if __name__ == "__main__":
    main()

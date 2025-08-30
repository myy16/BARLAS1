#!/usr/bin/env python3
"""
TEKNOFEST SLAM Navigasyon Sistemi
LiDAR verilerini kullanarak harita oluşturma ve otonom navigasyon
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Bool, String, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SLAMNavigationSystem:
    """
    SLAM tabanlı navigasyon sistemi
    """
    
    def __init__(self):
        """
        SLAM Navigation başlatıcı
        """
        # ROS node başlat
        rospy.init_node('slam_navigation_system', anonymous=True)
        
        # Parametreler
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser_frame')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        
        # Navigasyon parametreleri
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # rad/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)  # m
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.5)  # m
        
        # Robot durumu
        self.current_pose = None
        self.current_goal = None
        self.laser_data = None
        self.is_navigating = False
        self.navigation_mode = "manual"  # "manual", "waypoint", "autonomous"
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/navigation_markers', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/navigation_status', String, queue_size=1)
        
        # ROS Subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/navigation_cmd', String, self.navigation_command_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Path planning
        self.waypoints = []
        self.current_waypoint_index = 0
        
        rospy.loginfo("🗺️ SLAM Navigasyon Sistemi başlatıldı")
        rospy.loginfo(f"🚗 Robot frame: {self.robot_frame}")
        rospy.loginfo(f"📡 Laser frame: {self.laser_frame}")
    
    def laser_callback(self, msg):
        """
        Laser scan callback
        
        Args:
            msg: sensor_msgs/LaserScan
        """
        self.laser_data = msg
        
        # Obstacle detection için laser verilerini analiz et
        if self.is_navigating:
            self.check_obstacles()
    
    def odom_callback(self, msg):
        """
        Odometry callback
        
        Args:
            msg: nav_msgs/Odometry
        """
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """
        Goal callback (RViz'den gelen hedef)
        
        Args:
            msg: geometry_msgs/PoseStamped
        """
        rospy.loginfo(f"🎯 Yeni hedef alındı: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        self.current_goal = msg
        self.is_navigating = True
        self.navigation_mode = "waypoint"
    
    def navigation_command_callback(self, msg):
        """
        Navigasyon komut callback
        
        Args:
            msg: std_msgs/String
        """
        cmd = msg.data.lower()
        
        if cmd == "start_autonomous":
            self.start_autonomous_navigation()
        elif cmd == "stop":
            self.stop_navigation()
        elif cmd == "resume":
            self.resume_navigation()
        elif cmd.startswith("goto"):
            # "goto x,y" formatı
            try:
                coords = cmd.split()[1].split(',')
                x, y = float(coords[0]), float(coords[1])
                self.goto_position(x, y)
            except:
                rospy.logwarn("⚠️ Geçersiz goto komutu formatı: goto x,y")
        elif cmd == "return_home":
            self.goto_position(0, 0)
    
    def check_obstacles(self):
        """
        Laser verilerini kullanarak engel kontrolü
        """
        if not self.laser_data:
            return
        
        # Önde engel var mı kontrol et
        ranges = np.array(self.laser_data.ranges)
        angle_increment = self.laser_data.angle_increment
        angle_min = self.laser_data.angle_min
        
        # Ön kısım için açı aralığı (-45° - +45°)
        front_start_angle = -math.pi/4
        front_end_angle = math.pi/4
        
        front_indices = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if front_start_angle <= angle <= front_end_angle:
                if not math.isinf(r) and not math.isnan(r):
                    front_indices.append((i, r))
        
        if front_indices:
            min_distance = min([r for i, r in front_indices])
            
            if min_distance < self.obstacle_distance:
                rospy.logwarn(f"⚠️ Engel tespit edildi! Mesafe: {min_distance:.2f}m")
                self.handle_obstacle(min_distance)
    
    def handle_obstacle(self, distance):
        """
        Engel ile karşılaştığında çözüm stratejisi
        
        Args:
            distance: Engele olan mesafe
        """
        if distance < 0.3:  # Çok yakın engel
            # Dur ve geri git
            self.stop_robot()
            rospy.logwarn("🛑 Acil durak - çok yakın engel!")
            
        elif distance < self.obstacle_distance:
            # Yavaşla ve alternatif yol ara
            self.slow_down()
            # TODO: Obstacle avoidance algoritması
            rospy.loginfo("🐌 Yavaşlatılıyor - engel yakın")
    
    def goto_position(self, x, y, theta=0):
        """
        Belirtilen pozisyona git
        
        Args:
            x: X koordinatı
            y: Y koordinatı  
            theta: Yönelim (rad)
        """
        # PoseStamped mesajı oluştur
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = rospy.Time.now()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        
        # Quaternion'a çevir
        quat = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        
        self.current_goal = goal
        self.is_navigating = True
        self.navigation_mode = "waypoint"
        
        rospy.loginfo(f"🎯 Hedefe gidiliyor: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)")
    
    def start_autonomous_navigation(self):
        """
        Otonom navigasyonu başlat (parkur modunda)
        """
        rospy.loginfo("🚀 Otonom navigasyon başlatıldı")
        self.navigation_mode = "autonomous"
        self.is_navigating = True
        
        # TEKNOFEST parkuru için waypoint'ler
        # Bu waypoint'ler tabela tanıma ile güncellenecek
        self.waypoints = [
            (2.0, 0.0, 0),      # İlk checkpoint
            (4.0, 2.0, math.pi/2),  # Dönüş noktası
            (6.0, 2.0, 0),      # Düz yol
            (8.0, 0.0, -math.pi/2), # Son dönüş
            (10.0, 0.0, 0)      # Finish
        ]
        
        self.current_waypoint_index = 0
        self.goto_position(*self.waypoints[0])
    
    def simple_path_planning(self):
        """
        Basit path planning algoritması
        (Gelişmiş A* veya RRT* ile değiştirilebilir)
        """
        if not self.current_pose or not self.current_goal:
            return
        
        # Şu anda basit düz çizgi planlaması
        # Gelecekte obstacle avoidance ile geliştirilecek
        
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        
        # Başlangıç noktası
        start_pose = PoseStamped()
        start_pose.header = path.header
        start_pose.pose = self.current_pose
        path.poses.append(start_pose)
        
        # Hedef noktası
        path.poses.append(self.current_goal)
        
        # Path'i yayınla
        self.path_pub.publish(path)
    
    def navigate_to_goal(self):
        """
        Hedefe doğru navigasyon kontrolü
        """
        if not self.current_pose or not self.current_goal:
            return
        
        # Mevcut pozisyon
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_orientation = self.current_pose.orientation
        
        # Mevcut açı
        _, _, current_yaw = euler_from_quaternion([
            current_orientation.x, current_orientation.y,
            current_orientation.z, current_orientation.w
        ])
        
        # Hedef pozisyon
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        
        # Hedef mesafe ve açı
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Açı farkı
        angle_diff = angle_to_goal - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
        
        # Hedefe ulaştı mı?
        if distance < self.goal_tolerance:
            rospy.loginfo("✅ Hedefe ulaşıldı!")
            self.goal_reached()
            return
        
        # Hareket komutları hesapla
        cmd_vel = Twist()
        
        # Önce döner, sonra ilerler (basit strateji)
        if abs(angle_diff) > 0.1:  # 5.7 derece
            # Sadece dön
            cmd_vel.angular.z = self.max_angular_speed * np.sign(angle_diff) * min(abs(angle_diff), 1.0)
        else:
            # İlerle
            linear_speed = min(self.max_linear_speed, distance * 0.5)  # Yaklaştıkça yavaşla
            cmd_vel.linear.x = linear_speed
            
            # Fine-tuning için küçük açı düzeltmesi
            cmd_vel.angular.z = angle_diff * 0.5
        
        # Komutu yayınla
        self.cmd_vel_pub.publish(cmd_vel)
    
    def goal_reached(self):
        """
        Hedefe ulaşıldığında çağrılır
        """
        self.stop_robot()
        
        if self.navigation_mode == "autonomous" and self.waypoints:
            # Sonraki waypoint'e geç
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(f"📍 Sonraki waypoint'e geçiliyor: {self.current_waypoint_index}")
                self.goto_position(*self.waypoints[self.current_waypoint_index])
            else:
                rospy.loginfo("🏁 Tüm waypoint'ler tamamlandı!")
                self.is_navigating = False
                self.navigation_mode = "manual"
        else:
            self.is_navigating = False
            rospy.loginfo("🎯 Waypoint navigasyonu tamamlandı")
    
    def stop_robot(self):
        """
        Robotu durdur
        """
        cmd_vel = Twist()  # Tüm değerler 0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def slow_down(self):
        """
        Robotu yavaşlat
        """
        # Mevcut hızın yarısına düş
        # Bu method navigate_to_goal içinde implement edilecek
        pass
    
    def stop_navigation(self):
        """
        Navigasyonu durdur
        """
        self.is_navigating = False
        self.stop_robot()
        rospy.loginfo("🛑 Navigasyon durduruldu")
    
    def resume_navigation(self):
        """
        Navigasyonu devam ettir
        """
        if self.current_goal:
            self.is_navigating = True
            rospy.loginfo("▶️ Navigasyon devam ettirildi")
    
    def publish_status(self):
        """
        Navigasyon durumunu yayınla
        """
        status = {
            'mode': self.navigation_mode,
            'navigating': self.is_navigating,
            'waypoint_index': self.current_waypoint_index if self.waypoints else -1,
            'total_waypoints': len(self.waypoints)
        }
        
        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)
    
    def run(self):
        """
        Ana çalıştırma döngüsü
        """
        rospy.loginfo("🚀 SLAM Navigasyon sistemi çalışıyor")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            while not rospy.is_shutdown():
                # Navigasyon aktifse hedefe git
                if self.is_navigating and self.current_goal:
                    self.navigate_to_goal()
                    self.simple_path_planning()
                
                # Status yayınla
                self.publish_status()
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 SLAM Navigasyon durduruluyor...")
            
        finally:
            self.stop_robot()

def main():
    """
    Ana fonksiyon
    """
    try:
        nav_system = SLAMNavigationSystem()
        nav_system.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupted")
    except Exception as e:
        rospy.logerr(f"❌ SLAM Navigation hatası: {e}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TEKNOFEST Manuel Kontrol Sistemi
Joystick/Klavye ile Arduino motor kontrolü
ROS topic'leri üzerinden komut gönderimi
"""

import rospy
import pygame
import threading
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from arduino_controller import ArduinoController

class ManualControlSystem:
    """
    Manuel kontrol sistemi - Joystick ve klavye desteği
    """
    
    def __init__(self):
        """
        Manuel kontrol sistemi başlatıcı
        """
        # ROS node başlat
        rospy.init_node('manual_control_system', anonymous=True)
        
        # Arduino controller
        self.arduino = ArduinoController(port='COM3')
        self.arduino_connected = False
        
        # Joystick setup
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.joystick_connected = False
        
        # Control states
        self.control_mode = "keyboard"  # "keyboard" or "joystick"
        self.speed_multiplier = 1.0
        self.pan_tilt_speed = 10  # derece/adım
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.arduino_cmd_pub = rospy.Publisher('/arduino_cmd', String, queue_size=10)
        self.status_pub = rospy.Publisher('/manual_control_status', String, queue_size=1)
        
        # ROS Subscribers
        rospy.Subscriber('/joy', Joy, self.joystick_callback)
        rospy.Subscriber('/manual_cmd', String, self.manual_command_callback)
        
        # Threading
        self.running = False
        self.keyboard_thread = None
        
        rospy.loginfo("🎮 Manuel Kontrol Sistemi başlatıldı")
    
    def setup_connections(self) -> bool:
        """
        Arduino ve Joystick bağlantılarını kur
        """
        success = True
        
        # Arduino bağlantısı
        if self.arduino.connect():
            self.arduino_connected = True
            rospy.loginfo("✅ Arduino bağlantısı başarılı")
        else:
            rospy.logwarn("⚠️ Arduino bağlantısı başarısız - Simülasyon modu")
            success = False
        
        # Joystick kontrolü
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_connected = True
            self.control_mode = "joystick"
            rospy.loginfo(f"🎮 Joystick bulundu: {self.joystick.get_name()}")
        else:
            rospy.loginfo("⌨️ Joystick bulunamadı - Klavye modu aktif")
            self.control_mode = "keyboard"
        
        return success
    
    def joystick_callback(self, joy_msg):
        """
        ROS joy topic callback (eğer ROS joy_node kullanıyorsak)
        
        Args:
            joy_msg: sensor_msgs/Joy mesajı
        """
        if not self.joystick_connected or not self.arduino_connected:
            return
            
        try:
            # Xbox/PS4 controller layout
            left_stick_x = joy_msg.axes[0]  # Sağ-sol (-1 to 1)
            left_stick_y = joy_msg.axes[1]  # İleri-geri (-1 to 1)
            right_stick_x = joy_msg.axes[2] # Pan kontrolü
            right_stick_y = joy_msg.axes[3] # Tilt kontrolü
            
            # Threshold (dead zone)
            threshold = 0.1
            
            # Motor kontrolü
            if abs(left_stick_y) > threshold or abs(left_stick_x) > threshold:
                self._process_movement(left_stick_x, left_stick_y)
            else:
                self.arduino.stop()
            
            # Pan-tilt kontrolü
            if abs(right_stick_x) > threshold:
                if right_stick_x > 0:
                    self.arduino.pan_right()
                else:
                    self.arduino.pan_left()
                    
            if abs(right_stick_y) > threshold:
                if right_stick_y > 0:
                    self.arduino.tilt_up()
                else:
                    self.arduino.tilt_down()
            
            # Buton kontrolları
            if len(joy_msg.buttons) > 0:
                if joy_msg.buttons[0]:  # A button - Far
                    self.arduino.headlight_on()
                if joy_msg.buttons[1]:  # B button - Buzzer
                    self.arduino.buzzer_on()
                if joy_msg.buttons[2]:  # X button - Fren
                    self.arduino.brake_on()
                if joy_msg.buttons[3]:  # Y button - LED blink
                    self.arduino.led_blink_mode()
                    
        except Exception as e:
            rospy.logerr(f"❌ Joystick callback hatası: {e}")
    
    def _process_movement(self, x_axis: float, y_axis: float):
        """
        Joystick ekseni verilerini motor komutlarına çevir
        
        Args:
            x_axis: Sağ-sol ekseni (-1 to 1)
            y_axis: İleri-geri ekseni (-1 to 1)
        """
        # Basit 8-yönlü hareket
        if y_axis > 0.5:  # İleri
            if x_axis > 0.5:
                self.arduino.forward_right()
            elif x_axis < -0.5:
                self.arduino.forward_left()
            else:
                self.arduino.move_forward()
                
        elif y_axis < -0.5:  # Geri
            if x_axis > 0.5:
                self.arduino.backward_right()
            elif x_axis < -0.5:
                self.arduino.backward_left()
            else:
                self.arduino.move_backward()
                
        elif x_axis > 0.5:  # Sadece sağ
            self.arduino.turn_right()
        elif x_axis < -0.5:  # Sadece sol
            self.arduino.turn_left()
        else:
            self.arduino.stop()
    
    def manual_command_callback(self, msg):
        """
        Manuel komut callback (/manual_cmd topic)
        
        Args:
            msg: std_msgs/String - komut string'i
        """
        cmd = msg.data.lower()
        
        if not self.arduino_connected:
            rospy.logwarn("⚠️ Arduino bağlı değil!")
            return
        
        # Hareket komutları
        movement_commands = {
            'forward': self.arduino.move_forward,
            'backward': self.arduino.move_backward,
            'left': self.arduino.turn_left,
            'right': self.arduino.turn_right,
            'forward_left': self.arduino.forward_left,
            'forward_right': self.arduino.forward_right,
            'backward_left': self.arduino.backward_left,
            'backward_right': self.arduino.backward_right,
            'stop': self.arduino.stop
        }
        
        # Pan-tilt komutları
        pantilt_commands = {
            'pan_left': self.arduino.pan_left,
            'pan_right': self.arduino.pan_right,
            'tilt_up': self.arduino.tilt_up,
            'tilt_down': self.arduino.tilt_down
        }
        
        # Aux komutları
        aux_commands = {
            'brake_on': self.arduino.brake_on,
            'brake_off': self.arduino.brake_off,
            'headlight_on': self.arduino.headlight_on,
            'headlight_off': self.arduino.headlight_off,
            'taillight_on': self.arduino.taillight_on,
            'taillight_off': self.arduino.taillight_off,
            'buzzer_on': self.arduino.buzzer_on,
            'buzzer_off': self.arduino.buzzer_off,
            'led_blink': self.arduino.led_blink_mode
        }
        
        # Komut çalıştır
        all_commands = {**movement_commands, **pantilt_commands, **aux_commands}
        
        if cmd in all_commands:
            all_commands[cmd]()
            rospy.loginfo(f"✅ Komut çalıştırıldı: {cmd}")
        else:
            rospy.logwarn(f"⚠️ Bilinmeyen komut: {cmd}")
    
    def start_keyboard_control(self):
        """
        Klavye kontrol thread'ini başlat
        """
        if self.control_mode == "keyboard":
            self.running = True
            self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self.keyboard_thread.start()
            rospy.loginfo("⌨️ Klavye kontrolü aktif")
    
    def _keyboard_loop(self):
        """
        Klavye kontrol döngüsü (pygame ile)
        """
        pygame.display.set_mode((400, 300))
        pygame.display.set_caption("TEKNOFEST Manuel Kontrol")
        
        clock = pygame.time.Clock()
        
        rospy.loginfo("⌨️ Klavye Kontrolleri:")
        rospy.loginfo("W/A/S/D: Hareket | Q/E: Pan | R/F: Tilt | Space: Stop")
        rospy.loginfo("H: Far | B: Buzzer | N: Fren | L: LED")
        
        while self.running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    
            if not self.arduino_connected:
                time.sleep(0.1)
                continue
                
            # Klavye durumunu kontrol et
            keys = pygame.key.get_pressed()
            
            # Hareket kontrolü
            if keys[pygame.K_w] and keys[pygame.K_a]:  # W+A
                self.arduino.forward_left()
            elif keys[pygame.K_w] and keys[pygame.K_d]:  # W+D
                self.arduino.forward_right()
            elif keys[pygame.K_s] and keys[pygame.K_a]:  # S+A
                self.arduino.backward_left()
            elif keys[pygame.K_s] and keys[pygame.K_d]:  # S+D
                self.arduino.backward_right()
            elif keys[pygame.K_w]:  # W
                self.arduino.move_forward()
            elif keys[pygame.K_s]:  # S
                self.arduino.move_backward()
            elif keys[pygame.K_a]:  # A
                self.arduino.turn_left()
            elif keys[pygame.K_d]:  # D
                self.arduino.turn_right()
            elif keys[pygame.K_SPACE]:  # Space
                self.arduino.stop()
                
            # Pan-tilt kontrolü
            if keys[pygame.K_q]:  # Q - Pan left
                self.arduino.pan_left()
                time.sleep(0.1)  # Servo hızını kontrol et
            elif keys[pygame.K_e]:  # E - Pan right
                self.arduino.pan_right()
                time.sleep(0.1)
                
            if keys[pygame.K_r]:  # R - Tilt up
                self.arduino.tilt_up()
                time.sleep(0.1)
            elif keys[pygame.K_f]:  # F - Tilt down
                self.arduino.tilt_down()
                time.sleep(0.1)
                
            # Aux kontrolü
            if keys[pygame.K_h]:  # H - Headlight
                self.arduino.headlight_on()
            if keys[pygame.K_b]:  # B - Buzzer
                self.arduino.buzzer_on()
            if keys[pygame.K_n]:  # N - Brake
                self.arduino.brake_on()
            if keys[pygame.K_l]:  # L - LED blink
                self.arduino.led_blink_mode()
            
            clock.tick(20)  # 20 FPS
    
    def publish_status(self):
        """
        Sistem durumu yayınla
        """
        status = {
            'control_mode': self.control_mode,
            'arduino_connected': self.arduino_connected,
            'joystick_connected': self.joystick_connected,
            'motor_state': self.arduino.motor_state if self.arduino_connected else "disconnected"
        }
        
        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)
    
    def run(self):
        """
        Ana çalıştırma döngüsü
        """
        rospy.loginfo("🚀 Manuel Kontrol Sistemi çalışıyor")
        
        # Bağlantıları kur
        self.setup_connections()
        
        # Arduino monitoring başlat
        if self.arduino_connected:
            self.arduino.start_monitoring()
        
        # Klavye kontrolünü başlat
        self.start_keyboard_control()
        
        # Ana döngü
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            while not rospy.is_shutdown():
                # Status yayınla
                self.publish_status()
                
                # Sensor verilerini kontrol et
                if self.arduino_connected:
                    sensor_data = self.arduino.get_sensor_data()
                    
                    # Düşük batarya uyarısı
                    if sensor_data['battery_voltage'] > 0 and sensor_data['battery_voltage'] < 7.0:
                        rospy.logwarn(f"⚠️ Düşük batarya: {sensor_data['battery_voltage']:.1f}V")
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Manuel kontrol sistemi durduruluyor...")
            
        finally:
            self.shutdown()
    
    def shutdown(self):
        """
        Sistemi güvenli şekilde kapat
        """
        rospy.loginfo("🛑 Sistem kapatılıyor...")
        
        # Thread'leri durdur
        self.running = False
        
        if self.keyboard_thread and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=2.0)
        
        # Arduino'yu durdur ve bağlantıyı kes
        if self.arduino_connected:
            self.arduino.stop()
        
        # Pygame'i kapat
        pygame.quit()
        
        rospy.loginfo("✅ Güvenli kapatma tamamlandı")

def main():
    """
    Ana fonksiyon
    """
    try:
        control_system = ManualControlSystem()
        control_system.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Manuel kontrol hatası: {e}")

if __name__ == "__main__":
    main()

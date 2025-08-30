#!/usr/bin/env python3
"""
TEKNOFEST Sensor Controller
Raspberry Pi sensör verilerini serial port üzerinden alır ve ROS topic'lerine publish eder
"""

import serial
import json
import threading
import time
from typing import Dict, Any, Optional
import logging

class SensorController:
    """
    Raspberry Pi'deki tüm sensörlerin verilerini yöneten ana sınıf
    """
    
    def __init__(self, port: str = 'COM3', baudrate: int = 115200):
        """
        Sensor Controller başlatıcı
        
        Args:
            port: Serial port adı (Windows: COMx, Linux: /dev/ttyUSBx)
            baudrate: İletişim hızı
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = False
        
        # Sensor verileri
        self.sensor_data = {
            'camera': {'frame_count': 0, 'fps': 0},
            'lidar': {'ranges': [], 'angle_min': 0, 'angle_max': 0},
            'imu': {'accel': {'x': 0, 'y': 0, 'z': 0}, 'gyro': {'x': 0, 'y': 0, 'z': 0}},
            'ultrasonic': {'distance_cm': 0},
            'environment': {'temperature': 0, 'humidity': 0},
            'encoders': {'left_ticks': 0, 'right_ticks': 0}
        }
        
        # Thread'ler
        self.read_thread = None
        self.heartbeat_thread = None
        
        # Logging setup
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def initialize(self):
        """
        Sensor controller'ı başlat
        
        Returns:
            bool: Başlatma durumu
        """
        try:
            success = self.connect()
            if success:
                self.start()
            return success
        except Exception as e:
            self.logger.error(f"❌ Initialization hatası: {e}")
            return False
        
    def connect(self) -> bool:
        """
        Raspberry Pi ile serial bağlantı kur
        
        Returns:
            bool: Bağlantı durumu
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            
            time.sleep(2)  # Arduino reset bekle
            
            # Bağlantı testi
            test_cmd = {"cmd": "ping"}
            self._send_command(test_cmd)
            
            response = self._read_response(timeout=3.0)
            if response and response.get('status') == 'pong':
                self.is_connected = True
                self.logger.info(f"✅ Raspberry Pi bağlantısı başarılı: {self.port}")
                return True
                
        except Exception as e:
            self.logger.error(f"❌ Raspberry Pi bağlantı hatası: {e}")
            
        return False
    
    def start_reading(self):
        """
        Sensor verilerini okuma thread'lerini başlat
        """
        if not self.is_connected:
            self.logger.error("❌ Bağlantı kurmadan okuma başlatılamaz!")
            return
            
        self.is_running = True
        
        # Ana veri okuma thread'i
        self.read_thread = threading.Thread(target=self._sensor_read_loop, daemon=True)
        self.read_thread.start()
        
        # Heartbeat thread'i
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        
        self.logger.info("🚀 Sensor okuma başlatıldı")
    
    def _sensor_read_loop(self):
        """
        Sürekli sensor verisi okuma döngüsü
        """
        while self.is_running and self.is_connected:
            try:
                # Tüm sensör verilerini iste
                cmd = {"cmd": "get_all_sensors"}
                self._send_command(cmd)
                
                response = self._read_response(timeout=0.1)
                if response and response.get('type') == 'sensor_data':
                    self._update_sensor_data(response.get('data', {}))
                    
            except Exception as e:
                self.logger.error(f"❌ Sensor okuma hatası: {e}")
                time.sleep(0.1)
                
            time.sleep(0.05)  # 20Hz okuma hızı
    
    def _heartbeat_loop(self):
        """
        Bağlantı kontrolü için heartbeat döngüsü
        """
        while self.is_running:
            try:
                cmd = {"cmd": "heartbeat"}
                self._send_command(cmd)
                
                response = self._read_response(timeout=2.0)
                if not response or response.get('status') != 'alive':
                    self.logger.warning("⚠️ Heartbeat yanıtı alınamadı")
                    
            except Exception as e:
                self.logger.error(f"❌ Heartbeat hatası: {e}")
                
            time.sleep(1.0)  # 1Hz heartbeat
    
    def _send_command(self, command: Dict[str, Any]) -> bool:
        """
        Raspberry Pi'ye komut gönder
        
        Args:
            command: Gönderilecek komut dictionary
            
        Returns:
            bool: Gönderim durumu
        """
        try:
            if self.serial_connection and self.serial_connection.is_open:
                cmd_json = json.dumps(command) + '\n'
                self.serial_connection.write(cmd_json.encode('utf-8'))
                return True
        except Exception as e:
            self.logger.error(f"❌ Komut gönderme hatası: {e}")
            
        return False
    
    def _read_response(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        Raspberry Pi'den yanıt oku
        
        Args:
            timeout: Okuma zaman aşımı
            
        Returns:
            Dict veya None
        """
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.timeout = timeout
                line = self.serial_connection.readline().decode('utf-8').strip()
                
                if line:
                    return json.loads(line)
                    
        except json.JSONDecodeError as e:
            self.logger.error(f"❌ JSON decode hatası: {e}")
        except Exception as e:
            self.logger.error(f"❌ Yanıt okuma hatası: {e}")
            
        return None
    
    def _update_sensor_data(self, new_data: Dict[str, Any]):
        """
        Gelen sensor verilerini güncelle
        
        Args:
            new_data: Raspberry Pi'den gelen sensor verileri
        """
        try:
            # Kamera verileri
            if 'camera' in new_data:
                self.sensor_data['camera'].update(new_data['camera'])
                
            # LiDAR verileri
            if 'lidar' in new_data:
                self.sensor_data['lidar'].update(new_data['lidar'])
                
            # IMU verileri
            if 'imu' in new_data:
                self.sensor_data['imu'].update(new_data['imu'])
                
            # Ultrasonik veriler
            if 'ultrasonic' in new_data:
                self.sensor_data['ultrasonic'].update(new_data['ultrasonic'])
                
            # Çevresel sensörler
            if 'environment' in new_data:
                self.sensor_data['environment'].update(new_data['environment'])
                
            # Encoder verileri
            if 'encoders' in new_data:
                self.sensor_data['encoders'].update(new_data['encoders'])
                
        except Exception as e:
            self.logger.error(f"❌ Sensor veri güncelleme hatası: {e}")
    
    # Sensor veri getter metodları
    def get_camera_data(self) -> Dict[str, Any]:
        """Kamera verilerini al"""
        return self.sensor_data['camera'].copy()
    
    def get_lidar_data(self) -> Dict[str, Any]:
        """LiDAR verilerini al"""
        return self.sensor_data['lidar'].copy()
    
    def get_imu_data(self) -> Dict[str, Any]:
        """IMU verilerini al"""
        return self.sensor_data['imu'].copy()
    
    def get_ultrasonic_distance(self) -> float:
        """Ultrasonik mesafe al"""
        return self.sensor_data['ultrasonic']['distance_cm']
    
    def get_environment_data(self) -> Dict[str, Any]:
        """Çevresel sensor verilerini al"""
        return self.sensor_data['environment'].copy()
    
    def get_encoder_data(self) -> Dict[str, Any]:
        """Encoder verilerini al"""
        return self.sensor_data['encoders'].copy()
    
    def get_all_sensor_data(self) -> Dict[str, Any]:
        """Tüm sensor verilerini al"""
        return self.sensor_data.copy()
    
    def stop(self):
        """
        Sensor okuma durdur ve bağlantıyı kapat
        """
        self.is_running = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
            
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2.0)
            
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        self.is_connected = False
        self.logger.info("🛑 Sensor controller durduruldu")

# Test fonksiyonu
def test_sensor_controller():
    """
    SensorController test fonksiyonu
    """
    print("🧪 TEKNOFEST Sensor Controller Test")
    print("=" * 40)
    
    # Controller oluştur
    controller = SensorController(port='COM3')
    
    try:
        # Bağlantı kur
        if controller.connect():
            print("✅ Raspberry Pi bağlantısı başarılı!")
            
            # Okumayı başlat
            controller.start_reading()
            
            # 10 saniye test et
            for i in range(10):
                time.sleep(1)
                
                # Sensor verilerini yazdır
                imu = controller.get_imu_data()
                distance = controller.get_ultrasonic_distance()
                env = controller.get_environment_data()
                
                print(f"Saniye {i+1}:")
                print(f"  IMU: Accel({imu['accel']['x']:.2f}, {imu['accel']['y']:.2f}, {imu['accel']['z']:.2f})")
                print(f"  Ultrasonik: {distance:.1f} cm")
                print(f"  Çevre: {env['temperature']:.1f}°C, {env['humidity']:.1f}%")
                
        else:
            print("❌ Raspberry Pi bağlantısı başarısız!")
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruldu")
        
    finally:
        controller.stop()

if __name__ == "__main__":
    test_sensor_controller()

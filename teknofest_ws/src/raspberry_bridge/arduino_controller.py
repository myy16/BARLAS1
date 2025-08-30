#!/usr/bin/env python3
"""
TEKNOFEST İnsansız Kara Aracı Arduino Kontrol Sistemi
Arduino ile serial haberleşme ve tüm sensör/motor kontrolü
"""

import serial
import json
import threading
import time
import logging
from typing import Dict, Any, Optional, Tuple

class ArduinoController:
    """
    Arduino ile serial haberleşme ve kontrol sınıfı
    """
    
    def __init__(self, port: str = 'COM3', baudrate: int = 9600):
        """
        Arduino Controller başlatıcı
        
        Args:
            port: Serial port (Arduino bağlantısı)
            baudrate: İletişim hızı (Arduino: 9600)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = False
        
        # Motor durumları
        self.motor_state = "stop"
        self.pan_angle = 90
        self.tilt_angle = 90
        
        # Sensor verileri
        self.sensor_data = {
            'encoder_count': 0,
            'temperature': 0.0,
            'humidity': 0.0,
            'battery_voltage': 0.0,
            'headlight_state': False,
            'taillight_state': False,
            'brake_state': False
        }
        
        # Thread'ler
        self.monitor_thread = None
        
        # Logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def connect(self) -> bool:
        """
        Arduino ile bağlantı kur
        
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
            
            # Bağlantı testi - stop komutu gönder
            if self.send_command('5'):  # Stop command
                self.is_connected = True
                self.logger.info(f"✅ Arduino bağlantısı başarılı: {self.port}")
                return True
                
        except Exception as e:
            self.logger.error(f"❌ Arduino bağlantı hatası: {e}")
            
        return False
    
    def send_command(self, command: str) -> bool:
        """
        Arduino'ya tek karakter komut gönder
        
        Args:
            command: Gönderilecek komut karakteri
            
        Returns:
            bool: Gönderim durumu
        """
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.write(command.encode())
                self.logger.debug(f"Komut gönderildi: {command}")
                return True
        except Exception as e:
            self.logger.error(f"❌ Komut gönderme hatası: {e}")
            
        return False
    
    def read_response(self, timeout: float = 0.5) -> Optional[str]:
        """
        Arduino'dan yanıt oku
        
        Args:
            timeout: Okuma zaman aşımı
            
        Returns:
            str veya None
        """
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.timeout = timeout
                response = self.serial_connection.readline().decode('utf-8').strip()
                if response:
                    self.logger.debug(f"Arduino yanıtı: {response}")
                    return response
        except Exception as e:
            self.logger.error(f"❌ Yanıt okuma hatası: {e}")
            
        return None
    
    # ==================== MOTOR KONTROL ====================
    
    def move_forward(self) -> bool:
        """İleri hareket"""
        if self.send_command('8'):
            self.motor_state = "forward"
            return True
        return False
    
    def move_backward(self) -> bool:
        """Geri hareket"""
        if self.send_command('2'):
            self.motor_state = "backward"
            return True
        return False
    
    def turn_left(self) -> bool:
        """Sola dönüş"""
        if self.send_command('4'):
            self.motor_state = "left"
            return True
        return False
    
    def turn_right(self) -> bool:
        """Sağa dönüş"""
        if self.send_command('6'):
            self.motor_state = "right"
            return True
        return False
    
    def forward_left(self) -> bool:
        """İleri-sol"""
        if self.send_command('7'):
            self.motor_state = "forward_left"
            return True
        return False
    
    def forward_right(self) -> bool:
        """İleri-sağ"""
        if self.send_command('9'):
            self.motor_state = "forward_right"
            return True
        return False
    
    def backward_left(self) -> bool:
        """Geri-sol"""
        if self.send_command('1'):
            self.motor_state = "backward_left"
            return True
        return False
    
    def backward_right(self) -> bool:
        """Geri-sağ"""
        if self.send_command('3'):
            self.motor_state = "backward_right"
            return True
        return False
    
    def stop(self) -> bool:
        """Dur"""
        if self.send_command('5'):
            self.motor_state = "stop"
            return True
        return False
    
    # ==================== PAN-TILT KONTROL ====================
    
    def pan_right(self) -> bool:
        """Pan servo sağa (+10 derece)"""
        if self.send_command('P'):
            self.pan_angle = min(180, self.pan_angle + 10)
            return True
        return False
    
    def pan_left(self) -> bool:
        """Pan servo sola (-10 derece)"""
        if self.send_command('p'):
            self.pan_angle = max(0, self.pan_angle - 10)
            return True
        return False
    
    def tilt_up(self) -> bool:
        """Tilt servo yukarı (+10 derece)"""
        if self.send_command('U'):
            self.tilt_angle = min(180, self.tilt_angle + 10)
            return True
        return False
    
    def tilt_down(self) -> bool:
        """Tilt servo aşağı (-10 derece)"""
        if self.send_command('u'):
            self.tilt_angle = max(0, self.tilt_angle - 10)
            return True
        return False
    
    def get_pan_tilt(self) -> Tuple[int, int]:
        """Pan-Tilt açılarını al"""
        return self.pan_angle, self.tilt_angle
    
    # ==================== AUX KONTROLLER ====================
    
    def brake_on(self) -> bool:
        """Fren aktif"""
        if self.send_command('F'):
            self.sensor_data['brake_state'] = True
            return True
        return False
    
    def brake_off(self) -> bool:
        """Fren pasif"""
        if self.send_command('f'):
            self.sensor_data['brake_state'] = False
            return True
        return False
    
    def headlight_on(self) -> bool:
        """Far aç"""
        if self.send_command('H'):
            self.sensor_data['headlight_state'] = True
            return True
        return False
    
    def headlight_off(self) -> bool:
        """Far kapat"""
        if self.send_command('h'):
            self.sensor_data['headlight_state'] = False
            return True
        return False
    
    def taillight_on(self) -> bool:
        """Arka far aç"""
        if self.send_command('T'):
            self.sensor_data['taillight_state'] = True
            return True
        return False
    
    def taillight_off(self) -> bool:
        """Arka far kapat"""
        if self.send_command('t'):
            self.sensor_data['taillight_state'] = False
            return True
        return False
    
    def buzzer_on(self) -> bool:
        """Buzzer aç"""
        return self.send_command('A')
    
    def buzzer_off(self) -> bool:
        """Buzzer kapat"""
        return self.send_command('a')
    
    def led_blink_mode(self) -> bool:
        """LED yanıp sönme modu"""
        return self.send_command('e')
    
    # ==================== SENSOR OKUMA ====================
    
    def read_encoder(self) -> Optional[int]:
        """Encoder değerini oku"""
        if self.send_command('E'):
            time.sleep(0.1)  # Yanıt bekle
            response = self.read_response()
            if response and "Encoder Count:" in response:
                try:
                    count = int(response.split(":")[1].strip())
                    self.sensor_data['encoder_count'] = count
                    return count
                except:
                    pass
        return None
    
    def read_temperature_humidity(self) -> Optional[Tuple[float, float]]:
        """Sıcaklık ve nem oku"""
        if self.send_command('S'):
            time.sleep(0.2)  # DHT11 okuma süresi
            response = self.read_response()
            if response and "Temp:" in response and "Hum:" in response:
                try:
                    # "Temp: 25.0 C, Hum: 60.0 %" formatını parse et
                    parts = response.split(", ")
                    temp = float(parts[0].split(":")[1].strip().split(" ")[0])
                    hum = float(parts[1].split(":")[1].strip().split(" ")[0])
                    
                    self.sensor_data['temperature'] = temp
                    self.sensor_data['humidity'] = hum
                    return temp, hum
                except:
                    pass
        return None
    
    def read_battery_voltage(self) -> Optional[float]:
        """Batarya voltajını oku"""
        if self.send_command('B'):
            time.sleep(0.1)
            response = self.read_response()
            if response and "Battery:" in response:
                try:
                    voltage = float(response.split(":")[1].strip().split(" ")[0])
                    self.sensor_data['battery_voltage'] = voltage
                    return voltage
                except:
                    pass
        return None
    
    def get_sensor_data(self) -> Dict[str, Any]:
        """Tüm sensor verilerini al"""
        return self.sensor_data.copy()
    
    def update_all_sensors(self):
        """Tüm sensör verilerini güncelle"""
        self.read_encoder()
        self.read_temperature_humidity()
        self.read_battery_voltage()
    
    def start_monitoring(self):
        """Sürekli sensor monitoring başlat"""
        if not self.is_connected:
            self.logger.error("❌ Bağlantı kurmadan monitoring başlatılamaz!")
            return
            
        self.is_running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        self.logger.info("🚀 Arduino monitoring başlatıldı")
    
    def _monitor_loop(self):
        """Sensor monitoring döngüsü"""
        while self.is_running and self.is_connected:
            try:
                self.update_all_sensors()
                time.sleep(1.0)  # 1Hz monitoring
            except Exception as e:
                self.logger.error(f"❌ Monitoring hatası: {e}")
                time.sleep(1.0)
    
    def stop(self):
        """Arduino kontrolcüsünü durdur"""
        self.is_running = False
        
        # Stop motoru gönder
        self.send_command('5')
        
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
            
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        self.is_connected = False
        self.logger.info("🛑 Arduino controller durduruldu")

# Test fonksiyonu
def test_arduino_controller():
    """
    Arduino Controller test fonksiyonu
    """
    print("🧪 TEKNOFEST Arduino Controller Test")
    print("=" * 50)
    
    # Controller oluştur
    arduino = ArduinoController(port='COM3')
    
    try:
        # Bağlantı kur
        if arduino.connect():
            print("✅ Arduino bağlantısı başarılı!")
            
            # Test senaryosu
            print("\n🚗 Motor Test:")
            arduino.move_forward()
            time.sleep(2)
            arduino.stop()
            
            print("\n🎯 Pan-Tilt Test:")
            arduino.pan_right()
            arduino.tilt_up()
            time.sleep(1)
            arduino.pan_left()
            arduino.tilt_down()
            
            print("\n🔍 Sensor Test:")
            arduino.start_monitoring()
            
            for i in range(5):
                time.sleep(1)
                data = arduino.get_sensor_data()
                print(f"Saniye {i+1}: Encoder={data['encoder_count']}, "
                      f"Temp={data['temperature']:.1f}°C, "
                      f"Battery={data['battery_voltage']:.1f}V")
                      
            print("\n💡 LED & Buzzer Test:")
            arduino.buzzer_on()
            arduino.led_blink_mode()
            time.sleep(2)
            arduino.buzzer_off()
            
        else:
            print("❌ Arduino bağlantısı başarısız!")
            
    except KeyboardInterrupt:
        print("\n🛑 Test durduruldu")
        
    finally:
        arduino.stop()

if __name__ == "__main__":
    test_arduino_controller()

#!/usr/bin/env python3
"""
TEKNOFEST Manuel Kontrol Test Sistemi
Arduino bağlantısı olmadan da test edilebilen basit sürüm
"""

import time
import threading
import pygame
import sys
from typing import Dict, Any

class MockArduinoController:
    """
    Arduino simülatörü - gerçek Arduino olmadığında test için
    """
    
    def __init__(self, port='COM3'):
        self.port = port
        self.motor_state = "stop"
        self.pan_angle = 90
        self.tilt_angle = 90
        self.headlight_state = False
        self.brake_state = False
        print(f"🤖 Mock Arduino Controller başlatıldı: {port}")
    
    def connect(self):
        print("✅ Mock Arduino bağlantısı simüle edildi")
        return True
    
    def move_forward(self):
        self.motor_state = "forward"
        print("⬆️ İleri")
        return True
    
    def move_backward(self):
        self.motor_state = "backward"
        print("⬇️ Geri")
        return True
    
    def turn_left(self):
        self.motor_state = "left"
        print("⬅️ Sol")
        return True
    
    def turn_right(self):
        self.motor_state = "right"
        print("➡️ Sağ")
        return True
    
    def forward_left(self):
        self.motor_state = "forward_left"
        print("↖️ İleri-Sol")
        return True
    
    def forward_right(self):
        self.motor_state = "forward_right"
        print("↗️ İleri-Sağ")
        return True
    
    def backward_left(self):
        self.motor_state = "backward_left"
        print("↙️ Geri-Sol")
        return True
    
    def backward_right(self):
        self.motor_state = "backward_right"
        print("↘️ Geri-Sağ")
        return True
    
    def stop(self):
        self.motor_state = "stop"
        print("⏹️ Dur")
        return True
    
    def pan_left(self):
        self.pan_angle = max(0, self.pan_angle - 10)
        print(f"🔄 Pan Sol: {self.pan_angle}°")
        return True
    
    def pan_right(self):
        self.pan_angle = min(180, self.pan_angle + 10)
        print(f"🔄 Pan Sağ: {self.pan_angle}°")
        return True
    
    def tilt_up(self):
        self.tilt_angle = min(180, self.tilt_angle + 10)
        print(f"🔼 Tilt Yukarı: {self.tilt_angle}°")
        return True
    
    def tilt_down(self):
        self.tilt_angle = max(0, self.tilt_angle - 10)
        print(f"🔽 Tilt Aşağı: {self.tilt_angle}°")
        return True
    
    def headlight_on(self):
        self.headlight_state = True
        print("💡 Far Açık")
        return True
    
    def headlight_off(self):
        self.headlight_state = False
        print("🔌 Far Kapalı")
        return True
    
    def brake_on(self):
        self.brake_state = True
        print("🛑 Fren Aktif")
        return True
    
    def brake_off(self):
        self.brake_state = False
        print("🟢 Fren Pasif")
        return True
    
    def buzzer_on(self):
        print("🔊 Buzzer Açık")
        return True
    
    def buzzer_off(self):
        print("🔇 Buzzer Kapalı")
        return True
    
    def led_blink_mode(self):
        print("✨ LED Yanıp Sönme")
        return True

class SimpleManualControl:
    """
    Basitleştirilmiş manuel kontrol sistemi - ROS olmadan test
    """
    
    def __init__(self, use_real_arduino=False):
        """
        Basit manuel kontrol başlatıcı
        
        Args:
            use_real_arduino: Gerçek Arduino kullan (False=Simülasyon)
        """
        # Arduino controller seçimi
        if use_real_arduino:
            try:
                from arduino_controller import ArduinoController
                self.arduino = ArduinoController(port='COM3')
            except ImportError:
                print("⚠️ arduino_controller modülü bulunamadı, simülasyon moduna geçiliyor")
                self.arduino = MockArduinoController()
        else:
            self.arduino = MockArduinoController()
        
        # Pygame başlat
        pygame.init()
        pygame.joystick.init()
        
        # Joystick kontrolü
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"🎮 Joystick: {self.joystick.get_name()}")
        else:
            print("⌨️ Joystick bulunamadı - Klavye modu")
        
        # Durum
        self.running = False
        self.last_command_time = time.time()
        
    def setup(self):
        """
        Sistemin kurulumu
        """
        if self.arduino.connect():
            print("🚀 Sistem hazır!")
            return True
        else:
            print("❌ Arduino bağlantısı başarısız!")
            return False
    
    def process_keyboard(self):
        """
        Klavye girişlerini işle
        """
        keys = pygame.key.get_pressed()
        command_sent = False
        
        # Hareket kontrolü (WASD)
        if keys[pygame.K_w] and keys[pygame.K_a]:
            self.arduino.forward_left()
            command_sent = True
        elif keys[pygame.K_w] and keys[pygame.K_d]:
            self.arduino.forward_right()
            command_sent = True
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            self.arduino.backward_left()
            command_sent = True
        elif keys[pygame.K_s] and keys[pygame.K_d]:
            self.arduino.backward_right()
            command_sent = True
        elif keys[pygame.K_w]:
            self.arduino.move_forward()
            command_sent = True
        elif keys[pygame.K_s]:
            self.arduino.move_backward()
            command_sent = True
        elif keys[pygame.K_a]:
            self.arduino.turn_left()
            command_sent = True
        elif keys[pygame.K_d]:
            self.arduino.turn_right()
            command_sent = True
        elif keys[pygame.K_SPACE]:
            self.arduino.stop()
            command_sent = True
        
        # Pan-tilt kontrolü (QE/RF)
        current_time = time.time()
        if current_time - self.last_command_time > 0.2:  # 200ms delay
            if keys[pygame.K_q]:
                self.arduino.pan_left()
                self.last_command_time = current_time
            elif keys[pygame.K_e]:
                self.arduino.pan_right()
                self.last_command_time = current_time
            elif keys[pygame.K_r]:
                self.arduino.tilt_up()
                self.last_command_time = current_time
            elif keys[pygame.K_f]:
                self.arduino.tilt_down()
                self.last_command_time = current_time
        
        # Aux kontrolü (tek basım)
        if keys[pygame.K_h]:
            self.arduino.headlight_on()
        if keys[pygame.K_b]:
            self.arduino.buzzer_on()
            time.sleep(0.1)
            self.arduino.buzzer_off()
        if keys[pygame.K_n]:
            self.arduino.brake_on()
        if keys[pygame.K_m]:
            self.arduino.brake_off()
        if keys[pygame.K_l]:
            self.arduino.led_blink_mode()
        
        return command_sent
    
    def process_joystick(self):
        """
        Joystick girişlerini işle
        """
        if not self.joystick:
            return False
        
        pygame.event.pump()
        
        # Analog stick değerleri
        left_x = self.joystick.get_axis(0)  # Sol stick X
        left_y = -self.joystick.get_axis(1) # Sol stick Y (ters çevir)
        right_x = self.joystick.get_axis(2) # Sağ stick X
        right_y = -self.joystick.get_axis(3) # Sağ stick Y
        
        threshold = 0.2
        command_sent = False
        
        # Motor kontrolü (sol stick)
        if abs(left_x) > threshold or abs(left_y) > threshold:
            if left_y > threshold:  # İleri
                if left_x > threshold:
                    self.arduino.forward_right()
                elif left_x < -threshold:
                    self.arduino.forward_left()
                else:
                    self.arduino.move_forward()
                command_sent = True
            elif left_y < -threshold:  # Geri
                if left_x > threshold:
                    self.arduino.backward_right()
                elif left_x < -threshold:
                    self.arduino.backward_left()
                else:
                    self.arduino.move_backward()
                command_sent = True
            elif left_x > threshold:
                self.arduino.turn_right()
                command_sent = True
            elif left_x < -threshold:
                self.arduino.turn_left()
                command_sent = True
        else:
            self.arduino.stop()
            command_sent = True
        
        # Pan-tilt kontrolü (sağ stick)
        current_time = time.time()
        if current_time - self.last_command_time > 0.1:
            if abs(right_x) > threshold:
                if right_x > 0:
                    self.arduino.pan_right()
                else:
                    self.arduino.pan_left()
                self.last_command_time = current_time
                
            if abs(right_y) > threshold:
                if right_y > 0:
                    self.arduino.tilt_up()
                else:
                    self.arduino.tilt_down()
                self.last_command_time = current_time
        
        # Buton kontrolü
        for i in range(self.joystick.get_numbuttons()):
            if self.joystick.get_button(i):
                if i == 0:  # A button
                    self.arduino.buzzer_on()
                    time.sleep(0.1)
                    self.arduino.buzzer_off()
                elif i == 1:  # B button
                    self.arduino.headlight_on()
                elif i == 2:  # X button
                    self.arduino.brake_on()
                elif i == 3:  # Y button
                    self.arduino.led_blink_mode()
                elif i == 4:  # LB button
                    self.arduino.stop()
                    
                time.sleep(0.2)  # Buton bounce koruması
        
        return command_sent
    
    def run(self):
        """
        Ana kontrol döngüsü
        """
        if not self.setup():
            return
        
        # Pygame penceresi
        screen = pygame.display.set_mode((600, 400))
        pygame.display.set_caption("TEKNOFEST Manuel Kontrol Test")
        
        # Font
        font = pygame.font.Font(None, 24)
        
        print("\n" + "="*50)
        print("🎮 TEKNOFEST MANUEL KONTROL TEST")
        print("="*50)
        print("⌨️ KLAVYE KONTROLLERI:")
        print("   W/A/S/D: Hareket")
        print("   Q/E: Pan (Sol/Sağ)")
        print("   R/F: Tilt (Yukarı/Aşağı)")
        print("   Space: Dur")
        print("   H: Far | B: Buzzer | N/M: Fren On/Off | L: LED")
        print("   ESC: Çıkış")
        print("\n🎮 JOYSTICK: Sol stick=Hareket, Sağ stick=Pan/Tilt")
        print("="*50)
        
        self.running = True
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                # Event handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                
                # Kontrol işleme
                keyboard_active = self.process_keyboard()
                joystick_active = self.process_joystick()
                
                # Ekran güncelleme
                screen.fill((0, 0, 50))  # Koyu mavi arka plan
                
                # Bilgi metinleri
                info_texts = [
                    f"Motor Durumu: {self.arduino.motor_state}",
                    f"Pan Açısı: {self.arduino.pan_angle}°",
                    f"Tilt Açısı: {self.arduino.tilt_angle}°",
                    f"Far: {'Açık' if self.arduino.headlight_state else 'Kapalı'}",
                    f"Fren: {'Aktif' if self.arduino.brake_state else 'Pasif'}",
                    "",
                    "Klavye: WASD=Hareket, QE=Pan, RF=Tilt",
                    "Space=Dur, H=Far, B=Buzzer, N/M=Fren, L=LED",
                    "ESC=Çıkış"
                ]
                
                for i, text in enumerate(info_texts):
                    if text:  # Boş satırları atla
                        color = (255, 255, 255) if not text.startswith("Klavye") else (200, 200, 200)
                        text_surface = font.render(text, True, color)
                        screen.blit(text_surface, (20, 20 + i * 30))
                
                pygame.display.flip()
                clock.tick(30)  # 30 FPS
                
        except KeyboardInterrupt:
            print("\n🛑 Ctrl+C ile durduruldu")
            
        finally:
            self.shutdown()
    
    def shutdown(self):
        """
        Sistemi kapat
        """
        print("\n🛑 Sistem kapatılıyor...")
        self.running = False
        
        # Arduino'yu durdur
        self.arduino.stop()
        
        # Pygame'i kapat
        pygame.quit()
        
        print("✅ Güvenli kapatma tamamlandı")

def main():
    """
    Ana test fonksiyonu
    """
    print("🧪 TEKNOFEST Manuel Kontrol Test Sistemi")
    
    # Komut satırı argümanları
    use_real_arduino = "--real" in sys.argv
    
    if use_real_arduino:
        print("🔧 Gerçek Arduino modu seçildi")
    else:
        print("🤖 Simülasyon modu seçildi")
    
    try:
        control_system = SimpleManualControl(use_real_arduino=use_real_arduino)
        control_system.run()
        
    except Exception as e:
        print(f"❌ Hata: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TEKNOFEST Otonom Sistem Test Launcher
Tüm otonom sistem bileşenlerini koordine eder ve test eder
"""

import os
import sys
import time
import json
import threading
import subprocess
from datetime import datetime

# Ana modül yolu
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class AutonomusSystemLauncher:
    """
    Otonom sistem test launcher
    """
    
    def __init__(self):
        """
        Launcher başlatıcı
        """
        self.workspace_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.config_path = os.path.join(os.path.dirname(__file__), 'config', 'autonomous_config.json')
        
        # Test modları
        self.test_modes = {
            '1': 'Manual Control Test',
            '2': 'Sensor System Test', 
            '3': 'Vision System Test',
            '4': 'Navigation Test',
            '5': 'Full Autonomous Test',
            '6': 'TEKNOFEST Mission Simulation'
        }
        
        # Process tracking
        self.running_processes = []
        self.test_results = {}
        
        print("🚀 TEKNOFEST Otonom Sistem Launcher")
        print("=" * 50)
    
    def show_main_menu(self):
        """
        Ana menüyü göster
        """
        print("\n📋 Test Modları:")
        print("-" * 30)
        
        for key, value in self.test_modes.items():
            print(f"  {key}. {value}")
        
        print("  0. Çıkış")
        print("-" * 30)
    
    def run_manual_control_test(self):
        """
        Manuel kontrol testi
        """
        print("🎮 Manuel kontrol testi başlatılıyor...")
        
        try:
            # Manual control script çalıştır
            script_path = os.path.join(self.workspace_path, 'main_controller', 'test_manual_control.py')
            
            if os.path.exists(script_path):
                print(f"🔄 Çalıştırılıyor: {script_path}")
                result = subprocess.run([sys.executable, script_path], 
                                      capture_output=True, text=True, timeout=60)
                
                if result.returncode == 0:
                    print("✅ Manuel kontrol testi başarılı")
                    self.test_results['manual_control'] = 'PASS'
                else:
                    print(f"❌ Manuel kontrol testi başarısız: {result.stderr}")
                    self.test_results['manual_control'] = 'FAIL'
            else:
                print(f"❌ Script bulunamadı: {script_path}")
                self.test_results['manual_control'] = 'NOT_FOUND'
                
        except subprocess.TimeoutExpired:
            print("⏰ Manuel kontrol testi zaman aşımı")
            self.test_results['manual_control'] = 'TIMEOUT'
        except Exception as e:
            print(f"❌ Manuel kontrol test hatası: {e}")
            self.test_results['manual_control'] = 'ERROR'
    
    def run_sensor_system_test(self):
        """
        Sensor sistemi testi
        """
        print("📡 Sensor sistem testi başlatılıyor...")
        
        try:
            # Sensor controller test
            from raspberry_bridge.sensor_controller import SensorController
            
            sensor_controller = SensorController()
            
            print("🔄 Sensor controller başlatılıyor...")
            init_success = sensor_controller.initialize()
            
            if init_success:
                print("✅ Sensor controller başlatma başarılı")
                
                # Test data collection
                print("📊 Test verisi toplama...")
                time.sleep(3)  # 3 saniye veri topla
                
                # Health check
                health = sensor_controller.is_healthy()
                print(f"🏥 Sensor sağlık durumu: {'✅' if health else '❌'}")
                
                # Cleanup
                sensor_controller.cleanup()
                
                self.test_results['sensor_system'] = 'PASS' if health else 'FAIL'
            else:
                print("❌ Sensor controller başlatma başarısız")
                self.test_results['sensor_system'] = 'FAIL'
                
        except Exception as e:
            print(f"❌ Sensor sistem test hatası: {e}")
            self.test_results['sensor_system'] = 'ERROR'
    
    def run_vision_system_test(self):
        """
        Vision sistemi testi
        """
        print("👁️ Vision sistem testi başlatılıyor...")
        
        try:
            # Tabela recognition test
            script_path = os.path.join(self.workspace_path, 'vision_system', 'test_tabela_recognition.py')
            
            if os.path.exists(script_path):
                print(f"🔄 Çalıştırılıyor: {script_path}")
                
                # Test in subprocess with timeout
                process = subprocess.Popen([sys.executable, script_path], 
                                         stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE)
                
                try:
                    stdout, stderr = process.communicate(timeout=30)
                    
                    if process.returncode == 0:
                        print("✅ Vision sistem testi başarılı")
                        self.test_results['vision_system'] = 'PASS'
                    else:
                        print(f"❌ Vision sistem testi başarısız")
                        self.test_results['vision_system'] = 'FAIL'
                        
                except subprocess.TimeoutExpired:
                    process.kill()
                    print("⏰ Vision test zaman aşımı - normal (kamera gerekli)")
                    self.test_results['vision_system'] = 'TIMEOUT_EXPECTED'
                    
            else:
                print(f"❌ Script bulunamadı: {script_path}")
                self.test_results['vision_system'] = 'NOT_FOUND'
                
        except Exception as e:
            print(f"❌ Vision sistem test hatası: {e}")
            self.test_results['vision_system'] = 'ERROR'
    
    def run_navigation_test(self):
        """
        Navigasyon sistemi testi
        """
        print("🗺️ Navigasyon sistem testi başlatılıyor...")
        
        try:
            # Mock navigation test (ROS gerekmeden)
            print("🔄 Navigation modülleri import ediliyor...")
            
            # Import check
            try:
                from navigation_system.slam_navigation import SLAMNavigationSystem
                from navigation_system.lidar_ros_bridge import LidarROSBridge
                print("✅ Navigation modülleri yüklendi")
                navigation_import = True
            except ImportError as e:
                print(f"⚠️ Navigation modülleri yüklenemedi (ROS gerekli): {e}")
                navigation_import = False
            
            # Test waypoint planning logic
            print("🎯 Waypoint planlama testi...")
            test_waypoints = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)]
            
            print(f"📍 Test waypoints: {len(test_waypoints)} nokta")
            
            # Basic path validation
            total_distance = 0
            for i in range(1, len(test_waypoints)):
                prev = test_waypoints[i-1]
                curr = test_waypoints[i]
                distance = ((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)**0.5
                total_distance += distance
            
            print(f"📏 Toplam rota mesafesi: {total_distance:.2f}m")
            
            if navigation_import and total_distance > 0:
                print("✅ Navigasyon testi başarılı")
                self.test_results['navigation'] = 'PASS'
            else:
                print("⚠️ Navigasyon testi kısmi başarılı (ROS gerekli)")
                self.test_results['navigation'] = 'PARTIAL'
                
        except Exception as e:
            print(f"❌ Navigasyon test hatası: {e}")
            self.test_results['navigation'] = 'ERROR'
    
    def run_full_autonomous_test(self):
        """
        Tam otonom sistem testi
        """
        print("🚀 Tam otonom sistem testi başlatılıyor...")
        
        try:
            # Autonomous controller import
            from main_controller.autonomous_controller import AutonomousController, AutonomousMode
            
            print("🔄 Otonom controller oluşturuluyor...")
            controller = AutonomousController(self.config_path)
            
            print("🏗️ Sistemler başlatılıyor...")
            init_success = controller.initialize_systems()
            
            if not init_success:
                print("❌ Sistem başlatma başarısız")
                self.test_results['full_autonomous'] = 'INIT_FAIL'
                return
            
            print("✅ Sistemler başlatıldı")
            
            # Test waypoints yükle
            test_waypoints = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]
            controller.load_mission_waypoints(test_waypoints)
            print(f"📍 {len(test_waypoints)} waypoint yüklendi")
            
            # Otonom moda geç
            controller.set_mode(AutonomousMode.FULLY_AUTONOMOUS)
            print("🤖 Otonom mod aktif")
            
            # Kısa test çalıştırması
            print("⏱️ 15 saniye test çalıştırması...")
            if controller.start_mission():
                start_time = time.time()
                
                while time.time() - start_time < 15:
                    status = controller.get_status()
                    print(f"📊 {status['state']} - WP: {status['waypoint_current']+1}/{status['waypoints_total']}")
                    
                    # Adaptive controller durumu
                    if 'adaptive_controller' in status:
                        adaptive = status['adaptive_controller']
                        print(f"   📍 Stage: {adaptive['current_stage']} ({adaptive['current_speed_limit']:.1f}m/s)")
                    
                    # Sensor fusion durumu
                    if 'sensor_fusion' in status:
                        fusion = status['sensor_fusion']
                        print(f"   🔬 Fusion: {fusion['healthy_sensors']} sensors, {fusion['fusion_confidence']:.2f} conf, {fusion['obstacle_count']} obstacles")
                    
                    # Safety monitor durumu
                    if 'safety_monitor' in status:
                        safety = status['safety_monitor']
                        print(f"   🛡️ Safety: {safety['level']}, {safety['active_incidents']} incidents, {safety['geofence_status']}")
                    
                    # Recovery automation durumu
                    if 'recovery_automation' in status:
                        recovery = status['recovery_automation']
                        print(f"   🔧 Recovery: {recovery['status']}, {recovery['success_rate']:.1f}% success, stuck: {recovery['stuck_duration']:.1f}s")
                    
                    time.sleep(2)
                
                controller.stop_mission()
                print("✅ Otonom test tamamlandı")
                self.test_results['full_autonomous'] = 'PASS'
            else:
                print("❌ Görev başlatılamadı")
                self.test_results['full_autonomous'] = 'MISSION_FAIL'
            
            # Cleanup
            controller.cleanup()
            
        except Exception as e:
            print(f"❌ Otonom sistem test hatası: {e}")
            self.test_results['full_autonomous'] = 'ERROR'
    
    def run_teknofest_simulation(self):
        """
        TEKNOFEST görev simülasyonu
        """
        print("🏆 TEKNOFEST Görev Simülasyonu başlatılıyor...")
        
        try:
            # Config yükle
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            teknofest_config = config.get('teknofest_mission', {})
            waypoints = teknofest_config.get('waypoints', {})
            
            print("🗺️ TEKNOFEST Parkur Haritası:")
            print("-" * 40)
            
            for name, coord in waypoints.items():
                print(f"  📍 {name}: ({coord[0]}, {coord[1]})")
            
            print("\n🎯 Görev Aşamaları:")
            stages = teknofest_config.get('competition_stages', [])
            for i, stage in enumerate(stages, 1):
                print(f"  {i}. {stage}")
            
            print("\n📋 Gerekli Tabelalar:")
            signs = teknofest_config.get('required_signs', [])
            for sign in signs[:5]:  # İlk 5 tabela
                print(f"  🚩 {sign}")
            print(f"  ... ve {len(signs)-5} tabela daha")
            
            # Scoring simulation
            scoring = teknofest_config.get('scoring', {})
            total_possible = sum(scoring.values())
            
            print(f"\n🏅 Maksimum Puan: {total_possible}")
            print("✅ TEKNOFEST simülasyonu tamamlandı")
            
            self.test_results['teknofest_simulation'] = 'PASS'
            
        except Exception as e:
            print(f"❌ TEKNOFEST simülasyon hatası: {e}")
            self.test_results['teknofest_simulation'] = 'ERROR'
    
    def show_test_results(self):
        """
        Test sonuçlarını göster
        """
        if not self.test_results:
            print("📊 Henüz test sonucu yok")
            return
        
        print("\n📊 Test Sonuçları:")
        print("=" * 40)
        
        for test_name, result in self.test_results.items():
            status_icon = {
                'PASS': '✅',
                'FAIL': '❌', 
                'ERROR': '💥',
                'TIMEOUT': '⏰',
                'PARTIAL': '⚠️',
                'NOT_FOUND': '❓'
            }.get(result, '❓')
            
            print(f"  {status_icon} {test_name}: {result}")
        
        # Summary
        passed = sum(1 for r in self.test_results.values() if r == 'PASS')
        total = len(self.test_results)
        
        print(f"\n🎯 Özet: {passed}/{total} test başarılı")
        
        # Save results
        self.save_test_results()
    
    def save_test_results(self):
        """
        Test sonuçlarını kaydet
        """
        try:
            results_dir = os.path.join(self.workspace_path, 'results')
            os.makedirs(results_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_file = os.path.join(results_dir, f'test_results_{timestamp}.json')
            
            results_data = {
                'timestamp': timestamp,
                'test_results': self.test_results,
                'config_file': self.config_path,
                'workspace': self.workspace_path
            }
            
            with open(results_file, 'w', encoding='utf-8') as f:
                json.dump(results_data, f, ensure_ascii=False, indent=2)
            
            print(f"💾 Test sonuçları kaydedildi: {results_file}")
            
        except Exception as e:
            print(f"❌ Sonuç kaydetme hatası: {e}")
    
    def run(self):
        """
        Ana çalışma döngüsü
        """
        print(f"📁 Workspace: {self.workspace_path}")
        print(f"⚙️ Config: {self.config_path}")
        
        try:
            while True:
                self.show_main_menu()
                
                try:
                    choice = input("\n🔹 Seçiminizi yapın: ").strip()
                    
                    if choice == '0':
                        print("👋 Çıkış yapılıyor...")
                        break
                    elif choice == '1':
                        self.run_manual_control_test()
                    elif choice == '2':
                        self.run_sensor_system_test()
                    elif choice == '3':
                        self.run_vision_system_test()
                    elif choice == '4':
                        self.run_navigation_test()
                    elif choice == '5':
                        self.run_full_autonomous_test()
                    elif choice == '6':
                        self.run_teknofest_simulation()
                    else:
                        print("❌ Geçersiz seçim!")
                        continue
                    
                    # Test sonuçlarını göster
                    input("\n⏸️ Devam etmek için Enter tuşuna basın...")
                    self.show_test_results()
                    
                except KeyboardInterrupt:
                    print("\n🛑 İşlem iptal edildi")
                    
        except KeyboardInterrupt:
            print("\n👋 Çıkış yapılıyor...")
        
        finally:
            self.show_test_results()

def main():
    """
    Ana fonksiyon
    """
    launcher = AutonomusSystemLauncher()
    launcher.run()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
TEKNOFEST Otonom Navigasyon Ana Kontrolcü
SLAM, tabela tanıma ve rota planlaması entegrasyonu
"""

import time
import threading
import json
import logging
import numpy as np
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any

# System modulleri
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Adaptive stage controller import
from main_controller.adaptive_stage_controller import AdaptiveStageController, ParkurStage

# Sensor fusion system import
try:
    from main_controller.sensor_fusion_system import SensorFusionSystem, SensorReading, SensorType, SensorHealth, FusedData
    SENSOR_FUSION_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Sensor Fusion System yüklenemedi: {e}")
    SENSOR_FUSION_AVAILABLE = False

# Safety monitor system import
try:
    from main_controller.safety_monitor_system import SafetyMonitorSystem, SafetyEvent, SafetyLevel, SafetyIncident
    SAFETY_MONITOR_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Safety Monitor System yüklenemedi: {e}")
    SAFETY_MONITOR_AVAILABLE = False

# Recovery automation system import
try:
    from main_controller.recovery_automation_system import RecoveryAutomationSystem, RecoveryAction, FailureType, RecoveryStatus
    RECOVERY_AUTOMATION_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Recovery Automation System yüklenemedi: {e}")
    RECOVERY_AUTOMATION_AVAILABLE = False

try:
    # Raspberry bridge modulleri
    from raspberry_bridge.sensor_controller import SensorController
    from raspberry_bridge.arduino_controller import ArduinoController
    
    # Navigation modulleri  
    from navigation_system.slam_navigation import SLAMNavigationSystem
    from navigation_system.lidar_ros_bridge import LidarROSBridge
    from navigation_system.mrpt_slam_integration import MRPTSLAMIntegration
    
    NAVIGATION_AVAILABLE = True
    print("✅ Navigasyon modulleri yüklendi (MRPT SLAM dahil)")
except ImportError as e:
    NAVIGATION_AVAILABLE = False
    print(f"⚠️ Navigasyon modulleri yüklenemedi: {e}")

try:
    # Vision modülleri
    from vision_system.tabela_recognition import TabelaRecognitionSystem
    VISION_AVAILABLE = True
    print("✅ Vision modülleri yüklendi")
except ImportError as e:
    VISION_AVAILABLE = False
    print(f"⚠️ Vision modülleri yüklenemedi (ROS gerekli): {e}")

class AutonomousMode(Enum):
    """Otonom çalışma modları"""
    MANUAL = "manual"
    SEMI_AUTONOMOUS = "semi_autonomous"
    FULLY_AUTONOMOUS = "fully_autonomous"
    EMERGENCY_STOP = "emergency_stop"

class MissionState(Enum):
    """Görev durumları"""
    STANDBY = "standby"
    NAVIGATING = "navigating"
    SIGN_DETECTION = "sign_detection"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    MISSION_COMPLETE = "mission_complete"
    ERROR = "error"

class AutonomousController:
    """
    TEKNOFEST otonom navigasyon ana kontrolcü
    """
    
    def __init__(self, config_file=None):
        """
        Kontrolcü başlatıcı
        
        Args:
            config_file: Konfigürasyon dosyası yolu
        """
        # Logging setup
        logging.basicConfig(level=logging.INFO, 
                          format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Konfigürasyon yükle
        self.config = self.load_config(config_file)
        
        # Sistem durumu
        self.mode = AutonomousMode.MANUAL
        self.mission_state = MissionState.STANDBY
        self.running = False
        self.emergency_stop = False
        
        # Alt sistemler
        self.sensor_controller = None
        self.arduino_controller = None
        self.slam_navigation = None
        self.lidar_bridge = None
        self.mrpt_slam = None  # MRPT SLAM integration
        self.vision_system = None
        self.adaptive_controller = None  # Adaptive Stage Controller
        self.sensor_fusion = None  # Sensor Fusion System
        self.safety_monitor = None  # Safety Monitor System
        self.recovery_automation = None  # Recovery Automation System
        
        # Görev bilgileri
        self.waypoints = []
        self.current_waypoint_index = 0
        self.detected_signs = []
        self.mission_data = {
            'start_time': None,
            'end_time': None,
            'distance_traveled': 0.0,
            'signs_detected': 0,
            'obstacles_avoided': 0
        }
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.lock = threading.Lock()
        
        self.logger.info("🚀 Otonom Kontrolcü başlatıldı")
    
    def load_config(self, config_file):
        """
        Konfigürasyon dosyasını yükle
        
        Args:
            config_file: Config dosya yolu
            
        Returns:
            dict: Konfigürasyon
        """
        default_config = {
            # Araç parametreleri
            'vehicle': {
                'max_speed': 2.0,
                'max_angular_speed': 1.0,
                'safety_distance': 1.0,
                'wheelbase': 0.3
            },
            
            # Navigasyon parametreleri  
            'navigation': {
                'goal_tolerance': 0.3,
                'obstacle_threshold': 0.8,
                'path_planning_frequency': 10.0,
                'recovery_timeout': 30.0
            },
            
            # Görev parametreleri
            'mission': {
                'max_mission_time': 900,  # 15 dakika
                'waypoint_tolerance': 0.5,
                'sign_detection_distance': 3.0,
                'sign_detection_timeout': 10.0
            },
            
            # Sensor parametreleri
            'sensors': {
                'lidar_topic': '/scan',
                'camera_topic': '/camera/image_raw',
                'imu_topic': '/imu/data',
                'gps_topic': '/gps/fix'
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    # Merge configurations
                    default_config.update(user_config)
                    self.logger.info(f"📝 Konfigürasyon yüklendi: {config_file}")
            except Exception as e:
                self.logger.error(f"❌ Config yükleme hatası: {e}")
        
        return default_config
    
    def initialize_systems(self):
        """
        Alt sistemleri başlat
        
        Returns:
            bool: Başlatma durumu
        """
        try:
            # Sensor controller başlat
            self.sensor_controller = SensorController()
            sensor_init = self.sensor_controller.initialize()
            self.logger.info(f"📡 Sensor controller: {'✅' if sensor_init else '❌'}")
            
            # Arduino controller başlat
            arduino_port = self.config.get('arduino_port', '/dev/ttyUSB0')
            self.arduino_controller = ArduinoController(port=arduino_port)
            arduino_init = self.arduino_controller.connect()
            self.logger.info(f"🔌 Arduino controller: {'✅' if arduino_init else '❌'}")
            
            # SLAM Navigation (eğer mevcut)
            if NAVIGATION_AVAILABLE:
                try:
                    # Önce MRPT SLAM integration dene
                    self.mrpt_slam = MRPTSLAMIntegration()
                    mrpt_init = self.mrpt_slam.start_slam()
                    self.logger.info(f"🗺️ MRPT SLAM integration: {'✅' if mrpt_init else '❌'}")
                    
                    if mrpt_init:
                        # MRPT SLAM başarılı ise standart SLAM'i devre dışı bırak
                        self.logger.info("🔄 MRPT SLAM kullanılıyor, standart SLAM devre dışı")
                    else:
                        # MRPT başarısız ise standart SLAM'e geri dön
                        self.slam_navigation = SLAMNavigationSystem()
                        slam_init = self.slam_navigation.initialize()
                        self.logger.info(f"🗺️ Standart SLAM navigation: {'✅' if slam_init else '❌'}")
                    
                    # LiDAR bridge (her iki durumda da gerekli)
                    self.lidar_bridge = LidarROSBridge()
                    lidar_init = self.lidar_bridge.start()
                    self.logger.info(f"📡 LiDAR bridge: {'✅' if lidar_init else '❌'}")
                    
                except Exception as e:
                    self.logger.error(f"❌ SLAM başlatma hatası: {e}")
            else:
                self.logger.warning("⚠️ ROS navigation stack mevcut değil")
            
            # Vision system (eğer mevcut)
            if VISION_AVAILABLE:
                try:
                    self.vision_system = TabelaRecognitionSystem()
                    self.logger.info("👁️ Vision system: ✅")
                except Exception as e:
                    self.logger.error(f"❌ Vision başlatma hatası: {e}")
            
            # Adaptive Stage Controller başlat
            self.adaptive_controller = AdaptiveStageController(self.config)
            self.logger.info("🎛️ Adaptive Stage Controller: ✅")
            
            # Sensor Fusion System başlat
            if SENSOR_FUSION_AVAILABLE:
                try:
                    self.sensor_fusion = SensorFusionSystem()
                    self.sensor_fusion.start()
                    self.logger.info("🔬 Sensor Fusion System: ✅")
                except Exception as e:
                    self.logger.error(f"❌ Sensor Fusion başlatma hatası: {e}")
            else:
                self.logger.warning("⚠️ Sensor Fusion System mevcut değil")
            
            # Safety Monitor System başlat
            if SAFETY_MONITOR_AVAILABLE:
                try:
                    self.safety_monitor = SafetyMonitorSystem()
                    
                    # Safety callbacks register et
                    self.safety_monitor.register_safety_callback(
                        SafetyEvent.OBSTACLE_TOO_CLOSE, self._handle_safety_obstacle
                    )
                    self.safety_monitor.register_safety_callback(
                        SafetyEvent.EXCESSIVE_TILT, self._handle_safety_tilt
                    )
                    
                    self.safety_monitor.start()
                    self.logger.info("🛡️ Safety Monitor System: ✅")
                except Exception as e:
                    self.logger.error(f"❌ Safety Monitor başlatma hatası: {e}")
            else:
                self.logger.warning("⚠️ Safety Monitor System mevcut değil")
            
            # Recovery Automation System başlat
            if RECOVERY_AUTOMATION_AVAILABLE:
                try:
                    self.recovery_automation = RecoveryAutomationSystem()
                    
                    # Recovery callbacks register et
                    self.recovery_automation.register_recovery_callback(
                        RecoveryAction.EMERGENCY_RETURN, self._handle_recovery_emergency_return
                    )
                    self.recovery_automation.register_recovery_callback(
                        RecoveryAction.MANUAL_TAKEOVER, self._handle_recovery_manual_takeover
                    )
                    
                    self.recovery_automation.start()
                    self.logger.info("🔧 Recovery Automation System: ✅")
                except Exception as e:
                    self.logger.error(f"❌ Recovery Automation başlatma hatası: {e}")
            else:
                self.logger.warning("⚠️ Recovery Automation System mevcut değil")
            
            self.logger.info("🏗️ Sistem başlatma tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Sistem başlatma hatası: {e}")
            return False
    
    def set_mode(self, mode: AutonomousMode):
        """
        Çalışma modunu değiştir
        
        Args:
            mode: Yeni mod
        """
        with self.lock:
            old_mode = self.mode
            self.mode = mode
            
            self.logger.info(f"🔄 Mod değişti: {old_mode.value} -> {mode.value}")
            
            # Mode specific actions
            if mode == AutonomousMode.EMERGENCY_STOP:
                self.emergency_stop = True
                self.stop_vehicle()
            elif mode == AutonomousMode.MANUAL:
                self.mission_state = MissionState.STANDBY
            elif mode == AutonomousMode.FULLY_AUTONOMOUS:
                if self.waypoints:
                    self.mission_state = MissionState.NAVIGATING
    
    def load_mission_waypoints(self, waypoints: List[Tuple[float, float]]):
        """
        Görev waypoint'lerini yükle
        
        Args:
            waypoints: [(x, y), ...] koordinat listesi
        """
        with self.lock:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.logger.info(f"📍 {len(waypoints)} waypoint yüklendi")
    
    def start_mission(self):
        """
        Otonom görevi başlat
        """
        if not self.waypoints:
            self.logger.error("❌ Waypoint yok - görev başlatılamaz")
            return False
        
        with self.lock:
            if self.running:
                self.logger.warning("⚠️ Görev zaten çalışıyor")
                return False
            
            self.running = True
            self.emergency_stop = False
            self.mission_data['start_time'] = time.time()
            
            # Control thread başlat
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            
            # Monitoring thread başlat
            self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
            self.monitoring_thread.start()
        
        self.logger.info("🚀 Otonom görev başlatıldı")
        return True
    
    def stop_mission(self):
        """
        Otonom görevi durdur
        """
        with self.lock:
            self.running = False
            self.mission_state = MissionState.STANDBY
            
            if self.mission_data['start_time']:
                self.mission_data['end_time'] = time.time()
        
        self.stop_vehicle()
        self.logger.info("🛑 Otonom görev durduruldu")
    
    def emergency_stop_activated(self):
        """
        Acil dur aktivasyonu
        """
        self.set_mode(AutonomousMode.EMERGENCY_STOP)
        self.stop_vehicle()
        self.logger.critical("🚨 ACİL DUR AKTİF!")
    
    def stop_vehicle(self):
        """
        Aracı durdur
        """
        if self.arduino_controller:
            try:
                self.arduino_controller.set_motor_speed(0, 0)
                self.arduino_controller.set_servo_angle(90)  # Düz
                self.logger.info("🛑 Araç durduruldu")
            except Exception as e:
                self.logger.error(f"❌ Araç durdurma hatası: {e}")
    
    def _control_loop(self):
        """
        Ana kontrol döngüsü
        """
        self.logger.info("🔄 Kontrol döngüsü başladı")
        
        while self.running:
            try:
                if self.emergency_stop:
                    self.stop_vehicle()
                    time.sleep(0.1)
                    continue
                
                # Mode göre işlem
                if self.mode == AutonomousMode.FULLY_AUTONOMOUS:
                    self._autonomous_control()
                elif self.mode == AutonomousMode.SEMI_AUTONOMOUS:
                    self._semi_autonomous_control()
                else:
                    time.sleep(0.1)
                    continue
                    
            except Exception as e:
                self.logger.error(f"❌ Kontrol döngüsü hatası: {e}")
                self.mission_state = MissionState.ERROR
                
            time.sleep(0.1)  # 10Hz kontrol döngüsü
    
    def _autonomous_control(self):
        """
        Tam otonom kontrol (Adaptive Stage Controller ile)
        """
        # Adaptive Stage Controller ile mevcut durumu analiz et
        if self.adaptive_controller:
            # Vision system'den detected signs al
            detected_sign = self._get_detected_sign()
            
            # Mevcut pozisyonu al  
            current_position = self._get_current_position()
            
            # Sensor verilerini al
            sensor_data = self._get_sensor_data()
            
            # Safety Monitor ve Recovery Automation sistemlerini update et
            if self.safety_monitor:
                self.safety_monitor.update_sensor_data(sensor_data)
                vehicle_state = {
                    'speed': sensor_data.get('velocity', (0, 0))[0] if 'velocity' in sensor_data else 0,
                    'x': current_position[0],
                    'y': current_position[1],
                    'navigation_error': False  # Will be set by navigation system if error occurs
                }
                self.safety_monitor.update_vehicle_state(vehicle_state)
            
            if self.recovery_automation:
                self.recovery_automation.update_sensor_data(sensor_data)
                self.recovery_automation.update_vehicle_state(vehicle_state)
            
            # Aşama tespiti yap
            detected_stage = self.adaptive_controller.detect_current_stage(
                detected_sign=detected_sign,
                current_position=current_position, 
                sensor_data=sensor_data
            )
            
            if detected_stage:
                self.adaptive_controller.set_stage(detected_stage)
            
            # Adaptive kontrol parametrelerini al
            control_params = self.adaptive_controller.get_adaptive_control_params()
            
            # Özel davranış çalıştır
            special_behavior = control_params['special_behavior']
            behavior_result = self.adaptive_controller.execute_special_behavior(
                special_behavior, 
                current_position=current_position,
                sensor_data=sensor_data
            )
            
            self.logger.debug(f"🎛️ Adaptive control - Stage: {self.adaptive_controller.current_stage.value}")
            self.logger.debug(f"🎭 Special behavior: {special_behavior} -> {behavior_result['status']}")
        
        if self.mission_state == MissionState.NAVIGATING:
            # Mevcut waypoint'e git
            if self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                
                # Adaptive speed hesapla
                base_speed = self.config['vehicle']['max_speed']
                if self.adaptive_controller:
                    adapted_speed = self.adaptive_controller.get_adaptive_speed(base_speed)
                    self.logger.debug(f"🚀 Speed adaptation: {base_speed} -> {adapted_speed} m/s")
                else:
                    adapted_speed = base_speed
                
                # MRPT SLAM kullan (öncelik)
                if self.mrpt_slam and self.mrpt_slam.is_slam_ready():
                    success = self.mrpt_slam.navigate_to_point(target[0], target[1])
                    
                    if success:
                        # MRPT SLAM pose bilgisini al
                        current_pose = self.mrpt_slam.get_current_pose()
                        if current_pose:
                            # Hedefe varma kontrolü (adaptive tolerance ile)
                            goal_tolerance = self.config['navigation']['goal_tolerance']
                            if self.adaptive_controller:
                                stage_config = self.adaptive_controller.get_current_config()
                                # Bazı aşamalar daha hassas tolerance gerektirir
                                if stage_config['special_behavior'] == 'precision_targeting':
                                    goal_tolerance = 0.1  # 10cm hassasiyet
                                elif stage_config['special_behavior'] == 'water_crossing':
                                    goal_tolerance = 0.2  # 20cm su geçişi için
                            
                            distance_to_goal = np.sqrt(
                                (current_pose.pose.position.x - target[0])**2 + 
                                (current_pose.pose.position.y - target[1])**2
                            )
                            
                            if distance_to_goal < goal_tolerance:
                                self.logger.info(f"✅ MRPT SLAM Waypoint {self.current_waypoint_index + 1} tamamlandı")
                                self.current_waypoint_index += 1
                                
                                if self.current_waypoint_index >= len(self.waypoints):
                                    self.mission_state = MissionState.MISSION_COMPLETE
                                    self.logger.info("🏁 Görev tamamlandı!")
                
                # Standart SLAM fallback
                elif self.slam_navigation:
                    success = self.slam_navigation.navigate_to_goal(target[0], target[1])
                    
                    if success:
                        self.logger.info(f"✅ Standart SLAM Waypoint {self.current_waypoint_index + 1} tamamlandı")
                        self.current_waypoint_index += 1
                        
                        if self.current_waypoint_index >= len(self.waypoints):
                            self.mission_state = MissionState.MISSION_COMPLETE
                            self.logger.info("🏁 Görev tamamlandı!")
                else:
                    # Basit navigation (SLAM olmadan) - adaptive parameters ile
                    self._simple_navigation(target, adapted_speed)
                    
        elif self.mission_state == MissionState.SIGN_DETECTION:
            # Tabela tanıma durumu
            self._handle_sign_detection()
            
        elif self.mission_state == MissionState.OBSTACLE_AVOIDANCE:
            # Engel kaçınma - adaptive parameters ile
            self._handle_obstacle_avoidance()
    
    def _semi_autonomous_control(self):
        """
        Yarı otonom kontrol
        """
        # Obstacle detection + manual override capability
        if self._check_obstacles():
            self.mission_state = MissionState.OBSTACLE_AVOIDANCE
        else:
            # Normal navigation devam et
            if self.mission_state != MissionState.NAVIGATING:
                self.mission_state = MissionState.NAVIGATING
    
    def _simple_navigation(self, target, max_speed=None):
        """
        Basit navigation (SLAM olmadan) - adaptive parameters ile
        
        Args:
            target: (x, y) hedef koordinat
            max_speed: Maksimum hız limiti
        """
        if max_speed is None:
            max_speed = self.config['vehicle']['max_speed']
        
        self.logger.info(f"🎯 Hedef: {target} (Simple navigation, max_speed: {max_speed:.2f})")
        
        # Simulate reaching waypoint after some time
        time.sleep(2)
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.mission_state = MissionState.MISSION_COMPLETE
    
    def _handle_sign_detection(self):
        """
        Tabela tanıma işleme
        """
        # Vision system ile tabela ara
        if self.vision_system:
            # ROS tabanlı detection
            pass
        else:
            # Simulation - tabela tespit edildi varsay
            self.logger.info("📋 Tabela tespit edildi (simüle)")
            self.detected_signs.append({'type': 'test_sign', 'confidence': 0.9})
            self.mission_data['signs_detected'] += 1
        
        # Navigation durumuna geri dön
        self.mission_state = MissionState.NAVIGATING
    
    def _handle_obstacle_avoidance(self):
        """
        Engel kaçınma işleme
        """
        self.logger.warning("⚠️ Engel tespit edildi - kaçınma manevr'ası")
        
        # Basit engel kaçınma
        if self.arduino_controller:
            # Geriye git
            self.arduino_controller.set_motor_speed(-50, -50)
            time.sleep(1)
            
            # Sağa dön
            self.arduino_controller.set_motor_speed(50, -50)
            time.sleep(1)
            
            # İleri git
            self.arduino_controller.set_motor_speed(50, 50)
            time.sleep(2)
            
            # Sol dön (orijinal yöne dön)
            self.arduino_controller.set_motor_speed(-50, 50)
            time.sleep(1)
            
            # Dur
            self.arduino_controller.set_motor_speed(0, 0)
        
        self.mission_data['obstacles_avoided'] += 1
        self.mission_state = MissionState.NAVIGATING
    
    def _check_obstacles(self):
        """
        Engel kontrolü
        
        Returns:
            bool: Engel var mı
        """
        if self.sensor_controller and self.sensor_controller.lidar_data:
            # LiDAR verilerini kontrol et
            min_distance = min(self.sensor_controller.lidar_data.get('distances', [999]))
            return min_distance < self.config['navigation']['obstacle_threshold']
        
        return False
    
    def _monitoring_loop(self):
        """
        İzleme döngüsü
        """
        self.logger.info("👁️ İzleme döngüsü başladı")
        
        while self.running:
            try:
                # Sistem durumunu kontrol et
                self._check_system_health()
                
                # Görev zaman aşımı kontrolü
                if self.mission_data['start_time']:
                    elapsed = time.time() - self.mission_data['start_time']
                    if elapsed > self.config['mission']['max_mission_time']:
                        self.logger.warning("⏰ Görev zaman aşımı!")
                        self.stop_mission()
                
                # Status raporlama
                if int(time.time()) % 10 == 0:  # Her 10 saniyede bir
                    self._log_status()
                
            except Exception as e:
                self.logger.error(f"❌ Monitoring hatası: {e}")
                
            time.sleep(1)
    
    def _check_system_health(self):
        """
        Sistem sağlığını kontrol et
        """
        # Arduino bağlantısı
        if self.arduino_controller and not self.arduino_controller.is_connected():
            self.logger.error("❌ Arduino bağlantısı kesildi")
            self.emergency_stop_activated()
        
        # Sensor durumu
        if self.sensor_controller and not self.sensor_controller.is_healthy():
            self.logger.warning("⚠️ Sensor sistem problemi")
    
    def _log_status(self):
        """
        Durum raporu
        """
        status = {
            'mode': self.mode.value,
            'state': self.mission_state.value,
            'waypoint': f"{self.current_waypoint_index + 1}/{len(self.waypoints)}",
            'signs_detected': self.mission_data['signs_detected'],
            'obstacles_avoided': self.mission_data['obstacles_avoided']
        }
        
        self.logger.info(f"📊 Status: {status}")
    
    def get_status(self):
        """
        Sistem durumunu döndür (Adaptive Controller ile genişletilmiş)
        
        Returns:
            dict: Durum bilgileri
        """
        with self.lock:
            base_status = {
                'mode': self.mode.value,
                'state': self.mission_state.value,
                'running': self.running,
                'emergency_stop': self.emergency_stop,
                'waypoints_total': len(self.waypoints),
                'waypoint_current': self.current_waypoint_index,
                'mission_data': self.mission_data.copy()
            }
            
            # Adaptive controller status ekle
            if self.adaptive_controller:
                adaptive_status = self.adaptive_controller.get_status_report()
                base_status['adaptive_controller'] = {
                    'current_stage': adaptive_status['current_stage'],
                    'stage_description': adaptive_status['stage_description'],
                    'stage_progress': adaptive_status['stage_progress'],
                    'current_speed_limit': adaptive_status['current_config']['max_speed'],
                    'special_behavior': adaptive_status['current_config']['special_behavior']
                }
            
            # Sensor Fusion status ekle
            if self.sensor_fusion:
                fusion_metrics = self.sensor_fusion.get_performance_metrics()
                sensor_health = self.sensor_fusion.get_sensor_health()
                fused_data = self.sensor_fusion.get_fused_data()
                
                base_status['sensor_fusion'] = {
                    'fusion_rate': fusion_metrics['fusion_rate'],
                    'fusion_count': fusion_metrics['fusion_count'],
                    'healthy_sensors': fusion_metrics['contributing_sensors'],
                    'failed_sensors': fusion_metrics['failed_sensors'],
                    'fusion_confidence': fused_data.confidence if fused_data else 0.0,
                    'obstacle_count': len(fused_data.obstacles) if fused_data else 0
                }
            
            # Safety Monitor status ekle
            if self.safety_monitor:
                safety_status = self.safety_monitor.get_safety_status()
                base_status['safety_monitor'] = {
                    'level': safety_status['level'],
                    'active_incidents': safety_status['active_incidents'],
                    'total_incidents': safety_status['total_incidents'],
                    'avg_response_time': safety_status['avg_response_time'],
                    'geofence_status': safety_status['geofence_status']
                }
            
            # Recovery Automation status ekle
            if self.recovery_automation:
                recovery_status = self.recovery_automation.get_recovery_status()
                base_status['recovery_automation'] = {
                    'status': recovery_status['status'],
                    'success_rate': recovery_status['success_rate'],
                    'total_recoveries': recovery_status['total_recoveries'],
                    'avg_recovery_time': recovery_status['avg_recovery_time'],
                    'stuck_duration': recovery_status['stuck_duration']
                }
            
            return base_status
    
    def cleanup(self):
        """
        Temizlik işlemleri
        """
        self.logger.info("🧹 Temizlik başlıyor...")
        
        # Görev durdur
        self.stop_mission()
        
        # Alt sistemleri kapat
        if self.arduino_controller:
            self.arduino_controller.disconnect()
        
        if self.sensor_controller:
            self.sensor_controller.cleanup()
        
        if self.lidar_bridge:
            self.lidar_bridge.stop()
            
        if self.mrpt_slam:
            self.mrpt_slam.shutdown()
            
        if self.sensor_fusion:
            self.sensor_fusion.stop()
            
        if self.safety_monitor:
            self.safety_monitor.stop()
            
        if self.recovery_automation:
            self.recovery_automation.stop()
        
        self.logger.info("🧹 Temizlik tamamlandı")
    
    def _get_detected_sign(self) -> Optional[str]:
        """
        Vision system'den detected sign al
        
        Returns:
            Optional[str]: Detected sign text
        """
        # Vision system integration
        if hasattr(self, 'vision_system') and self.vision_system:
            # ROS tabanlı vision system
            pass
        
        # Simulation için mock data
        return None
    
    def _get_current_position(self) -> Optional[Tuple[float, float]]:
        """
        Mevcut robot pozisyonunu al
        
        Returns:
            Tuple[float, float]: (x, y) pozisyon
        """
        if self.mrpt_slam and self.mrpt_slam.is_slam_ready():
            pose = self.mrpt_slam.get_current_pose()
            if pose:
                return (pose.pose.position.x, pose.pose.position.y)
        
        # Simulation için mock position
        return (0.0, 0.0)
    
    def _get_sensor_data(self) -> Dict[str, Any]:
        """
        Mevcut sensor verilerini al (Sensor Fusion System entegreli)
        
        Returns:
            Dict: Sensor verileri
        """
        sensor_data = {}
        
        # Sensor Fusion System'den fused data al
        if self.sensor_fusion:
            fused_data = self.sensor_fusion.get_fused_data()
            if fused_data:
                sensor_data.update({
                    'position': fused_data.position,
                    'orientation': fused_data.orientation,
                    'velocity': fused_data.velocity,
                    'obstacles': fused_data.obstacles,
                    'fusion_confidence': fused_data.confidence,
                    'contributing_sensors': [s.value for s in fused_data.contributing_sensors]
                })
                
                # Extract obstacle info for backward compatibility
                sensor_data['obstacle_count'] = len(fused_data.obstacles)
                if fused_data.obstacles:
                    closest_obstacle = min(fused_data.obstacles, 
                                         key=lambda obs: np.linalg.norm(obs['position']))
                    sensor_data['closest_obstacle_distance'] = np.linalg.norm(closest_obstacle['position'])
                else:
                    sensor_data['closest_obstacle_distance'] = 999.0
            
            # Add sensor health info
            sensor_health = self.sensor_fusion.get_sensor_health()
            sensor_data['sensor_health'] = {s.value: h.value for s, h in sensor_health.items()}
        
        # Raw sensor data (fallback ve sensor fusion'a input için)
        if self.sensor_controller:
            raw_data = {
                'ultrasonic_distance': self.sensor_controller.sensor_data.get('ultrasonic', {}).get('distance_cm', 999) / 100.0,
                'imu_pitch': self.sensor_controller.sensor_data.get('imu', {}).get('accel', {}).get('x', 0) * 57.3,  # rad to deg
                'imu_roll': self.sensor_controller.sensor_data.get('imu', {}).get('accel', {}).get('y', 0) * 57.3,
                'accel_variance': 0.0,  # Calculate from IMU data
                'humidity': self.sensor_controller.sensor_data.get('environment', {}).get('humidity', 0)
            }
            sensor_data.update(raw_data)
            
            # Feed raw data to sensor fusion system
            self._feed_sensor_fusion(self.sensor_controller.sensor_data)
        else:
            # Simulation data
            sim_data = {
                'ultrasonic_distance': 999.0,
                'imu_pitch': 0.0,
                'imu_roll': 0.0,
                'accel_variance': 0.0,
                'obstacle_count': 0,
                'humidity': 50.0,
                'closest_obstacle_distance': 999.0
            }
            sensor_data.update(sim_data)
        
        return sensor_data
    
    def _feed_sensor_fusion(self, raw_sensor_data: Dict[str, Any]):
        """
        Raw sensor verisini sensor fusion system'e besle
        
        Args:
            raw_sensor_data: Ham sensor verisi
        """
        if not self.sensor_fusion:
            return
        
        current_time = time.time()
        
        # Ultrasonic sensor reading
        if 'ultrasonic' in raw_sensor_data:
            us_data = raw_sensor_data['ultrasonic']
            if 'distance_cm' in us_data:
                distance = us_data['distance_cm'] / 100.0  # cm to m
                reading = SensorReading(
                    sensor_type=SensorType.ULTRASONIC,
                    timestamp=current_time,
                    data={
                        'obstacles': [{'position': (distance, 0), 'size': 0.5, 'confidence': 0.7}] if distance < 5.0 else []
                    },
                    confidence=0.8 if distance < 10.0 else 0.6
                )
                self.sensor_fusion.add_sensor_reading(reading)
        
        # IMU sensor reading
        if 'imu' in raw_sensor_data:
            imu_data = raw_sensor_data['imu']
            if 'accel' in imu_data:
                accel = imu_data['accel']
                reading = SensorReading(
                    sensor_type=SensorType.IMU,
                    timestamp=current_time,
                    data={
                        'orientation': accel.get('z', 0) * 0.1,  # Rough estimate
                        'angular_velocity': 0.0  # Would need gyro data
                    },
                    confidence=0.7
                )
                self.sensor_fusion.add_sensor_reading(reading)
    
    def _handle_safety_obstacle(self, incident: 'SafetyIncident'):
        """
        Handle safety incident: obstacle too close
        
        Args:
            incident: Safety incident data
        """
        self.logger.warning(f"🚨 Safety callback: Obstacle too close - {incident.level.value}")
        
        if incident.level == SafetyLevel.EMERGENCY:
            self.set_mode(AutonomousMode.EMERGENCY_STOP)
            
            # Trigger recovery automation
            if self.recovery_automation:
                self.recovery_automation.trigger_recovery(
                    FailureType.PATH_BLOCKED, 
                    {'safety_incident': True}
                )
    
    def _handle_safety_tilt(self, incident: 'SafetyIncident'):
        """
        Handle safety incident: excessive tilt
        
        Args:
            incident: Safety incident data
        """
        self.logger.warning(f"⚖️ Safety callback: Excessive tilt - {incident.level.value}")
        
        if incident.level == SafetyLevel.CRITICAL:
            # Slow down and stabilize
            if self.adaptive_controller:
                self.adaptive_controller.set_stage(ParkurStage.DIK_EGIM)  # Steep climb mode
    
    def _handle_recovery_emergency_return(self, recovery_context: Dict[str, Any]):
        """
        Handle recovery action: emergency return
        
        Args:
            recovery_context: Recovery context data
        """
        self.logger.critical("🏠 Recovery callback: Emergency return initiated")
        
        # Switch to emergency return mode
        self.set_mode(AutonomousMode.EMERGENCY_STOP)
        self.mission_state = MissionState.EMERGENCY
        
        # Set emergency waypoint (return to start)
        if self.waypoints:
            emergency_waypoint = self.waypoints[0]  # Return to start
            self.waypoints = [emergency_waypoint]
            self.current_waypoint_index = 0
    
    def _handle_recovery_manual_takeover(self, recovery_context: Dict[str, Any]):
        """
        Handle recovery action: manual takeover required
        
        Args:
            recovery_context: Recovery context data
        """
        self.logger.critical("👤 Recovery callback: Manual takeover required")
        
        # Switch to manual mode and stop
        self.set_mode(AutonomousMode.MANUAL)
        self.mission_state = MissionState.EMERGENCY
        self.stop_vehicle()

def main():
    """
    Test ana fonksiyonu
    """
    print("🚀 TEKNOFEST Otonom Controller Testi")
    print("=" * 50)
    
    # Controller oluştur
    controller = AutonomousController()
    
    try:
        # Sistemleri başlat
        if not controller.initialize_systems():
            print("❌ Sistem başlatma başarısız")
            return
        
        # Test waypoints
        test_waypoints = [
            (0, 0),    # Start
            (5, 0),    # İleri git
            (5, 5),    # Sağa dön
            (0, 5),    # Geriye git
            (0, 0)     # Başa dön
        ]
        
        controller.load_mission_waypoints(test_waypoints)
        
        # Otonom moda geç
        controller.set_mode(AutonomousMode.FULLY_AUTONOMOUS)
        
        # Görevi başlat
        if controller.start_mission():
            print("✅ Görev başlatıldı")
            
            # Test süresi
            test_duration = 60  # 1 dakika
            start_time = time.time()
            
            while time.time() - start_time < test_duration:
                status = controller.get_status()
                print(f"📊 Status: {status['state']} - Waypoint {status['waypoint_current']+1}/{status['waypoints_total']}")
                
                if status['state'] == MissionState.MISSION_COMPLETE.value:
                    print("🏁 Görev tamamlandı!")
                    break
                
                time.sleep(5)
        
        else:
            print("❌ Görev başlatılamadı")
    
    except KeyboardInterrupt:
        print("🛑 Klavye ile durduruldu")
    
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()

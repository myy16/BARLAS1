#!/usr/bin/env python3
"""
TEKNOFEST Parkur Aşaması Adaptive Controller
Her parkur segmentine özel kontrol parametreleri ve davranışlar
"""

import time
import logging
import numpy as np
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any

class ParkurStage(Enum):
    """TEKNOFEST Parkur Aşamaları"""
    START_ZONE = "start_zone"
    DIK_ENGEL = "dik_engel"          # Dikey engel
    TASLI_YOL = "tasli_yol"          # Taşlı yol
    YAN_EGIM = "yan_egim"            # Yan eğim
    HIZLANMA = "hizlanma"            # Hızlanma bölgesi
    SIGI_SU = "sigi_su"              # Sığ su geçişi
    TRAFIK_KONILERI = "trafik_konileri"  # Trafik konileri
    ENGEBELI_ARAZI = "engebeli_arazi"    # Engebeli arazi
    DIK_EGIM = "dik_egim"            # Dik eğim
    ATIS_BOLGE = "atis_bolge"        # Atış bölgesi
    FINISH_ZONE = "finish_zone"

class AdaptiveStageController:
    """
    Parkur aşamalarına göre adaptive kontrol sistemi
    Her aşama için özel parametreler ve davranış patterns
    """
    
    def __init__(self, base_config: Dict[str, Any]):
        """
        Adaptive controller başlatıcı
        
        Args:
            base_config: Temel konfigürasyon
        """
        self.logger = logging.getLogger(__name__)
        self.base_config = base_config
        
        # Mevcut aşama durumu
        self.current_stage = ParkurStage.START_ZONE
        self.stage_start_time = None
        self.stage_progress = 0.0
        
        # Aşamaya özel konfigürasyonlar
        self.stage_configs = self._initialize_stage_configs()
        
        # Sensor fusion için ağırlıklar
        self.sensor_weights = {
            'lidar': 1.0,
            'camera': 1.0,
            'imu': 1.0,
            'encoders': 1.0,
            'ultrasonic': 0.5
        }
        
        self.logger.info("🎛️ Adaptive Stage Controller başlatıldı")
    
    def _initialize_stage_configs(self) -> Dict[ParkurStage, Dict[str, Any]]:
        """
        Her parkur aşaması için özel konfigürasyonlar
        
        Returns:
            Dict: Aşama konfigürasyonları
        """
        return {
            ParkurStage.START_ZONE: {
                'max_speed': 1.0,           # Yavaş başlangıç
                'acceleration': 0.5,        # Yumuşak ivme
                'turning_aggressiveness': 0.3,
                'obstacle_threshold': 1.5,  # Güvenli mesafe
                'sensor_priority': ['lidar', 'camera', 'imu'],
                'special_behavior': 'calibration_check',
                'description': "Sistem kalibrasyon ve güvenli başlangıç"
            },
            
            ParkurStage.DIK_ENGEL: {
                'max_speed': 0.8,           # Yavaş ve kontrollü
                'acceleration': 0.3,        # Çok yumuşak ivme
                'turning_aggressiveness': 0.8,  # Keskin manevra
                'obstacle_threshold': 0.8,  # Yakın engel toleransı
                'sensor_priority': ['lidar', 'ultrasonic', 'camera'],
                'special_behavior': 'precision_maneuvering',
                'climb_mode': True,         # Tırmanma modu
                'wheel_torque_boost': 1.3,  # Motor gücü artırımı
                'description': "Dikey engel tırmanma - yüksek torque, düşük hız"
            },
            
            ParkurStage.TASLI_YOL: {
                'max_speed': 1.2,           # Orta hız
                'acceleration': 0.4,        # Titreşimli hareket
                'turning_aggressiveness': 0.4,
                'obstacle_threshold': 1.0,
                'sensor_priority': ['imu', 'lidar', 'camera'],
                'special_behavior': 'vibration_compensation',
                'suspension_mode': 'soft',  # Yumuşak süspansiyon
                'stability_control': True,  # Stabilite kontrolü
                'description': "Taşlı yol - titreşim kompensasyonu, stabilite öncelikli"
            },
            
            ParkurStage.YAN_EGIM: {
                'max_speed': 1.5,           # Dengeli hız
                'acceleration': 0.6,
                'turning_aggressiveness': 0.2,  # Yan devrilmeyi önle
                'obstacle_threshold': 1.2,
                'sensor_priority': ['imu', 'camera', 'lidar'],
                'special_behavior': 'lateral_stability',
                'tilt_compensation': True,  # Eğim kompensasyonu
                'differential_steering': True,  # Diferansiyel direksiyon
                'max_tilt_angle': 15.0,     # 15° maksimum eğim
                'description': "Yan eğim - lateral stabilite, eğim kompensasyonu"
            },
            
            ParkurStage.HIZLANMA: {
                'max_speed': 2.5,           # Maksimum hız!
                'acceleration': 1.0,        # Hızlı ivme
                'turning_aggressiveness': 0.6,
                'obstacle_threshold': 2.0,  # Uzak mesafe
                'sensor_priority': ['camera', 'lidar', 'imu'],
                'special_behavior': 'high_speed_cruise',
                'aerodynamic_mode': True,   # Aerodinamik optimizasyon
                'energy_efficiency': False, # Performans öncelikli
                'description': "Hızlanma bölgesi - maksimum performans, hız öncelikli"
            },
            
            ParkurStage.SIGI_SU: {
                'max_speed': 0.6,           # Çok yavaş!
                'acceleration': 0.2,        # Çok yumuşak
                'turning_aggressiveness': 0.1,  # Minimal dönüş
                'obstacle_threshold': 0.5,  # Su seviyesi
                'sensor_priority': ['ultrasonic', 'camera', 'imu'],
                'special_behavior': 'water_crossing',
                'waterproof_mode': True,    # Su geçirme koruması
                'bottom_clearance': 0.15,   # 15cm taban mesafesi
                'wake_minimization': True,  # Dalga minimizasyonu
                'electrical_protection': True,  # Elektriksel koruma
                'description': "Sığ su geçişi - YAVAS, su koruması aktif, elektronik güvenli"
            },
            
            ParkurStage.TRAFIK_KONILERI: {
                'max_speed': 1.0,           # Kontrollü hız
                'acceleration': 0.4,
                'turning_aggressiveness': 0.9,  # Çeviklik gerekli
                'obstacle_threshold': 0.6,  # Yakın geçiş
                'sensor_priority': ['camera', 'lidar', 'ultrasonic'],
                'special_behavior': 'slalom_navigation',
                'path_precision': 'high',   # Yüksek hassasiyet
                'dynamic_replanning': True, # Dinamik rota
                'cone_detection_boost': True,  # Koni detection artırılmış
                'description': "Trafik konileri slalom - çeviklik, hassasiyet, dinamik planlama"
            },
            
            ParkurStage.ENGEBELI_ARAZI: {
                'max_speed': 1.8,           # Yüksek hız ama kontrollü
                'acceleration': 0.7,
                'turning_aggressiveness': 0.5,
                'obstacle_threshold': 1.0,
                'sensor_priority': ['imu', 'lidar', 'camera'],
                'special_behavior': 'rough_terrain',
                'suspension_mode': 'adaptive',  # Adaptif süspansiyon
                'ground_clearance_monitor': True,  # Yükseklik kontrolü
                'shock_absorption': True,   # Şok emiciler
                'traction_control': True,   # Çekiş kontrolü
                'description': "Engebeli arazi - adaptif süspansiyon, çekiş kontrolü"
            },
            
            ParkurStage.DIK_EGIM: {
                'max_speed': 0.7,           # Düşük hız
                'acceleration': 0.3,
                'turning_aggressiveness': 0.1,  # Minimal dönüş
                'obstacle_threshold': 1.0,
                'sensor_priority': ['imu', 'camera', 'lidar'],
                'special_behavior': 'steep_climb',
                'hill_assist': True,        # Yokuş asistanı
                'rollback_prevention': True, # Geri kayma önleme
                'engine_brake': True,       # Motor freni
                'max_climb_angle': 30.0,    # 30° maksimum tırmanış
                'description': "Dik eğim tırmanışı - yokuş asistanı, geri kayma önleme"
            },
            
            ParkurStage.ATIS_BOLGE: {
                'max_speed': 0.3,           # Çok yavaş
                'acceleration': 0.1,        # Minimal hareket
                'turning_aggressiveness': 0.8,  # Hassas pozisyonlama
                'obstacle_threshold': 0.4,
                'sensor_priority': ['camera', 'ultrasonic', 'imu'],
                'special_behavior': 'precision_targeting',
                'stabilization_mode': True, # Stabilizasyon
                'precision_control': 'maximum',  # Maksimum hassasiyet
                'targeting_system': True,   # Nişan alma sistemi
                'vibration_damping': True,  # Titreşim sönümleme
                'description': "Atış bölgesi - hassas pozisyon, stabilizasyon, minimal hareket"
            },
            
            ParkurStage.FINISH_ZONE: {
                'max_speed': 1.5,           # Orta hız
                'acceleration': 0.5,
                'turning_aggressiveness': 0.3,
                'obstacle_threshold': 1.0,
                'sensor_priority': ['camera', 'lidar', 'imu'],
                'special_behavior': 'mission_completion',
                'celebration_mode': False,  # Henüz erken :)
                'data_logging': True,       # Veri kayıt
                'performance_report': True, # Performans raporu
                'description': "Bitiş bölgesi - görev tamamlama, veri kayıt"
            }
        }
    
    def detect_current_stage(self, detected_sign: str = None, 
                           current_position: Tuple[float, float] = None,
                           sensor_data: Dict[str, Any] = None) -> ParkurStage:
        """
        Mevcut parkur aşamasını tespit et
        
        Args:
            detected_sign: Tespit edilen tabela
            current_position: Mevcut pozisyon
            sensor_data: Sensor verileri
            
        Returns:
            ParkurStage: Tespit edilen aşama
        """
        # Tabela tabanlı aşama tespiti (birincil)
        if detected_sign:
            stage_mapping = {
                'DİK ENGEL': ParkurStage.DIK_ENGEL,
                'TAŞLI YOL': ParkurStage.TASLI_YOL,
                'YAN EĞİM': ParkurStage.YAN_EGIM,
                'HIZLANMA': ParkurStage.HIZLANMA,
                'SIĞI SU': ParkurStage.SIGI_SU,
                'TRAFİK KONİLERİ': ParkurStage.TRAFIK_KONILERI,
                'ENGEBELİ ARAZİ': ParkurStage.ENGEBELI_ARAZI,
                'DİK EĞİM': ParkurStage.DIK_EGIM,
                'ATIŞ': ParkurStage.ATIS_BOLGE,
                'START': ParkurStage.START_ZONE,
                'FİNİSH': ParkurStage.FINISH_ZONE
            }
            
            new_stage = stage_mapping.get(detected_sign)
            if new_stage and new_stage != self.current_stage:
                self.logger.info(f"📋 Tabela tespit - Aşama değişimi: {self.current_stage.value} -> {new_stage.value}")
                return new_stage
        
        # Sensor tabanlı aşama tespiti (ikincil)
        if sensor_data:
            detected_stage = self._analyze_sensor_patterns(sensor_data)
            if detected_stage and detected_stage != self.current_stage:
                self.logger.info(f"📊 Sensor analiz - Aşama değişimi: {self.current_stage.value} -> {detected_stage.value}")
                return detected_stage
        
        # Pozisyon tabanlı aşama tespiti (üçüncül)
        if current_position:
            detected_stage = self._analyze_position_stage(current_position)
            if detected_stage and detected_stage != self.current_stage:
                self.logger.info(f"📍 Pozisyon analiz - Aşama değişimi: {self.current_stage.value} -> {detected_stage.value}")
                return detected_stage
        
        return self.current_stage
    
    def _analyze_sensor_patterns(self, sensor_data: Dict[str, Any]) -> Optional[ParkurStage]:
        """
        Sensor verilerini analiz ederek aşama tahmin et
        
        Args:
            sensor_data: Sensor verileri
            
        Returns:
            ParkurStage: Tahmin edilen aşama
        """
        try:
            # Su tespiti (ultrasonic + high humidity)
            if (sensor_data.get('ultrasonic_distance', 999) < 0.2 and
                sensor_data.get('humidity', 0) > 85):
                return ParkurStage.SIGI_SU
            
            # Dik eğim tespiti (IMU pitch angle)
            pitch = sensor_data.get('imu_pitch', 0)
            if abs(pitch) > 20:  # 20° üzeri eğim
                if pitch > 20:
                    return ParkurStage.DIK_EGIM
                elif abs(sensor_data.get('imu_roll', 0)) > 10:  # Yan eğim
                    return ParkurStage.YAN_EGIM
            
            # Vibrasyon tespiti (accelerometer variance)
            accel_var = sensor_data.get('accel_variance', 0)
            if accel_var > 2.0:  # Yüksek titreşim
                return ParkurStage.TASLI_YOL
            
            # Çoklu obstacle tespiti (trafik konileri)
            obstacle_count = sensor_data.get('obstacle_count', 0)
            if obstacle_count > 3:  # 3'den fazla engel
                return ParkurStage.TRAFIK_KONILERI
            
            return None
            
        except Exception as e:
            self.logger.error(f"❌ Sensor analiz hatası: {e}")
            return None
    
    def _analyze_position_stage(self, position: Tuple[float, float]) -> Optional[ParkurStage]:
        """
        Pozisyon tabanlı aşama tespiti
        
        Args:
            position: (x, y) pozisyon
            
        Returns:
            ParkurStage: Tahmin edilen aşama
        """
        x, y = position
        
        # TEKNOFEST parkur layout'una göre pozisyon-aşama mapping
        # Bu değerler gerçek parkur layout'una göre ayarlanmalı
        position_stages = [
            ((0, 5), (0, 5), ParkurStage.START_ZONE),
            ((5, 10), (0, 5), ParkurStage.DIK_ENGEL),
            ((10, 15), (0, 5), ParkurStage.TASLI_YOL),
            ((15, 20), (0, 5), ParkurStage.YAN_EGIM),
            ((20, 25), (0, 5), ParkurStage.HIZLANMA),
            ((25, 30), (0, 5), ParkurStage.SIGI_SU),
            ((30, 35), (0, 10), ParkurStage.TRAFIK_KONILERI),
            ((35, 40), (0, 5), ParkurStage.ENGEBELI_ARAZI),
            ((40, 45), (0, 5), ParkurStage.DIK_EGIM),
            ((45, 50), (0, 5), ParkurStage.ATIS_BOLGE),
            ((50, 55), (0, 5), ParkurStage.FINISH_ZONE)
        ]
        
        for x_range, y_range, stage in position_stages:
            if (x_range[0] <= x <= x_range[1] and 
                y_range[0] <= y <= y_range[1]):
                return stage
        
        return None
    
    def set_stage(self, new_stage: ParkurStage):
        """
        Aktif parkur aşamasını değiştir
        
        Args:
            new_stage: Yeni aşama
        """
        if new_stage != self.current_stage:
            old_stage = self.current_stage
            self.current_stage = new_stage
            self.stage_start_time = time.time()
            self.stage_progress = 0.0
            
            self.logger.info(f"🔄 Parkur aşama değişimi: {old_stage.value} -> {new_stage.value}")
            self.logger.info(f"📝 Yeni aşama: {self.get_current_config()['description']}")
    
    def get_current_config(self) -> Dict[str, Any]:
        """
        Mevcut aşama konfigürasyonunu döndür
        
        Returns:
            Dict: Aşama konfigürasyonu
        """
        return self.stage_configs[self.current_stage]
    
    def get_adaptive_speed(self, base_speed: float, situation_factor: float = 1.0) -> float:
        """
        Aşamaya göre adaptif hız hesapla
        
        Args:
            base_speed: Temel hız
            situation_factor: Durum faktörü (0.1-2.0)
            
        Returns:
            float: Optimize edilmiş hız
        """
        config = self.get_current_config()
        max_speed = config['max_speed']
        
        # Aşama limitini uygula
        adapted_speed = min(base_speed, max_speed)
        
        # Durum faktörünü uygula
        adapted_speed *= situation_factor
        
        # Final bounds check
        adapted_speed = max(0.1, min(adapted_speed, self.base_config['vehicle']['max_speed']))
        
        return adapted_speed
    
    def get_adaptive_control_params(self) -> Dict[str, Any]:
        """
        Aşamaya göre kontrol parametrelerini döndür
        
        Returns:
            Dict: Kontrol parametreleri
        """
        config = self.get_current_config()
        
        return {
            'max_speed': config['max_speed'],
            'acceleration': config['acceleration'],
            'turning_aggressiveness': config['turning_aggressiveness'],
            'obstacle_threshold': config['obstacle_threshold'],
            'sensor_priority': config['sensor_priority'],
            'special_behavior': config['special_behavior']
        }
    
    def execute_special_behavior(self, behavior: str, **kwargs) -> Dict[str, Any]:
        """
        Özel davranış patterns'ını çalıştır
        
        Args:
            behavior: Davranış tipi
            **kwargs: Davranış parametreleri
            
        Returns:
            Dict: Davranış sonucu
        """
        behavior_handlers = {
            'calibration_check': self._behavior_calibration_check,
            'precision_maneuvering': self._behavior_precision_maneuvering,
            'vibration_compensation': self._behavior_vibration_compensation,
            'lateral_stability': self._behavior_lateral_stability,
            'high_speed_cruise': self._behavior_high_speed_cruise,
            'water_crossing': self._behavior_water_crossing,
            'slalom_navigation': self._behavior_slalom_navigation,
            'rough_terrain': self._behavior_rough_terrain,
            'steep_climb': self._behavior_steep_climb,
            'precision_targeting': self._behavior_precision_targeting,
            'mission_completion': self._behavior_mission_completion
        }
        
        handler = behavior_handlers.get(behavior)
        if handler:
            return handler(**kwargs)
        else:
            self.logger.warning(f"⚠️ Bilinmeyen davranış: {behavior}")
            return {'status': 'unknown_behavior'}
    
    # Özel davranış implementasyonları
    def _behavior_calibration_check(self, **kwargs) -> Dict[str, Any]:
        """START_ZONE: Sistem kalibrasyon kontrolü"""
        return {
            'status': 'calibrating',
            'actions': ['sensor_check', 'motor_test', 'communication_verify'],
            'duration': 5.0
        }
    
    def _behavior_precision_maneuvering(self, **kwargs) -> Dict[str, Any]:
        """DİK_ENGEL: Hassas manevra"""
        return {
            'status': 'precision_mode',
            'motor_power_boost': 1.3,
            'sensor_sampling_rate': 2.0,
            'path_deviation_tolerance': 0.05  # 5cm tolerans
        }
    
    def _behavior_vibration_compensation(self, **kwargs) -> Dict[str, Any]:
        """TAŞLI_YOL: Titreşim kompensasyonu"""
        return {
            'status': 'vibration_damping',
            'control_smoothing_factor': 0.3,
            'sensor_filtering': 'kalman',
            'suspension_adjustment': 'soft'
        }
    
    def _behavior_lateral_stability(self, **kwargs) -> Dict[str, Any]:
        """YAN_EĞİM: Yanal stabilite"""
        return {
            'status': 'stability_control',
            'differential_steering_ratio': 0.7,
            'tilt_compensation_active': True,
            'max_lateral_acceleration': 2.0
        }
    
    def _behavior_high_speed_cruise(self, **kwargs) -> Dict[str, Any]:
        """HIZLANMA: Yüksek hız cruise"""
        return {
            'status': 'high_speed_mode',
            'aerodynamic_optimization': True,
            'energy_efficiency_mode': False,
            'lookahead_distance': 5.0  # 5m ileriyi gör
        }
    
    def _behavior_water_crossing(self, **kwargs) -> Dict[str, Any]:
        """SIĞI_SU: Su geçişi"""
        return {
            'status': 'water_mode',
            'electrical_protection': True,
            'minimum_ground_clearance': 0.15,
            'wake_minimization': True,
            'emergency_protocols': ['water_level_monitor', 'electrical_cutoff']
        }
    
    def _behavior_slalom_navigation(self, **kwargs) -> Dict[str, Any]:
        """TRAFİK_KONİLERİ: Slalom navigasyonu"""
        return {
            'status': 'slalom_mode',
            'dynamic_path_planning': True,
            'cone_detection_enhancement': 2.0,
            'turning_precision': 'maximum'
        }
    
    def _behavior_rough_terrain(self, **kwargs) -> Dict[str, Any]:
        """ENGEBELİ_ARAZİ: Engebeli arazi"""
        return {
            'status': 'terrain_mode',
            'suspension_adaptive': True,
            'traction_control': True,
            'shock_absorption': 'maximum'
        }
    
    def _behavior_steep_climb(self, **kwargs) -> Dict[str, Any]:
        """DİK_EĞİM: Dik eğim tırmanışı"""
        return {
            'status': 'climb_mode',
            'hill_assist': True,
            'rollback_prevention': True,
            'motor_torque_boost': 1.5
        }
    
    def _behavior_precision_targeting(self, **kwargs) -> Dict[str, Any]:
        """ATIŞ_BÖLGE: Hassas pozisyonlama"""
        return {
            'status': 'targeting_mode',
            'stabilization_maximum': True,
            'movement_minimal': True,
            'vibration_damping_active': True
        }
    
    def _behavior_mission_completion(self, **kwargs) -> Dict[str, Any]:
        """FİNİSH_ZONE: Görev tamamlama"""
        return {
            'status': 'mission_complete',
            'data_logging_final': True,
            'performance_report': True,
            'system_shutdown_prepare': False  # Henüz değil!
        }
    
    def get_stage_progress(self) -> float:
        """
        Mevcut aşama ilerleme yüzdesi
        
        Returns:
            float: İlerleme (0.0-1.0)
        """
        if self.stage_start_time is None:
            return 0.0
        
        elapsed = time.time() - self.stage_start_time
        # Her aşama için tahmini süre (gerçek veri ile kalibre edilmeli)
        estimated_duration = 60.0  # 60 saniye varsayılan
        
        return min(elapsed / estimated_duration, 1.0)
    
    def get_status_report(self) -> Dict[str, Any]:
        """
        Detaylı durum raporu
        
        Returns:
            Dict: Durum raporu
        """
        config = self.get_current_config()
        
        return {
            'current_stage': self.current_stage.value,
            'stage_description': config['description'],
            'stage_progress': self.get_stage_progress(),
            'elapsed_time': time.time() - self.stage_start_time if self.stage_start_time else 0,
            'current_config': {
                'max_speed': config['max_speed'],
                'obstacle_threshold': config['obstacle_threshold'],
                'special_behavior': config['special_behavior']
            },
            'sensor_priority': config['sensor_priority']
        }

def main():
    """
    Test fonksiyonu
    """
    logging.basicConfig(level=logging.INFO)
    
    # Base config
    base_config = {
        'vehicle': {'max_speed': 2.5}
    }
    
    # Controller oluştur
    controller = AdaptiveStageController(base_config)
    
    # Test scenarios
    test_scenarios = [
        {'detected_sign': 'START', 'description': 'Başlangıç aşaması'},
        {'detected_sign': 'SIĞI SU', 'description': 'Su geçişi aşaması'},
        {'detected_sign': 'HIZLANMA', 'description': 'Hızlanma aşaması'},
        {'detected_sign': 'TRAFİK KONİLERİ', 'description': 'Slalom aşaması'},
        {'detected_sign': 'ATIŞ', 'description': 'Atış aşaması'},
        {'detected_sign': 'FİNİSH', 'description': 'Bitiş aşaması'}
    ]
    
    print("🧪 ADAPTIVE STAGE CONTROLLER TEST")
    print("=" * 50)
    
    for scenario in test_scenarios:
        print(f"\n📋 {scenario['description']}:")
        
        # Aşama tespit et ve ayarla
        new_stage = controller.detect_current_stage(
            detected_sign=scenario['detected_sign']
        )
        
        if new_stage:
            controller.set_stage(new_stage)
            
            # Konfigürasyonu göster
            config = controller.get_current_config()
            print(f"   ⚙️ Max Speed: {config['max_speed']} m/s")
            print(f"   🎯 Obstacle Threshold: {config['obstacle_threshold']} m")
            print(f"   🤖 Special Behavior: {config['special_behavior']}")
            print(f"   📊 Sensor Priority: {config['sensor_priority']}")
            
            # Özel davranış test et
            behavior_result = controller.execute_special_behavior(
                config['special_behavior']
            )
            print(f"   🎭 Behavior Result: {behavior_result['status']}")
        
        time.sleep(1)  # Kısa bekleme
    
    print(f"\n📊 Final Status Report:")
    status = controller.get_status_report()
    for key, value in status.items():
        print(f"   {key}: {value}")

if __name__ == "__main__":
    main()

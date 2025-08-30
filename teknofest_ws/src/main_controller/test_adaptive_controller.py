#!/usr/bin/env python3
"""
Adaptive Stage Controller Test Script
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

try:
    from main_controller.adaptive_stage_controller import AdaptiveStageController, ParkurStage
    from dataclasses import dataclass
    from typing import List
    
    # StageConfig sınıfını tanımla (adaptive_stage_controller'dan kopyala)
    @dataclass
    class StageConfig:
        max_speed: float = 1.5
        max_acceleration: float = 2.0
        min_speed: float = 0.2
        steering_sensitivity: float = 1.0
        brake_sensitivity: float = 1.0
        sensor_priority: List[str] = None
        special_behavior: str = ""
        
        def __post_init__(self):
            if self.sensor_priority is None:
                self.sensor_priority = ['lidar', 'camera', 'ultrasonic']
    
    print("🚀 AdaptiveStageController Test Başlıyor...")
    
    # Base config oluştur
    base_config = StageConfig(
        max_speed=1.5,
        max_acceleration=2.0,
        min_speed=0.2,
        steering_sensitivity=1.0,
        brake_sensitivity=1.0,
        sensor_priority=['lidar', 'camera', 'ultrasonic'],
        special_behavior=""
    )
    
    # Controller oluştur
    controller = AdaptiveStageController(base_config)
    print("✅ AdaptiveStageController başarıyla oluşturuldu")
    
    # İlk durum raporu
    status = controller.get_status_report()
    print("\n📊 İlk Durum:")
    print(f"   Stage: {status['current_stage']}")
    print(f"   Açıklama: {status['stage_description']}")
    print(f"   Max Hız: {status['current_config']['max_speed']} m/s")
    print(f"   Engel Threshold: {status['current_config']['obstacle_threshold']} m")
    print(f"   Sensor Önceliği: {status['sensor_priority']}")
    print(f"   Geçen Süre: {status['elapsed_time']:.1f} s")
    
    # Stage geçişi test et
    print("\n🔄 Stage Geçiş Testi...")
    
    # Dikey engel stage'ine geç
    controller.set_stage(ParkurStage.DIK_ENGEL)
    status = controller.get_status_report()
    print(f"   ➡️ {status['current_stage']}: {status['stage_description']}")
    print(f"   ⚡ Hız Limiti: {status['current_config']['max_speed']} m/s")
    print(f"   🎯 Özel Davranış: {status['current_config']['special_behavior']}")
    
    # Sığ su stage'ine geç  
    controller.set_stage(ParkurStage.SIGI_SU)
    status = controller.get_status_report()
    print(f"   ➡️ {status['current_stage']}: {status['stage_description']}")
    print(f"   ⚡ Hız Limiti: {status['current_config']['max_speed']} m/s")
    print(f"   🌊 Özel Davranış: {status['current_config']['special_behavior']}")
    
    # Hızlanma stage'ine geç
    controller.set_stage(ParkurStage.HIZLANMA)
    status = controller.get_status_report()
    print(f"   ➡️ {status['current_stage']}: {status['stage_description']}")
    print(f"   ⚡ Hız Limiti: {status['current_config']['max_speed']} m/s")
    print(f"   🏁 Özel Davranış: {status['current_config']['special_behavior']}")
    
    # Adaptif parametre hesaplama test
    print("\n🎯 Adaptif Kontrol Parametreleri Testi...")
    
    params = controller.get_adaptive_control_params()
    print(f"   📊 Max Hız: {params['max_speed']:.2f} m/s")
    print(f"   📊 İvme: {params['acceleration']:.2f} m/s²")
    print(f"   📊 Turning Aggressiveness: {params['turning_aggressiveness']:.2f}")
    print(f"   📊 Obstacle Threshold: {params['obstacle_threshold']:.2f} m")
    print(f"   📊 Sensor Öncelik: {params['sensor_priority']}")
    
    # Stage ilerleme testi
    progress = controller.get_stage_progress()
    print(f"   📊 Stage İlerleme: %{progress:.1f}")
    
    # Özel davranış testi
    print("\n🎭 Özel Davranış Testi...")
    behavior_result = controller.execute_special_behavior('water_crossing')
    print(f"   🌊 Su Geçişi Davranışı: {behavior_result['status']}")
    if 'actions' in behavior_result:
        print(f"   📋 Aksiyonlar: {', '.join(behavior_result['actions'])}")
    
    print("\n✅ Adaptive Stage Controller test başarılı!")
    print("🔥 11 Parkur stage'i için adaptive control sistemi aktif!")
    
except ImportError as e:
    print(f"❌ Import hatası: {e}")
except Exception as e:
    print(f"❌ Test hatası: {e}")
    import traceback
    traceback.print_exc()

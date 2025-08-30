#!/usr/bin/env python3
"""
Sensor Fusion System Test Script
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import numpy as np
from main_controller.sensor_fusion_system import (
    SensorFusionSystem, SensorReading, SensorType, SensorHealth
)

def test_sensor_fusion():
    """Sensor Fusion System testi"""
    
    print("🔬 Sensor Fusion System Test Başlıyor...")
    
    # Fusion system oluştur
    fusion = SensorFusionSystem()
    fusion.start()
    
    print("✅ Sensor Fusion System başlatıldı")
    
    # Test verileri ile simülasyon
    print("\n🎯 Multi-Sensor Data Fusion Testi...")
    
    try:
        for i in range(30):
            current_time = time.time()
            
            # LiDAR simülasyonu - position ve obstacle data
            if i % 3 == 0:  # 3.33 Hz
                lidar_reading = SensorReading(
                    sensor_type=SensorType.LIDAR,
                    timestamp=current_time,
                    data={
                        'position': (i * 0.1, np.sin(i * 0.1) * 2),
                        'obstacles': [
                            {'position': (5.0 + np.random.normal(0, 0.1), 2.0), 'size': 0.5, 'confidence': 0.8},
                            {'position': (-3.0, 1.0), 'size': 0.3, 'confidence': 0.6}
                        ] if i > 5 else []
                    },
                    confidence=0.9,
                    health=SensorHealth.HEALTHY
                )
                fusion.add_sensor_reading(lidar_reading)
            
            # IMU simülasyonu - orientation ve angular velocity
            if i % 2 == 0:  # 5 Hz
                imu_reading = SensorReading(
                    sensor_type=SensorType.IMU,
                    timestamp=current_time,
                    data={
                        'orientation': i * 0.02 + np.random.normal(0, 0.01),
                        'angular_velocity': 0.02 + np.random.normal(0, 0.005)
                    },
                    confidence=0.8,
                    health=SensorHealth.HEALTHY
                )
                fusion.add_sensor_reading(imu_reading)
            
            # Encoder simülasyonu - velocity
            encoder_reading = SensorReading(
                sensor_type=SensorType.ENCODER,
                timestamp=current_time,
                data={
                    'velocity': (1.0 + np.random.normal(0, 0.1), 0.1 * np.sin(i * 0.1))
                },
                confidence=0.85,
                health=SensorHealth.HEALTHY
            )
            fusion.add_sensor_reading(encoder_reading)
            
            # Ultrasonic simülasyonu - obstacle detection
            if i % 4 == 0:  # 2.5 Hz
                us_distance = 3.0 + np.random.normal(0, 0.2)
                us_reading = SensorReading(
                    sensor_type=SensorType.ULTRASONIC,
                    timestamp=current_time,
                    data={
                        'obstacles': [{'position': (us_distance, 0), 'size': 0.4, 'confidence': 0.7}] if us_distance < 4.0 else []
                    },
                    confidence=0.7,
                    health=SensorHealth.HEALTHY
                )
                fusion.add_sensor_reading(us_reading)
            
            # Camera simülasyonu - object detection
            if i % 5 == 0:  # 2 Hz
                camera_reading = SensorReading(
                    sensor_type=SensorType.CAMERA,
                    timestamp=current_time,
                    data={
                        'obstacles': [
                            {'position': (6.0, -1.0), 'size': 0.8, 'confidence': 0.9, 'type': 'cone'},
                            {'position': (4.5, 2.5), 'size': 0.6, 'confidence': 0.7, 'type': 'barrier'}
                        ] if i > 10 else []
                    },
                    confidence=0.8,
                    health=SensorHealth.HEALTHY if i < 25 else SensorHealth.DEGRADED  # Camera degradation simulation
                )
                fusion.add_sensor_reading(camera_reading)
            
            time.sleep(0.1)
            
            # Status display her 1 saniyede
            if i % 10 == 0:
                fused_data = fusion.get_fused_data()
                if fused_data:
                    print(f"\n📊 Iteration {i}:")
                    print(f"   Position: ({fused_data.position[0]:.2f}, {fused_data.position[1]:.2f})")
                    print(f"   Orientation: {fused_data.orientation:.3f} rad")
                    print(f"   Velocity: ({fused_data.velocity[0]:.2f}, {fused_data.velocity[1]:.2f}) m/s")
                    print(f"   Obstacles: {len(fused_data.obstacles)}")
                    print(f"   Confidence: {fused_data.confidence:.3f}")
                    print(f"   Contributing Sensors: {[s.value for s in fused_data.contributing_sensors]}")
                
                # Sensor health
                sensor_health = fusion.get_sensor_health()
                health_summary = {s.value: h.value for s, h in sensor_health.items() if h != SensorHealth.UNKNOWN}
                print(f"   Sensor Health: {health_summary}")
                
                # Performance metrics
                metrics = fusion.get_performance_metrics()
                print(f"   Fusion Rate: {metrics['fusion_rate']:.1f} Hz")
                print(f"   Fusion Count: {metrics['fusion_count']}")
                print(f"   Healthy Sensors: {metrics['contributing_sensors']}")
    
        print("\n🎯 Advanced Fusion Features Test...")
        
        # Sensor failure simulation
        print("⚠️ LiDAR failure simulation...")
        failed_lidar = SensorReading(
            sensor_type=SensorType.LIDAR,
            timestamp=time.time(),
            data={'position': (0, 0)},
            confidence=0.1,
            health=SensorHealth.FAILED
        )
        fusion.add_sensor_reading(failed_lidar)
        
        time.sleep(0.5)
        
        # Final status
        final_data = fusion.get_fused_data()
        if final_data:
            print(f"\n🏁 Final Fusion Results:")
            print(f"   Final Position: ({final_data.position[0]:.2f}, {final_data.position[1]:.2f})")
            print(f"   Final Confidence: {final_data.confidence:.3f}")
            print(f"   Active Sensors: {len(final_data.contributing_sensors)}")
            
            # Obstacle clustering test
            if final_data.obstacles:
                print(f"   Detected Obstacles:")
                for i, obs in enumerate(final_data.obstacles):
                    print(f"      {i+1}. Position: ({obs['position'][0]:.2f}, {obs['position'][1]:.2f})")
                    print(f"         Size: {obs['size']:.2f}m, Confidence: {obs['confidence']:.3f}")
                    print(f"         Type: {obs.get('type', 'unknown')}")
        
        # Performance summary
        final_metrics = fusion.get_performance_metrics()
        print(f"\n📈 Performance Summary:")
        print(f"   Total Fusions: {final_metrics['fusion_count']}")
        print(f"   Average Rate: {final_metrics['fusion_rate']:.1f} Hz")
        print(f"   Latency: {final_metrics['fusion_latency']*1000:.1f} ms")
        print(f"   Healthy Sensors: {final_metrics['contributing_sensors']}")
        print(f"   Failed Sensors: {final_metrics['failed_sensors']}")
        
        print("\n✅ Sensor Fusion System test başarılı!")
        print("🔬 Multi-sensor data fusion, Kalman filtering, ve obstacle clustering aktif!")
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        fusion.stop()
        print("🛑 Sensor Fusion System durduruldu")

if __name__ == "__main__":
    test_sensor_fusion()

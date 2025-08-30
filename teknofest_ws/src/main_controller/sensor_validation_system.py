#!/usr/bin/env python3
"""
🔍 SENSOR VALIDATION SYSTEM
Real-time sensor health monitoring, cross-validation, and confidence scoring
"""

import json
import time
import numpy as np
from threading import Thread, Lock
from datetime import datetime, timedelta
from collections import deque
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import statistics
import logging

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

@dataclass
class SensorReading:
    """Single sensor reading with metadata"""
    sensor_id: str
    timestamp: float
    data: Dict
    confidence: float = 1.0
    validated: bool = False
    outlier_score: float = 0.0

@dataclass
class ValidationRule:
    """Sensor validation rule definition"""
    rule_id: str
    sensor_type: str
    parameter: str
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    max_change_rate: Optional[float] = None
    required_consistency: float = 0.8
    validation_window: int = 10

@dataclass
class SensorHealth:
    """Sensor health status"""
    sensor_id: str
    health_score: float = 1.0
    error_count: int = 0
    last_valid_reading: Optional[float] = None
    consecutive_failures: int = 0
    total_readings: int = 0
    valid_readings: int = 0
    avg_confidence: float = 1.0
    status: str = "healthy"  # healthy, degraded, failed, offline

class CrossValidator:
    """Cross-validates sensor readings using multiple sensors"""
    
    def __init__(self):
        self.position_sensors = ['lidar', 'camera', 'gps']
        self.distance_sensors = ['lidar', 'ultrasonic', 'camera']
        self.orientation_sensors = ['imu', 'encoder', 'gps']
        
    def validate_position(self, readings: Dict[str, SensorReading]) -> float:
        """Cross-validate position readings"""
        positions = {}
        for sensor_id, reading in readings.items():
            if reading.sensor_id in self.position_sensors and 'position' in reading.data:
                positions[sensor_id] = reading.data['position']
        
        if len(positions) < 2:
            return 0.5  # Low confidence without cross-validation
        
        # Calculate position variance
        x_coords = [pos['x'] for pos in positions.values()]
        y_coords = [pos['y'] for pos in positions.values()]
        
        if len(x_coords) >= 2:
            x_var = statistics.variance(x_coords)
            y_var = statistics.variance(y_coords)
            total_var = x_var + y_var
            
            # Lower variance = higher confidence
            confidence = max(0.1, 1.0 - min(total_var / 10.0, 0.9))
            return confidence
        
        return 0.5
    
    def validate_distance(self, readings: Dict[str, SensorReading], target_direction: str) -> float:
        """Cross-validate distance readings in specific direction"""
        distances = {}
        for sensor_id, reading in readings.items():
            if reading.sensor_id in self.distance_sensors:
                if 'obstacles' in reading.data:
                    obstacles = reading.data['obstacles']
                    # Find closest obstacle in target direction
                    for obstacle in obstacles:
                        if obstacle.get('direction') == target_direction:
                            distances[sensor_id] = obstacle['distance']
                            break
                elif 'distance' in reading.data:
                    distances[sensor_id] = reading.data['distance']
        
        if len(distances) < 2:
            return 0.6
        
        # Calculate distance variance
        distance_list = list(distances.values())
        if len(distance_list) >= 2:
            distance_var = statistics.variance(distance_list)
            confidence = max(0.1, 1.0 - min(distance_var / 5.0, 0.9))
            return confidence
        
        return 0.6
    
    def validate_orientation(self, readings: Dict[str, SensorReading]) -> float:
        """Cross-validate orientation readings"""
        orientations = {}
        for sensor_id, reading in readings.items():
            if reading.sensor_id in self.orientation_sensors and 'orientation' in reading.data:
                orientations[sensor_id] = reading.data['orientation']
        
        if len(orientations) < 2:
            return 0.5
        
        # Calculate orientation variance (handle angle wrapping)
        angles = [ori['yaw'] for ori in orientations.values()]
        if len(angles) >= 2:
            # Convert to unit vectors to handle angle wrapping
            x_components = [np.cos(np.radians(angle)) for angle in angles]
            y_components = [np.sin(np.radians(angle)) for angle in angles]
            
            x_var = statistics.variance(x_components)
            y_var = statistics.variance(y_components)
            total_var = x_var + y_var
            
            confidence = max(0.1, 1.0 - min(total_var * 10.0, 0.9))
            return confidence
        
        return 0.5

class OutlierDetector:
    """Detects outlier readings using statistical methods"""
    
    def __init__(self, window_size: int = 20):
        self.window_size = window_size
        self.history = {}
    
    def add_reading(self, sensor_id: str, value: float):
        """Add reading to history"""
        if sensor_id not in self.history:
            self.history[sensor_id] = deque(maxlen=self.window_size)
        self.history[sensor_id].append(value)
    
    def is_outlier(self, sensor_id: str, value: float, threshold: float = 2.0) -> Tuple[bool, float]:
        """Detect if value is outlier using z-score"""
        if sensor_id not in self.history or len(self.history[sensor_id]) < 3:
            return False, 0.0
        
        history_list = list(self.history[sensor_id])
        mean = statistics.mean(history_list)
        stdev = statistics.stdev(history_list) if len(history_list) > 1 else 0.1
        
        z_score = abs((value - mean) / stdev) if stdev > 0 else 0
        is_outlier = z_score > threshold
        
        return is_outlier, z_score
    
    def detect_rapid_change(self, sensor_id: str, value: float, max_change_rate: float) -> bool:
        """Detect rapid changes in sensor readings"""
        if sensor_id not in self.history or len(self.history[sensor_id]) == 0:
            return False
        
        last_value = self.history[sensor_id][-1]
        change_rate = abs(value - last_value)
        
        return change_rate > max_change_rate

class SensorValidationSystem:
    """Main sensor validation system"""
    
    def __init__(self, config_file: str = None):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # System state
        self.active = False
        self.validation_thread = None
        self.lock = Lock()
        
        # Components
        self.cross_validator = CrossValidator()
        self.outlier_detector = OutlierDetector()
        
        # Sensor data
        self.sensor_readings = {}
        self.sensor_health = {}
        self.validation_rules = {}
        
        # Performance metrics
        self.validation_count = 0
        self.invalid_count = 0
        self.outlier_count = 0
        self.start_time = time.time()
        
        # Load configuration
        self.load_validation_rules()
        
        self.logger.info("Sensor Validation System initialized")
    
    def load_validation_rules(self):
        """Load sensor validation rules"""
        # Default validation rules
        default_rules = [
            ValidationRule("lidar_range", "lidar", "distance", 0.1, 10.0, 5.0),
            ValidationRule("camera_conf", "camera", "confidence", 0.1, 1.0, 0.5),
            ValidationRule("imu_accel", "imu", "acceleration", -20.0, 20.0, 15.0),
            ValidationRule("ultrasonic_range", "ultrasonic", "distance", 0.02, 4.0, 3.0),
            ValidationRule("encoder_speed", "encoder", "speed", -5.0, 5.0, 3.0),
            ValidationRule("gps_accuracy", "gps", "accuracy", 0.0, 10.0, 5.0)
        ]
        
        for rule in default_rules:
            self.validation_rules[rule.rule_id] = rule
        
        self.logger.info(f"Loaded {len(default_rules)} validation rules")
    
    def start_validation(self):
        """Start sensor validation system"""
        if self.active:
            return
        
        self.active = True
        self.validation_thread = Thread(target=self._validation_loop, daemon=True)
        self.validation_thread.start()
        self.logger.info("Sensor validation started")
    
    def stop_validation(self):
        """Stop sensor validation system"""
        self.active = False
        if self.validation_thread:
            self.validation_thread.join(timeout=1.0)
        self.logger.info("Sensor validation stopped")
    
    def add_sensor_reading(self, sensor_id: str, data: Dict) -> SensorReading:
        """Add new sensor reading for validation"""
        with self.lock:
            reading = SensorReading(
                sensor_id=sensor_id,
                timestamp=time.time(),
                data=data
            )
            
            # Store reading
            self.sensor_readings[sensor_id] = reading
            
            # Initialize sensor health if new
            if sensor_id not in self.sensor_health:
                self.sensor_health[sensor_id] = SensorHealth(sensor_id)
            
            # Update sensor health stats
            health = self.sensor_health[sensor_id]
            health.total_readings += 1
            health.last_valid_reading = reading.timestamp
            
            return reading
    
    def validate_reading(self, reading: SensorReading) -> SensorReading:
        """Validate a single sensor reading"""
        with self.lock:
            sensor_id = reading.sensor_id
            health = self.sensor_health.get(sensor_id)
            if not health:
                return reading
            
            validation_score = 1.0
            issues = []
            
            # Rule-based validation
            for rule in self.validation_rules.values():
                if rule.sensor_type == sensor_id or rule.sensor_type in sensor_id:
                    rule_score, rule_issues = self._validate_against_rule(reading, rule)
                    validation_score = min(validation_score, rule_score)
                    issues.extend(rule_issues)
            
            # Outlier detection
            if 'value' in reading.data:
                value = reading.data['value']
                self.outlier_detector.add_reading(sensor_id, value)
                is_outlier, outlier_score = self.outlier_detector.is_outlier(sensor_id, value)
                
                reading.outlier_score = outlier_score
                if is_outlier:
                    validation_score *= 0.5
                    issues.append(f"Outlier detected (z-score: {outlier_score:.2f})")
                    self.outlier_count += 1
            
            # Cross-validation
            cross_validation_score = self._perform_cross_validation(reading)
            validation_score = min(validation_score, cross_validation_score)
            
            # Update reading
            reading.confidence = validation_score
            reading.validated = True
            
            # Update sensor health
            self._update_sensor_health(sensor_id, validation_score, issues)
            
            self.validation_count += 1
            if validation_score < 0.5:
                self.invalid_count += 1
            
            return reading
    
    def _validate_against_rule(self, reading: SensorReading, rule: ValidationRule) -> Tuple[float, List[str]]:
        """Validate reading against specific rule"""
        if rule.parameter not in reading.data:
            return 1.0, []
        
        value = reading.data[rule.parameter]
        issues = []
        score = 1.0
        
        # Range check
        if rule.min_value is not None and value < rule.min_value:
            score = 0.1
            issues.append(f"{rule.parameter} below minimum ({value} < {rule.min_value})")
        
        if rule.max_value is not None and value > rule.max_value:
            score = 0.1
            issues.append(f"{rule.parameter} above maximum ({value} > {rule.max_value})")
        
        # Change rate check
        if rule.max_change_rate is not None:
            if self.outlier_detector.detect_rapid_change(reading.sensor_id, value, rule.max_change_rate):
                score *= 0.3
                issues.append(f"Rapid change detected in {rule.parameter}")
        
        return score, issues
    
    def _perform_cross_validation(self, reading: SensorReading) -> float:
        """Perform cross-validation with other sensors"""
        cross_validation_score = 1.0
        
        # Position cross-validation
        if 'position' in reading.data:
            cross_validation_score = min(cross_validation_score, 
                                       self.cross_validator.validate_position(self.sensor_readings))
        
        # Distance cross-validation
        if 'distance' in reading.data or 'obstacles' in reading.data:
            for direction in ['front', 'left', 'right', 'back']:
                direction_score = self.cross_validator.validate_distance(self.sensor_readings, direction)
                cross_validation_score = min(cross_validation_score, direction_score)
        
        # Orientation cross-validation
        if 'orientation' in reading.data:
            cross_validation_score = min(cross_validation_score,
                                       self.cross_validator.validate_orientation(self.sensor_readings))
        
        return cross_validation_score
    
    def _update_sensor_health(self, sensor_id: str, validation_score: float, issues: List[str]):
        """Update sensor health based on validation results"""
        health = self.sensor_health[sensor_id]
        
        # Update validation stats
        if validation_score >= 0.5:
            health.valid_readings += 1
            health.consecutive_failures = 0
        else:
            health.error_count += 1
            health.consecutive_failures += 1
        
        # Update average confidence
        total_confidence = health.avg_confidence * (health.total_readings - 1) + validation_score
        health.avg_confidence = total_confidence / health.total_readings
        
        # Update health score (weighted average)
        health.health_score = 0.7 * (health.valid_readings / health.total_readings) + 0.3 * health.avg_confidence
        
        # Determine status
        if health.consecutive_failures > 10:
            health.status = "failed"
        elif health.consecutive_failures > 5:
            health.status = "degraded"
        elif health.health_score < 0.3:
            health.status = "degraded"
        elif time.time() - health.last_valid_reading > 5.0:
            health.status = "offline"
        else:
            health.status = "healthy"
        
        # Log critical issues
        if health.status in ["failed", "degraded"] and issues:
            self.logger.warning(f"Sensor {sensor_id} health: {health.status} - {'; '.join(issues)}")
    
    def _validation_loop(self):
        """Main validation loop"""
        while self.active:
            try:
                with self.lock:
                    # Validate recent readings
                    current_time = time.time()
                    for sensor_id, reading in list(self.sensor_readings.items()):
                        if not reading.validated and current_time - reading.timestamp < 1.0:
                            self.validate_reading(reading)
                
                time.sleep(0.05)  # 20Hz validation rate
                
            except Exception as e:
                self.logger.error(f"Validation loop error: {e}")
                time.sleep(0.1)
    
    def get_sensor_health_status(self) -> Dict:
        """Get current sensor health status"""
        with self.lock:
            status = {
                'healthy_sensors': [],
                'degraded_sensors': [],
                'failed_sensors': [],
                'offline_sensors': [],
                'overall_health': 0.0,
                'validation_stats': {
                    'total_validations': self.validation_count,
                    'invalid_readings': self.invalid_count,
                    'outliers_detected': self.outlier_count,
                    'validation_rate': self.validation_count / max(time.time() - self.start_time, 1),
                    'accuracy_rate': (self.validation_count - self.invalid_count) / max(self.validation_count, 1) * 100
                }
            }
            
            total_health = 0.0
            sensor_count = 0
            
            for sensor_id, health in self.sensor_health.items():
                health_info = {
                    'sensor_id': sensor_id,
                    'health_score': round(health.health_score, 3),
                    'avg_confidence': round(health.avg_confidence, 3),
                    'error_rate': round(health.error_count / max(health.total_readings, 1) * 100, 1),
                    'consecutive_failures': health.consecutive_failures,
                    'last_reading': health.last_valid_reading
                }
                
                if health.status == 'healthy':
                    status['healthy_sensors'].append(health_info)
                elif health.status == 'degraded':
                    status['degraded_sensors'].append(health_info)
                elif health.status == 'failed':
                    status['failed_sensors'].append(health_info)
                else:  # offline
                    status['offline_sensors'].append(health_info)
                
                total_health += health.health_score
                sensor_count += 1
            
            status['overall_health'] = round(total_health / max(sensor_count, 1), 3)
            
            return status
    
    def get_validation_confidence(self, sensor_types: List[str] = None) -> float:
        """Get overall validation confidence for specified sensor types"""
        with self.lock:
            if not sensor_types:
                sensor_types = list(self.sensor_health.keys())
            
            total_confidence = 0.0
            count = 0
            
            for sensor_id in sensor_types:
                if sensor_id in self.sensor_health:
                    health = self.sensor_health[sensor_id]
                    total_confidence += health.avg_confidence
                    count += 1
            
            return total_confidence / max(count, 1)

def test_sensor_validation_system():
    """Test the sensor validation system"""
    print("🔍 Testing Sensor Validation System...")
    
    # Initialize system
    validator = SensorValidationSystem()
    validator.start_validation()
    
    try:
        # Simulate sensor readings
        test_sensors = ['lidar', 'camera', 'imu', 'ultrasonic']
        
        for i in range(50):
            # Add normal readings
            for sensor in test_sensors:
                if sensor == 'lidar':
                    data = {
                        'distance': 2.5 + np.random.normal(0, 0.1),
                        'obstacles': [{'direction': 'front', 'distance': 2.5 + np.random.normal(0, 0.1)}],
                        'position': {'x': i * 0.1, 'y': 0.0}
                    }
                elif sensor == 'camera':
                    data = {
                        'confidence': 0.85 + np.random.normal(0, 0.05),
                        'obstacles': [{'direction': 'front', 'distance': 2.6 + np.random.normal(0, 0.2)}],
                        'position': {'x': i * 0.1 + np.random.normal(0, 0.05), 'y': np.random.normal(0, 0.02)}
                    }
                elif sensor == 'imu':
                    data = {
                        'acceleration': np.random.normal(0, 1.0),
                        'orientation': {'yaw': i * 2.0 + np.random.normal(0, 5)}
                    }
                else:  # ultrasonic
                    data = {
                        'distance': 2.4 + np.random.normal(0, 0.3),
                        'value': 2.4 + np.random.normal(0, 0.3)
                    }
                
                # Add some outliers
                if i > 20 and i % 15 == 0:
                    if sensor == 'lidar':
                        data['distance'] = 15.0  # Outlier
                    elif sensor == 'camera':
                        data['confidence'] = 0.1  # Low confidence
                
                reading = validator.add_sensor_reading(sensor, data)
                validator.validate_reading(reading)
            
            time.sleep(0.02)  # 50Hz simulation
            
            # Print status every 10 iterations
            if i % 10 == 0:
                status = validator.get_sensor_health_status()
                print(f"\n--- Iteration {i} Status ---")
                print(f"Overall Health: {status['overall_health']}")
                print(f"Healthy Sensors: {len(status['healthy_sensors'])}")
                print(f"Degraded Sensors: {len(status['degraded_sensors'])}")
                print(f"Failed Sensors: {len(status['failed_sensors'])}")
                print(f"Validation Accuracy: {status['validation_stats']['accuracy_rate']:.1f}%")
                print(f"Outliers Detected: {status['validation_stats']['outliers_detected']}")
                
                # Show individual sensor health
                for sensor_info in status['healthy_sensors'][:2]:  # Show first 2
                    print(f"  {sensor_info['sensor_id']}: Health={sensor_info['health_score']:.3f}, "
                          f"Confidence={sensor_info['avg_confidence']:.3f}")
        
        # Final results
        final_status = validator.get_sensor_health_status()
        print("\n🎯 FINAL VALIDATION RESULTS:")
        print(f"✅ Overall System Health: {final_status['overall_health']:.3f}")
        print(f"📊 Total Validations: {final_status['validation_stats']['total_validations']}")
        print(f"🎯 Validation Accuracy: {final_status['validation_stats']['accuracy_rate']:.1f}%")
        print(f"⚠️  Outliers Detected: {final_status['validation_stats']['outliers_detected']}")
        print(f"📈 Validation Rate: {final_status['validation_stats']['validation_rate']:.1f} Hz")
        
        print(f"\n🟢 Healthy Sensors: {len(final_status['healthy_sensors'])}")
        print(f"🟡 Degraded Sensors: {len(final_status['degraded_sensors'])}")
        print(f"🔴 Failed Sensors: {len(final_status['failed_sensors'])}")
        print(f"⚫ Offline Sensors: {len(final_status['offline_sensors'])}")
        
        # Test confidence scoring
        confidence = validator.get_validation_confidence(['lidar', 'camera'])
        print(f"\n🔍 Cross-validation confidence (LiDAR + Camera): {confidence:.3f}")
        
        print("\n✅ Sensor Validation System test completed successfully!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise
    
    finally:
        validator.stop_validation()

if __name__ == "__main__":
    test_sensor_validation_system()

#!/usr/bin/env python3
"""
Sensor Fusion System for Teknofest Autonomous Vehicle
Advanced multi-sensor data fusion using Kalman Filter and weighted averaging
"""

import numpy as np
import time
import logging
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import threading
from collections import deque
import json

class SensorType(Enum):
    """Sensor tipleri"""
    LIDAR = "lidar"
    CAMERA = "camera"
    IMU = "imu"
    ULTRASONIC = "ultrasonic"
    ENCODER = "encoder"
    GPS = "gps"

class SensorHealth(Enum):
    """Sensor sağlık durumu"""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    UNKNOWN = "unknown"

@dataclass
class SensorReading:
    """Sensor okuma verisi"""
    sensor_type: SensorType
    timestamp: float
    data: Dict[str, Any]
    confidence: float = 1.0
    health: SensorHealth = SensorHealth.HEALTHY
    
@dataclass
class FusedData:
    """Birleştirilmiş sensor verisi"""
    timestamp: float
    position: Tuple[float, float]  # (x, y)
    orientation: float  # yaw angle
    velocity: Tuple[float, float]  # (vx, vy)
    obstacles: List[Dict[str, Any]]
    confidence: float
    contributing_sensors: List[SensorType]

class KalmanFilter:
    """Extended Kalman Filter for vehicle state estimation"""
    
    def __init__(self):
        """Initialize Kalman Filter"""
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1  # Covariance matrix
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
        
        # Measurement noise (will be set per sensor)
        self.R_lidar = np.diag([0.1, 0.1])
        self.R_imu = np.diag([0.05, 0.02])
        self.R_encoder = np.diag([0.02, 0.02])
        
        self.dt = 0.1  # Time step
        self.last_update = time.time()
        
    def predict(self, dt: float = None):
        """Prediction step"""
        if dt is None:
            dt = self.dt
            
        # State transition matrix F
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_lidar(self, measurement: np.ndarray):
        """Update with LiDAR measurement [x, y]"""
        H = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0]])
        
        self._update(measurement, H, self.R_lidar)
        
    def update_imu(self, measurement: np.ndarray):
        """Update with IMU measurement [theta, omega]"""
        H = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1]])
        
        self._update(measurement, H, self.R_imu)
        
    def update_encoder(self, measurement: np.ndarray):
        """Update with encoder measurement [vx, vy]"""
        H = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0]])
        
        self._update(measurement, H, self.R_encoder)
        
    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """Generic update step"""
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        I_KH = np.eye(len(self.state)) - K @ H
        self.P = I_KH @ self.P
        
    def get_state(self) -> Dict[str, float]:
        """Get current state estimate"""
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2],
            'vx': self.state[3],
            'vy': self.state[4],
            'omega': self.state[5]
        }

class SensorFusionSystem:
    """
    Multi-sensor fusion system for autonomous vehicle
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize sensor fusion system
        
        Args:
            config_path: Path to configuration file
        """
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # Kalman filter for state estimation
        self.kalman = KalmanFilter()
        
        # Sensor data buffers
        self.sensor_buffers = {
            sensor_type: deque(maxlen=100) for sensor_type in SensorType
        }
        
        # Sensor health monitoring
        self.sensor_health = {
            sensor_type: SensorHealth.UNKNOWN for sensor_type in SensorType
        }
        
        # Sensor weights (confidence factors)
        self.sensor_weights = {
            SensorType.LIDAR: 0.9,
            SensorType.CAMERA: 0.8,
            SensorType.IMU: 0.7,
            SensorType.ULTRASONIC: 0.6,
            SensorType.ENCODER: 0.8,
            SensorType.GPS: 0.5
        }
        
        # Fusion configuration
        self.config = self._load_config(config_path)
        
        # Threading
        self.running = False
        self.fusion_thread = None
        self.lock = threading.Lock()
        
        # Latest fused data
        self.latest_fused_data = None
        self.fusion_rate = 10.0  # Hz
        
        # Performance metrics
        self.fusion_count = 0
        self.last_fusion_time = 0
        self.fusion_latency = 0
        
        self.logger.info("🔬 Sensor Fusion System initialized")
        
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load fusion configuration"""
        default_config = {
            "fusion_rate": 10.0,
            "max_sensor_age": 1.0,
            "min_confidence_threshold": 0.3,
            "obstacle_detection_threshold": 0.5,
            "position_fusion_method": "kalman",
            "obstacle_fusion_method": "weighted_average"
        }
        
        if config_path:
            try:
                with open(config_path, 'r') as f:
                    user_config = json.load(f)
                default_config.update(user_config)
            except Exception as e:
                self.logger.warning(f"⚠️ Config yüklenemedi, default kullanılıyor: {e}")
                
        return default_config
    
    def start(self):
        """Start fusion system"""
        if not self.running:
            self.running = True
            self.fusion_thread = threading.Thread(target=self._fusion_loop, daemon=True)
            self.fusion_thread.start()
            self.logger.info("🚀 Sensor fusion başlatıldı")
    
    def stop(self):
        """Stop fusion system"""
        self.running = False
        if self.fusion_thread:
            self.fusion_thread.join(timeout=2.0)
        self.logger.info("🛑 Sensor fusion durduruldu")
    
    def add_sensor_reading(self, reading: SensorReading):
        """
        Add new sensor reading
        
        Args:
            reading: Sensor reading data
        """
        with self.lock:
            # Add to buffer
            self.sensor_buffers[reading.sensor_type].append(reading)
            
            # Update sensor health
            self.sensor_health[reading.sensor_type] = reading.health
            
            # Update Kalman filter if applicable
            self._update_kalman_filter(reading)
    
    def _update_kalman_filter(self, reading: SensorReading):
        """Update Kalman filter with new reading"""
        try:
            if reading.sensor_type == SensorType.LIDAR:
                if 'position' in reading.data:
                    pos = reading.data['position']
                    measurement = np.array([pos[0], pos[1]])
                    self.kalman.update_lidar(measurement)
                    
            elif reading.sensor_type == SensorType.IMU:
                if 'orientation' in reading.data and 'angular_velocity' in reading.data:
                    measurement = np.array([
                        reading.data['orientation'],
                        reading.data['angular_velocity']
                    ])
                    self.kalman.update_imu(measurement)
                    
            elif reading.sensor_type == SensorType.ENCODER:
                if 'velocity' in reading.data:
                    vel = reading.data['velocity']
                    measurement = np.array([vel[0], vel[1]])
                    self.kalman.update_encoder(measurement)
                    
        except Exception as e:
            self.logger.error(f"❌ Kalman güncelleme hatası {reading.sensor_type.value}: {e}")
    
    def _fusion_loop(self):
        """Main fusion loop"""
        while self.running:
            start_time = time.time()
            
            try:
                # Predict Kalman filter
                current_time = time.time()
                dt = current_time - self.kalman.last_update
                self.kalman.predict(dt)
                self.kalman.last_update = current_time
                
                # Perform sensor fusion
                fused_data = self._fuse_sensor_data()
                
                if fused_data:
                    with self.lock:
                        self.latest_fused_data = fused_data
                        self.fusion_count += 1
                        
                    # Calculate performance metrics
                    self.fusion_latency = time.time() - start_time
                    
            except Exception as e:
                self.logger.error(f"❌ Fusion loop hatası: {e}")
            
            # Sleep to maintain rate
            sleep_time = 1.0 / self.fusion_rate - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _fuse_sensor_data(self) -> Optional[FusedData]:
        """
        Fuse sensor data from all sources
        
        Returns:
            FusedData: Fused sensor information
        """
        current_time = time.time()
        max_age = self.config["max_sensor_age"]
        
        # Get fresh sensor data
        fresh_readings = {}
        contributing_sensors = []
        
        for sensor_type, buffer in self.sensor_buffers.items():
            if buffer:
                # Get most recent reading
                latest_reading = buffer[-1]
                age = current_time - latest_reading.timestamp
                
                if age <= max_age and latest_reading.confidence >= self.config["min_confidence_threshold"]:
                    fresh_readings[sensor_type] = latest_reading
                    contributing_sensors.append(sensor_type)
        
        if not fresh_readings:
            return None
        
        # Position fusion using Kalman filter
        kalman_state = self.kalman.get_state()
        position = (kalman_state['x'], kalman_state['y'])
        orientation = kalman_state['theta']
        velocity = (kalman_state['vx'], kalman_state['vy'])
        
        # Obstacle fusion using weighted average
        obstacles = self._fuse_obstacles(fresh_readings)
        
        # Calculate overall confidence
        confidence = self._calculate_confidence(fresh_readings, contributing_sensors)
        
        return FusedData(
            timestamp=current_time,
            position=position,
            orientation=orientation,
            velocity=velocity,
            obstacles=obstacles,
            confidence=confidence,
            contributing_sensors=contributing_sensors
        )
    
    def _fuse_obstacles(self, readings: Dict[SensorType, SensorReading]) -> List[Dict[str, Any]]:
        """
        Fuse obstacle data from multiple sensors
        
        Args:
            readings: Fresh sensor readings
            
        Returns:
            List: Fused obstacle data
        """
        obstacles = []
        
        # Collect all obstacle detections
        all_detections = []
        
        for sensor_type, reading in readings.items():
            if 'obstacles' in reading.data:
                sensor_obstacles = reading.data['obstacles']
                weight = self.sensor_weights[sensor_type] * reading.confidence
                
                for obs in sensor_obstacles:
                    detection = {
                        'position': obs.get('position', (0, 0)),
                        'size': obs.get('size', 0.5),
                        'confidence': obs.get('confidence', 0.5) * weight,
                        'sensor': sensor_type.value,
                        'type': obs.get('type', 'unknown')
                    }
                    all_detections.append(detection)
        
        # Cluster nearby detections
        clustered = self._cluster_detections(all_detections)
        
        # Filter by confidence
        threshold = self.config["obstacle_detection_threshold"]
        obstacles = [obs for obs in clustered if obs['confidence'] >= threshold]
        
        return obstacles
    
    def _cluster_detections(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Cluster nearby obstacle detections
        
        Args:
            detections: Raw obstacle detections
            
        Returns:
            List: Clustered obstacles
        """
        if not detections:
            return []
        
        clusters = []
        cluster_radius = 1.0  # meters
        
        for detection in detections:
            # Find nearby clusters
            nearby_cluster = None
            min_distance = float('inf')
            
            for cluster in clusters:
                distance = np.linalg.norm(
                    np.array(detection['position']) - np.array(cluster['position'])
                )
                if distance < cluster_radius and distance < min_distance:
                    nearby_cluster = cluster
                    min_distance = distance
            
            if nearby_cluster:
                # Merge with existing cluster
                total_confidence = nearby_cluster['confidence'] + detection['confidence']
                
                # Weighted average position
                w1 = nearby_cluster['confidence'] / total_confidence
                w2 = detection['confidence'] / total_confidence
                
                new_pos = (
                    w1 * nearby_cluster['position'][0] + w2 * detection['position'][0],
                    w1 * nearby_cluster['position'][1] + w2 * detection['position'][1]
                )
                
                nearby_cluster['position'] = new_pos
                nearby_cluster['confidence'] = total_confidence
                nearby_cluster['size'] = max(nearby_cluster['size'], detection['size'])
            else:
                # Create new cluster
                clusters.append(detection.copy())
        
        return clusters
    
    def _calculate_confidence(self, readings: Dict[SensorType, SensorReading], 
                            contributing_sensors: List[SensorType]) -> float:
        """
        Calculate overall confidence of fused data
        
        Args:
            readings: Sensor readings
            contributing_sensors: Sensors that contributed to fusion
            
        Returns:
            float: Overall confidence [0-1]
        """
        if not contributing_sensors:
            return 0.0
        
        # Weighted average of sensor confidences
        total_weight = 0
        weighted_confidence = 0
        
        for sensor_type in contributing_sensors:
            if sensor_type in readings:
                weight = self.sensor_weights[sensor_type]
                confidence = readings[sensor_type].confidence
                
                # Health penalty
                health = self.sensor_health[sensor_type]
                if health == SensorHealth.DEGRADED:
                    weight *= 0.7
                elif health == SensorHealth.FAILED:
                    weight *= 0.1
                
                weighted_confidence += weight * confidence
                total_weight += weight
        
        if total_weight > 0:
            base_confidence = weighted_confidence / total_weight
        else:
            base_confidence = 0.0
        
        # Diversity bonus (more sensors = higher confidence)
        diversity_bonus = min(0.2, len(contributing_sensors) * 0.05)
        
        return min(1.0, base_confidence + diversity_bonus)
    
    def get_fused_data(self) -> Optional[FusedData]:
        """
        Get latest fused sensor data
        
        Returns:
            FusedData: Latest fused data or None
        """
        with self.lock:
            return self.latest_fused_data
    
    def get_sensor_health(self) -> Dict[SensorType, SensorHealth]:
        """
        Get sensor health status
        
        Returns:
            Dict: Sensor health mapping
        """
        with self.lock:
            return self.sensor_health.copy()
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get fusion performance metrics
        
        Returns:
            Dict: Performance metrics
        """
        return {
            'fusion_count': self.fusion_count,
            'fusion_rate': self.fusion_count / (time.time() - self.last_fusion_time) if self.last_fusion_time > 0 else 0,
            'fusion_latency': self.fusion_latency,
            'contributing_sensors': len([h for h in self.sensor_health.values() if h == SensorHealth.HEALTHY]),
            'failed_sensors': len([h for h in self.sensor_health.values() if h == SensorHealth.FAILED])
        }
    
    def reset_filters(self):
        """Reset all filters and buffers"""
        with self.lock:
            self.kalman = KalmanFilter()
            for buffer in self.sensor_buffers.values():
                buffer.clear()
            self.latest_fused_data = None
            self.logger.info("🔄 Sensor fusion filters reset")

def main():
    """Test function"""
    logging.basicConfig(level=logging.INFO)
    
    # Create fusion system
    fusion = SensorFusionSystem()
    fusion.start()
    
    # Simulate sensor readings
    try:
        for i in range(50):
            # Simulate LiDAR reading
            lidar_reading = SensorReading(
                sensor_type=SensorType.LIDAR,
                timestamp=time.time(),
                data={
                    'position': (i * 0.1, 0.0),
                    'obstacles': [{'position': (5.0, 2.0), 'size': 0.5, 'confidence': 0.8}]
                },
                confidence=0.9
            )
            fusion.add_sensor_reading(lidar_reading)
            
            # Simulate IMU reading
            imu_reading = SensorReading(
                sensor_type=SensorType.IMU,
                timestamp=time.time(),
                data={
                    'orientation': i * 0.01,
                    'angular_velocity': 0.02
                },
                confidence=0.8
            )
            fusion.add_sensor_reading(imu_reading)
            
            time.sleep(0.1)
            
            # Get fused data
            fused = fusion.get_fused_data()
            if fused and i % 10 == 0:
                print(f"Fused Position: ({fused.position[0]:.2f}, {fused.position[1]:.2f})")
                print(f"Confidence: {fused.confidence:.2f}")
                print(f"Obstacles: {len(fused.obstacles)}")
                print("-" * 30)
                
    finally:
        fusion.stop()

if __name__ == "__main__":
    main()

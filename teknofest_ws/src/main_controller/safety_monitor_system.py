#!/usr/bin/env python3
"""
Safety Monitor System for Teknofest Autonomous Vehicle
Multi-level safety monitoring with emergency response capabilities
"""

import threading
import time
import logging
from datetime import datetime, timedelta
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Callable
from dataclasses import dataclass
import numpy as np
from collections import deque
import json

class SafetyLevel(Enum):
    """Güvenlik seviyeleri"""
    SAFE = "safe"
    CAUTION = "caution" 
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"

class SafetyEvent(Enum):
    """Güvenlik olayları"""
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    SENSOR_FAILURE = "sensor_failure"
    COMMUNICATION_LOSS = "communication_loss"
    EXCESSIVE_TILT = "excessive_tilt"
    HIGH_TEMPERATURE = "high_temperature"
    LOW_BATTERY = "low_battery"
    SPEED_VIOLATION = "speed_violation"
    STUCK_VEHICLE = "stuck_vehicle"
    WATER_DETECTION = "water_detection"
    GEOFENCE_VIOLATION = "geofence_violation"

@dataclass
class SafetyRule:
    """Güvenlik kuralı tanımı"""
    name: str
    event_type: SafetyEvent
    condition: str  # Python expression
    level: SafetyLevel
    response_action: str  # Action to take
    timeout: float = 5.0  # Timeout for rule evaluation
    enabled: bool = True

@dataclass
class SafetyIncident:
    """Güvenlik olayı kaydı"""
    timestamp: float
    event_type: SafetyEvent
    level: SafetyLevel
    description: str
    sensor_data: Dict[str, Any]
    response_taken: str
    resolved: bool = False
    resolution_time: Optional[float] = None

class SafetyMonitorSystem:
    """
    Comprehensive safety monitoring system for autonomous vehicle
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize safety monitor system
        
        Args:
            config_path: Path to safety configuration file
        """
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # Safety configuration
        self.config = self._load_config(config_path)
        
        # Current safety status
        self.current_level = SafetyLevel.SAFE
        self.active_incidents = []
        self.incident_history = deque(maxlen=1000)
        
        # Safety rules
        self.safety_rules = self._initialize_safety_rules()
        
        # Monitoring data
        self.sensor_data = {}
        self.vehicle_state = {}
        self.environment_data = {}
        
        # Threading
        self.running = False
        self.monitor_thread = None
        self.lock = threading.Lock()
        
        # Safety callbacks
        self.safety_callbacks = {}
        
        # Performance metrics
        self.total_incidents = 0
        self.false_positives = 0
        self.response_times = deque(maxlen=100)
        
        # Geofence (safety boundaries)
        self.geofence_boundaries = self.config.get('geofence', {
            'min_x': -50, 'max_x': 50,
            'min_y': -50, 'max_y': 50
        })
        
        self.logger.info("🛡️ Safety Monitor System initialized")
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load safety configuration"""
        default_config = {
            "monitor_frequency": 20.0,  # Hz
            "max_tilt_angle": 30.0,     # degrees
            "max_temperature": 60.0,    # Celsius
            "min_battery_voltage": 10.0, # Volts
            "max_speed": 5.0,           # m/s
            "obstacle_warning_distance": 2.0,  # meters
            "obstacle_critical_distance": 0.5, # meters
            "stuck_threshold": 0.1,     # m/s for 10 seconds
            "communication_timeout": 5.0, # seconds
            "water_resistance_depth": 0.3, # meters
            "response_timeout": 2.0,    # seconds for emergency response
            "geofence": {
                "min_x": -50, "max_x": 50,
                "min_y": -50, "max_y": 50
            }
        }
        
        if config_path:
            try:
                with open(config_path, 'r') as f:
                    user_config = json.load(f)
                default_config.update(user_config)
            except Exception as e:
                self.logger.warning(f"⚠️ Safety config yüklenemedi, default kullanılıyor: {e}")
        
        return default_config
    
    def _initialize_safety_rules(self) -> List[SafetyRule]:
        """Initialize safety rules"""
        rules = [
            # Obstacle detection rules
            SafetyRule(
                name="Obstacle Warning",
                event_type=SafetyEvent.OBSTACLE_TOO_CLOSE,
                condition="min(sensor_data.get('obstacles', [{'distance': 999}]), key=lambda x: x.get('distance', 999)).get('distance', 999) < config['obstacle_warning_distance']",
                level=SafetyLevel.WARNING,
                response_action="slow_down"
            ),
            SafetyRule(
                name="Obstacle Critical",
                event_type=SafetyEvent.OBSTACLE_TOO_CLOSE,
                condition="min(sensor_data.get('obstacles', [{'distance': 999}]), key=lambda x: x.get('distance', 999)).get('distance', 999) < config['obstacle_critical_distance']",
                level=SafetyLevel.EMERGENCY,
                response_action="emergency_stop"
            ),
            
            # Tilt monitoring
            SafetyRule(
                name="Excessive Tilt",
                event_type=SafetyEvent.EXCESSIVE_TILT,
                condition="abs(sensor_data.get('imu_pitch', 0)) > config['max_tilt_angle'] or abs(sensor_data.get('imu_roll', 0)) > config['max_tilt_angle']",
                level=SafetyLevel.CRITICAL,
                response_action="stabilize_vehicle"
            ),
            
            # Temperature monitoring
            SafetyRule(
                name="High Temperature",
                event_type=SafetyEvent.HIGH_TEMPERATURE,
                condition="sensor_data.get('temperature', 0) > config['max_temperature']",
                level=SafetyLevel.WARNING,
                response_action="thermal_protection"
            ),
            
            # Battery monitoring
            SafetyRule(
                name="Low Battery",
                event_type=SafetyEvent.LOW_BATTERY,
                condition="sensor_data.get('battery_voltage', 12) < config['min_battery_voltage']",
                level=SafetyLevel.CAUTION,
                response_action="power_conservation"
            ),
            
            # Speed monitoring  
            SafetyRule(
                name="Speed Violation",
                event_type=SafetyEvent.SPEED_VIOLATION,
                condition="vehicle_state.get('speed', 0) > config['max_speed']",
                level=SafetyLevel.WARNING,
                response_action="enforce_speed_limit"
            ),
            
            # Stuck vehicle detection
            SafetyRule(
                name="Vehicle Stuck",
                event_type=SafetyEvent.STUCK_VEHICLE,
                condition="vehicle_state.get('stuck_duration', 0) > 10.0",
                level=SafetyLevel.WARNING,
                response_action="unstuck_maneuver"
            ),
            
            # Water detection
            SafetyRule(
                name="Water Detection",
                event_type=SafetyEvent.WATER_DETECTION,
                condition="sensor_data.get('water_depth', 0) > config['water_resistance_depth']",
                level=SafetyLevel.CRITICAL,
                response_action="water_emergency"
            ),
            
            # Geofence monitoring
            SafetyRule(
                name="Geofence Violation",
                event_type=SafetyEvent.GEOFENCE_VIOLATION,
                condition="not (config['geofence']['min_x'] <= vehicle_state.get('x', 0) <= config['geofence']['max_x'] and config['geofence']['min_y'] <= vehicle_state.get('y', 0) <= config['geofence']['max_y'])",
                level=SafetyLevel.CRITICAL,
                response_action="return_to_safe_zone"
            ),
            
            # Sensor failure detection
            SafetyRule(
                name="Critical Sensor Failure",
                event_type=SafetyEvent.SENSOR_FAILURE,
                condition="sensor_data.get('failed_sensors', 0) > 2",
                level=SafetyLevel.CRITICAL,
                response_action="failsafe_mode"
            )
        ]
        
        self.logger.info(f"📋 {len(rules)} güvenlik kuralı yüklendi")
        return rules
    
    def start(self):
        """Start safety monitoring"""
        if not self.running:
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            self.logger.info("🚀 Safety monitoring başlatıldı")
    
    def stop(self):
        """Stop safety monitoring"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self.logger.info("🛑 Safety monitoring durduruldu")
    
    def update_sensor_data(self, sensor_data: Dict[str, Any]):
        """
        Update sensor data for safety monitoring
        
        Args:
            sensor_data: Current sensor readings
        """
        with self.lock:
            self.sensor_data.update(sensor_data)
    
    def update_vehicle_state(self, vehicle_state: Dict[str, Any]):
        """
        Update vehicle state for safety monitoring
        
        Args:
            vehicle_state: Current vehicle state
        """
        with self.lock:
            self.vehicle_state.update(vehicle_state)
    
    def register_safety_callback(self, event_type: SafetyEvent, callback: Callable):
        """
        Register callback for safety events
        
        Args:
            event_type: Type of safety event
            callback: Function to call when event occurs
        """
        if event_type not in self.safety_callbacks:
            self.safety_callbacks[event_type] = []
        self.safety_callbacks[event_type].append(callback)
        self.logger.info(f"📞 Safety callback registered for {event_type.value}")
    
    def _monitor_loop(self):
        """Main safety monitoring loop"""
        while self.running:
            start_time = time.time()
            
            try:
                # Evaluate all safety rules
                self._evaluate_safety_rules()
                
                # Update safety level
                self._update_safety_level()
                
                # Process active incidents
                self._process_active_incidents()
                
                # Check for incident resolution
                self._check_incident_resolution()
                
            except Exception as e:
                self.logger.error(f"❌ Safety monitoring hatası: {e}")
            
            # Sleep to maintain frequency
            sleep_time = 1.0 / self.config['monitor_frequency'] - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _evaluate_safety_rules(self):
        """Evaluate all active safety rules"""
        with self.lock:
            current_data = {
                'sensor_data': self.sensor_data.copy(),
                'vehicle_state': self.vehicle_state.copy(),
                'environment_data': self.environment_data.copy(),
                'config': self.config
            }
        
        for rule in self.safety_rules:
            if not rule.enabled:
                continue
            
            try:
                # Evaluate rule condition
                if self._evaluate_condition(rule.condition, current_data):
                    self._trigger_safety_event(rule, current_data)
                    
            except Exception as e:
                self.logger.error(f"❌ Rule evaluation hatası ({rule.name}): {e}")
    
    def _evaluate_condition(self, condition: str, data: Dict[str, Any]) -> bool:
        """
        Safely evaluate a condition string
        
        Args:
            condition: Python expression to evaluate
            data: Data context for evaluation
            
        Returns:
            bool: True if condition is met
        """
        try:
            # Create safe evaluation context
            safe_dict = {
                'sensor_data': data['sensor_data'],
                'vehicle_state': data['vehicle_state'],
                'environment_data': data['environment_data'],
                'config': data['config'],
                'abs': abs,
                'min': min,
                'max': max,
                'len': len,
                'time': time.time()
            }
            
            return eval(condition, {"__builtins__": {}}, safe_dict)
            
        except Exception as e:
            self.logger.debug(f"Condition evaluation error: {e}")
            return False
    
    def _trigger_safety_event(self, rule: SafetyRule, data: Dict[str, Any]):
        """
        Trigger a safety event
        
        Args:
            rule: Safety rule that triggered
            data: Current data context
        """
        current_time = time.time()
        
        # Check if this incident is already active
        active_events = [inc for inc in self.active_incidents 
                        if inc.event_type == rule.event_type and not inc.resolved]
        
        if active_events:
            # Update existing incident
            incident = active_events[0]
            incident.sensor_data = data['sensor_data'].copy()
        else:
            # Create new incident
            incident = SafetyIncident(
                timestamp=current_time,
                event_type=rule.event_type,
                level=rule.level,
                description=f"{rule.name}: {rule.condition}",
                sensor_data=data['sensor_data'].copy(),
                response_taken="",
                resolved=False
            )
            
            self.active_incidents.append(incident)
            self.incident_history.append(incident)
            self.total_incidents += 1
            
            self.logger.warning(f"⚠️ Safety event: {rule.event_type.value} - {rule.level.value}")
            
            # Execute response action
            response_start = time.time()
            self._execute_response_action(rule.response_action, incident, data)
            response_time = time.time() - response_start
            self.response_times.append(response_time)
            
            # Trigger callbacks
            self._trigger_callbacks(rule.event_type, incident)
    
    def _execute_response_action(self, action: str, incident: SafetyIncident, data: Dict[str, Any]):
        """
        Execute safety response action
        
        Args:
            action: Action to execute
            incident: Safety incident
            data: Current data context
        """
        try:
            if action == "emergency_stop":
                incident.response_taken = "Emergency stop initiated"
                self.logger.critical("🛑 EMERGENCY STOP ACTIVATED")
                
            elif action == "slow_down":
                incident.response_taken = "Speed reduction applied"
                self.logger.warning("🐌 Speed reduction for obstacle avoidance")
                
            elif action == "stabilize_vehicle":
                incident.response_taken = "Vehicle stabilization"
                self.logger.warning("⚖️ Vehicle stabilization for excessive tilt")
                
            elif action == "thermal_protection":
                incident.response_taken = "Thermal protection activated"
                self.logger.warning("🌡️ Thermal protection mode")
                
            elif action == "power_conservation":
                incident.response_taken = "Power conservation mode"
                self.logger.warning("🔋 Power conservation activated")
                
            elif action == "enforce_speed_limit":
                incident.response_taken = "Speed limit enforced"
                self.logger.warning("🚨 Speed limit enforcement")
                
            elif action == "unstuck_maneuver":
                incident.response_taken = "Unstuck maneuver"
                self.logger.warning("🔄 Unstuck maneuver initiated")
                
            elif action == "water_emergency":
                incident.response_taken = "Water emergency protocol"
                self.logger.critical("🌊 WATER EMERGENCY PROTOCOL")
                
            elif action == "return_to_safe_zone":
                incident.response_taken = "Return to safe zone"
                self.logger.critical("🚨 Returning to safe zone - geofence violation")
                
            elif action == "failsafe_mode":
                incident.response_taken = "Failsafe mode activated"
                self.logger.critical("🔒 FAILSAFE MODE - Multiple sensor failures")
                
            else:
                incident.response_taken = f"Unknown action: {action}"
                self.logger.error(f"❌ Unknown response action: {action}")
                
        except Exception as e:
            incident.response_taken = f"Action failed: {str(e)}"
            self.logger.error(f"❌ Response action hatası: {e}")
    
    def _trigger_callbacks(self, event_type: SafetyEvent, incident: SafetyIncident):
        """
        Trigger registered callbacks for safety event
        
        Args:
            event_type: Type of safety event
            incident: Safety incident data
        """
        if event_type in self.safety_callbacks:
            for callback in self.safety_callbacks[event_type]:
                try:
                    callback(incident)
                except Exception as e:
                    self.logger.error(f"❌ Safety callback hatası: {e}")
    
    def _update_safety_level(self):
        """Update overall safety level based on active incidents"""
        if not self.active_incidents:
            self.current_level = SafetyLevel.SAFE
            return
        
        # Find highest severity level
        active_levels = [inc.level for inc in self.active_incidents if not inc.resolved]
        
        if SafetyLevel.EMERGENCY in active_levels:
            self.current_level = SafetyLevel.EMERGENCY
        elif SafetyLevel.CRITICAL in active_levels:
            self.current_level = SafetyLevel.CRITICAL
        elif SafetyLevel.WARNING in active_levels:
            self.current_level = SafetyLevel.WARNING
        elif SafetyLevel.CAUTION in active_levels:
            self.current_level = SafetyLevel.CAUTION
        else:
            self.current_level = SafetyLevel.SAFE
    
    def _process_active_incidents(self):
        """Process and update active incidents"""
        current_time = time.time()
        
        for incident in self.active_incidents:
            if not incident.resolved:
                # Check if incident should auto-resolve
                age = current_time - incident.timestamp
                
                # Auto-resolve certain incidents after timeout
                if (incident.event_type in [SafetyEvent.OBSTACLE_TOO_CLOSE, SafetyEvent.SPEED_VIOLATION] 
                    and age > 30.0):
                    self._resolve_incident(incident, "Auto-resolved: timeout")
    
    def _check_incident_resolution(self):
        """Check if incidents can be resolved"""
        with self.lock:
            current_data = {
                'sensor_data': self.sensor_data.copy(),
                'vehicle_state': self.vehicle_state.copy(),
                'environment_data': self.environment_data.copy(),
                'config': self.config
            }
        
        for incident in self.active_incidents:
            if not incident.resolved:
                # Check if original condition is no longer true
                rule = next((r for r in self.safety_rules if r.event_type == incident.event_type), None)
                if rule and not self._evaluate_condition(rule.condition, current_data):
                    self._resolve_incident(incident, "Condition resolved")
    
    def _resolve_incident(self, incident: SafetyIncident, reason: str):
        """
        Resolve a safety incident
        
        Args:
            incident: Incident to resolve
            reason: Reason for resolution
        """
        incident.resolved = True
        incident.resolution_time = time.time()
        self.logger.info(f"✅ Safety incident resolved: {incident.event_type.value} - {reason}")
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get current safety status
        
        Returns:
            Dict: Safety status information
        """
        with self.lock:
            active_count = len([inc for inc in self.active_incidents if not inc.resolved])
            
            return {
                'level': self.current_level.value,
                'active_incidents': active_count,
                'total_incidents': self.total_incidents,
                'last_incident': self.incident_history[-1].event_type.value if self.incident_history else None,
                'avg_response_time': np.mean(self.response_times) if self.response_times else 0,
                'geofence_status': self._check_geofence_status()
            }
    
    def _check_geofence_status(self) -> str:
        """Check current geofence status"""
        x = self.vehicle_state.get('x', 0)
        y = self.vehicle_state.get('y', 0)
        
        gf = self.geofence_boundaries
        if gf['min_x'] <= x <= gf['max_x'] and gf['min_y'] <= y <= gf['max_y']:
            return "inside"
        else:
            return "violation"
    
    def get_active_incidents(self) -> List[SafetyIncident]:
        """
        Get list of active safety incidents
        
        Returns:
            List: Active incidents
        """
        return [inc for inc in self.active_incidents if not inc.resolved]
    
    def get_incident_history(self, limit: int = 50) -> List[SafetyIncident]:
        """
        Get incident history
        
        Args:
            limit: Maximum number of incidents to return
            
        Returns:
            List: Recent incidents
        """
        return list(self.incident_history)[-limit:]
    
    def force_emergency_stop(self, reason: str = "Manual emergency stop"):
        """
        Force emergency stop
        
        Args:
            reason: Reason for emergency stop
        """
        incident = SafetyIncident(
            timestamp=time.time(),
            event_type=SafetyEvent.COMMUNICATION_LOSS,  # Use as generic emergency
            level=SafetyLevel.EMERGENCY,
            description=reason,
            sensor_data=self.sensor_data.copy(),
            response_taken="Manual emergency stop",
            resolved=False
        )
        
        self.active_incidents.append(incident)
        self.logger.critical(f"🚨 FORCED EMERGENCY STOP: {reason}")
        
        # Trigger emergency callbacks
        self._trigger_callbacks(SafetyEvent.COMMUNICATION_LOSS, incident)

def main():
    """Test function"""
    logging.basicConfig(level=logging.INFO)
    
    # Create safety monitor
    safety = SafetyMonitorSystem()
    safety.start()
    
    print("🛡️ Safety Monitor System Test")
    print("=" * 40)
    
    try:
        # Test normal operation
        for i in range(30):
            # Simulate sensor data
            sensor_data = {
                'obstacles': [{'distance': 5.0 - i * 0.1}] if i > 20 else [],
                'imu_pitch': i * 2 if i > 25 else 0,  # Excessive tilt at end
                'temperature': 25 + i,
                'battery_voltage': 12.5 - i * 0.1,
                'water_depth': 0.1 if i > 15 else 0
            }
            
            vehicle_state = {
                'speed': 2.0 + i * 0.1,
                'x': i * 0.5,
                'y': 0,
                'stuck_duration': max(0, i - 20)
            }
            
            safety.update_sensor_data(sensor_data)
            safety.update_vehicle_state(vehicle_state)
            
            if i % 10 == 0:
                status = safety.get_safety_status()
                print(f"Safety Level: {status['level']}")
                print(f"Active Incidents: {status['active_incidents']}")
                if status['last_incident']:
                    print(f"Last Incident: {status['last_incident']}")
                print("-" * 20)
            
            time.sleep(0.1)
        
        # Test emergency stop
        print("\n🚨 Testing Emergency Stop...")
        safety.force_emergency_stop("Test emergency")
        
        # Final status
        final_status = safety.get_safety_status()
        print(f"\nFinal Safety Status:")
        print(f"Level: {final_status['level']}")
        print(f"Total Incidents: {final_status['total_incidents']}")
        print(f"Avg Response Time: {final_status['avg_response_time']:.3f}s")
        
        print("\n✅ Safety Monitor System test completed!")
        
    finally:
        safety.stop()

if __name__ == "__main__":
    main()

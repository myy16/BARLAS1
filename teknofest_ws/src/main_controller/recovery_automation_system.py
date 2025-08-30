#!/usr/bin/env python3
"""
Recovery Automation System for Teknofest Autonomous Vehicle
Automated recovery from failures and emergency situations
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

class RecoveryAction(Enum):
    """Kurtarma aksiyonları"""
    STOP_AND_ASSESS = "stop_and_assess"
    REVERSE_MANEUVER = "reverse_maneuver" 
    ALTERNATIVE_PATH = "alternative_path"
    SENSOR_RECALIBRATION = "sensor_recalibration"
    SYSTEM_RESTART = "system_restart"
    EMERGENCY_RETURN = "emergency_return"
    MANUAL_TAKEOVER = "manual_takeover"
    WAIT_AND_RETRY = "wait_and_retry"

class RecoveryStatus(Enum):
    """Kurtarma durumları"""
    IDLE = "idle"
    ANALYZING = "analyzing"
    EXECUTING = "executing"
    SUCCESS = "success"
    FAILED = "failed"
    MANUAL_REQUIRED = "manual_required"

class FailureType(Enum):
    """Hata tipleri"""
    STUCK_VEHICLE = "stuck_vehicle"
    PATH_BLOCKED = "path_blocked"
    SENSOR_FAILURE = "sensor_failure"
    COMMUNICATION_LOSS = "communication_loss"
    NAVIGATION_ERROR = "navigation_error"
    MECHANICAL_FAILURE = "mechanical_failure"
    POWER_FAILURE = "power_failure"
    WEATHER_CONDITIONS = "weather_conditions"

@dataclass
class RecoveryPlan:
    """Kurtarma planı"""
    failure_type: FailureType
    actions: List[RecoveryAction]
    max_attempts: int
    timeout: float
    success_criteria: str  # Python expression
    priority: int = 1

@dataclass
class RecoveryAttempt:
    """Kurtarma girişimi kaydı"""
    timestamp: float
    failure_type: FailureType
    action: RecoveryAction
    duration: float
    status: RecoveryStatus
    success: bool
    error_message: Optional[str] = None

class RecoveryAutomationSystem:
    """
    Automated recovery system for handling failures and emergencies
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize recovery automation system
        
        Args:
            config_path: Path to recovery configuration file
        """
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # Recovery configuration
        self.config = self._load_config(config_path)
        
        # Current recovery state
        self.current_status = RecoveryStatus.IDLE
        self.active_recovery = None
        self.recovery_history = deque(maxlen=500)
        
        # Recovery plans
        self.recovery_plans = self._initialize_recovery_plans()
        
        # System state
        self.vehicle_state = {}
        self.sensor_data = {}
        self.last_known_good_state = {}
        
        # Threading
        self.running = False
        self.recovery_thread = None
        self.lock = threading.Lock()
        
        # Recovery callbacks
        self.recovery_callbacks = {}
        
        # Performance metrics
        self.total_recoveries = 0
        self.successful_recoveries = 0
        self.recovery_times = deque(maxlen=100)
        
        # Recovery position tracking
        self.stuck_position = None
        self.stuck_start_time = None
        self.last_movement_time = time.time()
        
        self.logger.info("🔧 Recovery Automation System initialized")
    
    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load recovery configuration"""
        default_config = {
            "stuck_threshold": 0.1,      # m/s minimum speed
            "stuck_timeout": 10.0,       # seconds before considering stuck
            "recovery_timeout": 60.0,    # seconds for each recovery attempt
            "max_recovery_attempts": 3,  # attempts per failure
            "reverse_distance": 2.0,     # meters to reverse
            "alternative_search_radius": 5.0,  # meters
            "sensor_recalibration_time": 15.0,  # seconds
            "system_restart_time": 30.0,  # seconds
            "communication_retry_interval": 5.0,  # seconds
            "weather_wait_time": 300.0,  # seconds (5 minutes)
            "emergency_return_speed": 0.5,  # m/s
        }
        
        if config_path:
            try:
                with open(config_path, 'r') as f:
                    user_config = json.load(f)
                default_config.update(user_config)
            except Exception as e:
                self.logger.warning(f"⚠️ Recovery config yüklenemedi, default kullanılıyor: {e}")
        
        return default_config
    
    def _initialize_recovery_plans(self) -> Dict[FailureType, RecoveryPlan]:
        """Initialize recovery plans for different failure types"""
        plans = {
            FailureType.STUCK_VEHICLE: RecoveryPlan(
                failure_type=FailureType.STUCK_VEHICLE,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.REVERSE_MANEUVER,
                    RecoveryAction.ALTERNATIVE_PATH,
                    RecoveryAction.MANUAL_TAKEOVER
                ],
                max_attempts=3,
                timeout=120.0,
                success_criteria="vehicle_state.get('speed', 0) > 0.2",
                priority=2
            ),
            
            FailureType.PATH_BLOCKED: RecoveryPlan(
                failure_type=FailureType.PATH_BLOCKED,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.ALTERNATIVE_PATH,
                    RecoveryAction.WAIT_AND_RETRY,
                    RecoveryAction.EMERGENCY_RETURN
                ],
                max_attempts=2,
                timeout=90.0,
                success_criteria="len(sensor_data.get('obstacles', [])) == 0",
                priority=2
            ),
            
            FailureType.SENSOR_FAILURE: RecoveryPlan(
                failure_type=FailureType.SENSOR_FAILURE,
                actions=[
                    RecoveryAction.SENSOR_RECALIBRATION,
                    RecoveryAction.SYSTEM_RESTART,
                    RecoveryAction.MANUAL_TAKEOVER
                ],
                max_attempts=2,
                timeout=60.0,
                success_criteria="sensor_data.get('healthy_sensors', 0) >= 2",
                priority=1
            ),
            
            FailureType.COMMUNICATION_LOSS: RecoveryPlan(
                failure_type=FailureType.COMMUNICATION_LOSS,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.WAIT_AND_RETRY,
                    RecoveryAction.EMERGENCY_RETURN,
                    RecoveryAction.MANUAL_TAKEOVER
                ],
                max_attempts=3,
                timeout=180.0,
                success_criteria="sensor_data.get('communication_ok', False)",
                priority=1
            ),
            
            FailureType.NAVIGATION_ERROR: RecoveryPlan(
                failure_type=FailureType.NAVIGATION_ERROR,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.SENSOR_RECALIBRATION,
                    RecoveryAction.ALTERNATIVE_PATH,
                    RecoveryAction.EMERGENCY_RETURN
                ],
                max_attempts=2,
                timeout=90.0,
                success_criteria="vehicle_state.get('navigation_error', True) == False",
                priority=2
            ),
            
            FailureType.POWER_FAILURE: RecoveryPlan(
                failure_type=FailureType.POWER_FAILURE,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.EMERGENCY_RETURN,
                    RecoveryAction.MANUAL_TAKEOVER
                ],
                max_attempts=1,
                timeout=300.0,
                success_criteria="sensor_data.get('battery_voltage', 0) > 11.0",
                priority=1
            ),
            
            FailureType.WEATHER_CONDITIONS: RecoveryPlan(
                failure_type=FailureType.WEATHER_CONDITIONS,
                actions=[
                    RecoveryAction.STOP_AND_ASSESS,
                    RecoveryAction.WAIT_AND_RETRY,
                    RecoveryAction.EMERGENCY_RETURN
                ],
                max_attempts=2,
                timeout=600.0,
                success_criteria="sensor_data.get('weather_safe', False)",
                priority=3
            )
        }
        
        self.logger.info(f"📋 {len(plans)} kurtarma planı yüklendi")
        return plans
    
    def start(self):
        """Start recovery automation"""
        if not self.running:
            self.running = True
            self.recovery_thread = threading.Thread(target=self._recovery_loop, daemon=True)
            self.recovery_thread.start()
            self.logger.info("🚀 Recovery automation başlatıldı")
    
    def stop(self):
        """Stop recovery automation"""
        self.running = False
        if self.recovery_thread:
            self.recovery_thread.join(timeout=2.0)
        self.logger.info("🛑 Recovery automation durduruldu")
    
    def update_vehicle_state(self, vehicle_state: Dict[str, Any]):
        """
        Update vehicle state for recovery monitoring
        
        Args:
            vehicle_state: Current vehicle state
        """
        with self.lock:
            self.vehicle_state.update(vehicle_state)
            
            # Update last known good state if vehicle is moving normally
            speed = vehicle_state.get('speed', 0)
            if speed > 0.2:  # Vehicle is moving
                self.last_known_good_state.update(vehicle_state)
                self.last_movement_time = time.time()
    
    def update_sensor_data(self, sensor_data: Dict[str, Any]):
        """
        Update sensor data for recovery monitoring
        
        Args:
            sensor_data: Current sensor readings
        """
        with self.lock:
            self.sensor_data.update(sensor_data)
    
    def trigger_recovery(self, failure_type: FailureType, context: Dict[str, Any] = None):
        """
        Manually trigger recovery for specific failure type
        
        Args:
            failure_type: Type of failure to recover from
            context: Additional context for recovery
        """
        if self.current_status == RecoveryStatus.IDLE:
            self.logger.warning(f"🔧 Manual recovery triggered: {failure_type.value}")
            self._initiate_recovery(failure_type, context or {})
        else:
            self.logger.warning(f"⚠️ Recovery already in progress, ignoring trigger for {failure_type.value}")
    
    def register_recovery_callback(self, action: RecoveryAction, callback: Callable):
        """
        Register callback for recovery actions
        
        Args:
            action: Recovery action type
            callback: Function to call when action is executed
        """
        if action not in self.recovery_callbacks:
            self.recovery_callbacks[action] = []
        self.recovery_callbacks[action].append(callback)
        self.logger.info(f"📞 Recovery callback registered for {action.value}")
    
    def _recovery_loop(self):
        """Main recovery monitoring and execution loop"""
        while self.running:
            try:
                # Check for failure conditions
                self._detect_failures()
                
                # Execute active recovery if any
                if self.current_status in [RecoveryStatus.ANALYZING, RecoveryStatus.EXECUTING]:
                    self._process_active_recovery()
                
            except Exception as e:
                self.logger.error(f"❌ Recovery loop hatası: {e}")
            
            time.sleep(1.0)
    
    def _detect_failures(self):
        """Detect failure conditions that require recovery"""
        if self.current_status != RecoveryStatus.IDLE:
            return
        
        current_time = time.time()
        
        with self.lock:
            vehicle_data = self.vehicle_state.copy()
            sensor_data = self.sensor_data.copy()
        
        # Check for stuck vehicle
        speed = vehicle_data.get('speed', 0)
        if speed < self.config['stuck_threshold']:
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time
                self.stuck_position = (vehicle_data.get('x', 0), vehicle_data.get('y', 0))
            elif current_time - self.stuck_start_time > self.config['stuck_timeout']:
                self._initiate_recovery(FailureType.STUCK_VEHICLE, {'stuck_duration': current_time - self.stuck_start_time})
        else:
            self.stuck_start_time = None
            self.stuck_position = None
        
        # Check for path blocked
        obstacles = sensor_data.get('obstacles', [])
        if obstacles:
            close_obstacles = [obs for obs in obstacles if obs.get('distance', 999) < 1.0]
            if close_obstacles and speed < 0.1:
                self._initiate_recovery(FailureType.PATH_BLOCKED, {'obstacle_count': len(close_obstacles)})
        
        # Check for sensor failures
        failed_sensors = sensor_data.get('failed_sensors', 0)
        if failed_sensors > 1:
            self._initiate_recovery(FailureType.SENSOR_FAILURE, {'failed_count': failed_sensors})
        
        # Check for communication loss
        if not sensor_data.get('communication_ok', True):
            last_comm = sensor_data.get('last_communication', 0)
            if current_time - last_comm > 10.0:
                self._initiate_recovery(FailureType.COMMUNICATION_LOSS, {'comm_down_duration': current_time - last_comm})
        
        # Check for navigation errors
        if vehicle_data.get('navigation_error', False):
            self._initiate_recovery(FailureType.NAVIGATION_ERROR, {'error_type': vehicle_data.get('error_type', 'unknown')})
        
        # Check for power issues
        battery_voltage = sensor_data.get('battery_voltage', 12.0)
        if battery_voltage < 10.5:
            self._initiate_recovery(FailureType.POWER_FAILURE, {'battery_voltage': battery_voltage})
    
    def _initiate_recovery(self, failure_type: FailureType, context: Dict[str, Any]):
        """
        Initiate recovery process for failure type
        
        Args:
            failure_type: Type of failure
            context: Additional context information
        """
        if failure_type not in self.recovery_plans:
            self.logger.error(f"❌ No recovery plan for failure type: {failure_type.value}")
            return
        
        plan = self.recovery_plans[failure_type]
        
        self.active_recovery = {
            'plan': plan,
            'context': context,
            'current_action_index': 0,
            'attempts': 0,
            'start_time': time.time(),
            'action_start_time': None
        }
        
        self.current_status = RecoveryStatus.ANALYZING
        self.total_recoveries += 1
        
        self.logger.info(f"🔧 Recovery initiated: {failure_type.value}")
        self.logger.info(f"📋 Recovery plan: {len(plan.actions)} actions, max {plan.max_attempts} attempts")
    
    def _process_active_recovery(self):
        """Process currently active recovery"""
        if not self.active_recovery:
            return
        
        plan = self.active_recovery['plan']
        current_time = time.time()
        
        # Check overall recovery timeout
        if current_time - self.active_recovery['start_time'] > plan.timeout:
            self._fail_recovery("Overall timeout exceeded")
            return
        
        # Check if recovery is successful
        if self._check_success_criteria(plan.success_criteria):
            self._complete_recovery("Success criteria met")
            return
        
        # Execute current action
        if self.current_status == RecoveryStatus.ANALYZING:
            self._start_next_action()
        elif self.current_status == RecoveryStatus.EXECUTING:
            self._continue_action_execution()
    
    def _start_next_action(self):
        """Start next recovery action"""
        plan = self.active_recovery['plan']
        action_index = self.active_recovery['current_action_index']
        
        if action_index >= len(plan.actions):
            # All actions exhausted
            if self.active_recovery['attempts'] < plan.max_attempts:
                # Retry from beginning
                self.active_recovery['current_action_index'] = 0
                self.active_recovery['attempts'] += 1
                action_index = 0
                self.logger.info(f"🔄 Recovery attempt {self.active_recovery['attempts']}/{plan.max_attempts}")
            else:
                self._fail_recovery("All actions and attempts exhausted")
                return
        
        action = plan.actions[action_index]
        self.active_recovery['action_start_time'] = time.time()
        self.current_status = RecoveryStatus.EXECUTING
        
        self.logger.info(f"⚡ Executing recovery action: {action.value}")
        
        # Execute the action
        self._execute_recovery_action(action)
    
    def _execute_recovery_action(self, action: RecoveryAction):
        """
        Execute specific recovery action
        
        Args:
            action: Recovery action to execute
        """
        start_time = time.time()
        
        try:
            if action == RecoveryAction.STOP_AND_ASSESS:
                self._action_stop_and_assess()
            elif action == RecoveryAction.REVERSE_MANEUVER:
                self._action_reverse_maneuver()
            elif action == RecoveryAction.ALTERNATIVE_PATH:
                self._action_alternative_path()
            elif action == RecoveryAction.SENSOR_RECALIBRATION:
                self._action_sensor_recalibration()
            elif action == RecoveryAction.SYSTEM_RESTART:
                self._action_system_restart()
            elif action == RecoveryAction.EMERGENCY_RETURN:
                self._action_emergency_return()
            elif action == RecoveryAction.MANUAL_TAKEOVER:
                self._action_manual_takeover()
            elif action == RecoveryAction.WAIT_AND_RETRY:
                self._action_wait_and_retry()
            else:
                self.logger.error(f"❌ Unknown recovery action: {action.value}")
            
            # Record attempt
            attempt = RecoveryAttempt(
                timestamp=start_time,
                failure_type=self.active_recovery['plan'].failure_type,
                action=action,
                duration=time.time() - start_time,
                status=RecoveryStatus.SUCCESS,
                success=True
            )
            self.recovery_history.append(attempt)
            
        except Exception as e:
            # Record failed attempt
            attempt = RecoveryAttempt(
                timestamp=start_time,
                failure_type=self.active_recovery['plan'].failure_type,
                action=action,
                duration=time.time() - start_time,
                status=RecoveryStatus.FAILED,
                success=False,
                error_message=str(e)
            )
            self.recovery_history.append(attempt)
            
            self.logger.error(f"❌ Recovery action failed: {action.value} - {e}")
        
        # Trigger callbacks
        self._trigger_recovery_callbacks(action)
    
    def _action_stop_and_assess(self):
        """Stop vehicle and assess situation"""
        self.logger.info("🛑 Stopping and assessing situation...")
        time.sleep(2.0)  # Allow systems to settle
        # Vehicle should be stopped by safety system
    
    def _action_reverse_maneuver(self):
        """Execute reverse maneuver to get unstuck"""
        self.logger.info("⬅️ Executing reverse maneuver...")
        distance = self.config['reverse_distance']
        # Implementation would send reverse commands to vehicle
        time.sleep(3.0)  # Simulate reverse time
        self.logger.info(f"⬅️ Reversed {distance}m")
    
    def _action_alternative_path(self):
        """Find and execute alternative path"""
        self.logger.info("🗺️ Searching for alternative path...")
        search_radius = self.config['alternative_search_radius']
        time.sleep(5.0)  # Simulate path planning
        self.logger.info(f"🗺️ Alternative path found within {search_radius}m radius")
    
    def _action_sensor_recalibration(self):
        """Recalibrate sensors"""
        self.logger.info("🔧 Recalibrating sensors...")
        recal_time = self.config['sensor_recalibration_time']
        time.sleep(recal_time)
        self.logger.info("🔧 Sensor recalibration completed")
    
    def _action_system_restart(self):
        """Restart critical systems"""
        self.logger.info("🔄 Restarting systems...")
        restart_time = self.config['system_restart_time']
        time.sleep(restart_time)
        self.logger.info("🔄 System restart completed")
    
    def _action_emergency_return(self):
        """Execute emergency return to safe location"""
        self.logger.info("🏠 Initiating emergency return...")
        if self.last_known_good_state:
            target = (
                self.last_known_good_state.get('x', 0),
                self.last_known_good_state.get('y', 0)
            )
            self.logger.info(f"🏠 Returning to last known good position: {target}")
        else:
            self.logger.info("🏠 Returning to start position")
        time.sleep(10.0)  # Simulate return journey
    
    def _action_manual_takeover(self):
        """Request manual takeover"""
        self.logger.critical("👤 MANUAL TAKEOVER REQUIRED")
        self.current_status = RecoveryStatus.MANUAL_REQUIRED
        # This would trigger manual control system
    
    def _action_wait_and_retry(self):
        """Wait and retry operation"""
        wait_time = self.config.get('weather_wait_time', 30.0)
        self.logger.info(f"⏳ Waiting {wait_time}s before retry...")
        time.sleep(min(wait_time, 10.0))  # Cap at 10s for demo
        self.logger.info("⏳ Wait period completed")
    
    def _continue_action_execution(self):
        """Continue monitoring action execution"""
        # Check if current action has timed out or completed
        current_time = time.time()
        action_duration = current_time - self.active_recovery['action_start_time']
        
        # Most actions complete quickly, but some may need time
        if action_duration > 30.0:  # 30 second timeout per action
            self.logger.warning("⏰ Recovery action timeout")
            self._advance_to_next_action()
        else:
            # Check if action is complete based on success criteria
            plan = self.active_recovery['plan']
            if self._check_success_criteria(plan.success_criteria):
                self._complete_recovery("Action completed successfully")
            else:
                # Action still in progress, give it more time
                if action_duration > 15.0:  # After 15 seconds, move to next action
                    self._advance_to_next_action()
    
    def _advance_to_next_action(self):
        """Advance to next recovery action"""
        self.active_recovery['current_action_index'] += 1
        self.current_status = RecoveryStatus.ANALYZING
    
    def _check_success_criteria(self, criteria: str) -> bool:
        """
        Check if success criteria are met
        
        Args:
            criteria: Success criteria expression
            
        Returns:
            bool: True if criteria are met
        """
        try:
            # Create evaluation context
            context = {
                'vehicle_state': self.vehicle_state,
                'sensor_data': self.sensor_data,
                'time': time.time(),
                'len': len
            }
            
            return eval(criteria, {"__builtins__": {}}, context)
            
        except Exception as e:
            self.logger.debug(f"Success criteria evaluation error: {e}")
            return False
    
    def _complete_recovery(self, reason: str):
        """
        Complete recovery process successfully
        
        Args:
            reason: Reason for completion
        """
        duration = time.time() - self.active_recovery['start_time']
        self.recovery_times.append(duration)
        self.successful_recoveries += 1
        
        failure_type = self.active_recovery['plan'].failure_type
        
        self.logger.info(f"✅ Recovery completed: {failure_type.value} - {reason}")
        self.logger.info(f"⏱️ Recovery duration: {duration:.1f}s")
        
        self._reset_recovery_state()
    
    def _fail_recovery(self, reason: str):
        """
        Fail recovery process
        
        Args:
            reason: Reason for failure
        """
        duration = time.time() - self.active_recovery['start_time']
        failure_type = self.active_recovery['plan'].failure_type
        
        self.logger.error(f"❌ Recovery failed: {failure_type.value} - {reason}")
        self.logger.error(f"⏱️ Recovery duration: {duration:.1f}s")
        
        # Escalate to manual takeover
        self.current_status = RecoveryStatus.MANUAL_REQUIRED
        self._action_manual_takeover()
        
        self._reset_recovery_state()
    
    def _reset_recovery_state(self):
        """Reset recovery state to idle"""
        self.active_recovery = None
        self.current_status = RecoveryStatus.IDLE
        self.stuck_start_time = None
        self.stuck_position = None
    
    def _trigger_recovery_callbacks(self, action: RecoveryAction):
        """
        Trigger registered callbacks for recovery action
        
        Args:
            action: Recovery action that was executed
        """
        if action in self.recovery_callbacks:
            for callback in self.recovery_callbacks[action]:
                try:
                    callback(self.active_recovery)
                except Exception as e:
                    self.logger.error(f"❌ Recovery callback hatası: {e}")
    
    def get_recovery_status(self) -> Dict[str, Any]:
        """
        Get current recovery status
        
        Returns:
            Dict: Recovery status information
        """
        return {
            'status': self.current_status.value,
            'total_recoveries': self.total_recoveries,
            'successful_recoveries': self.successful_recoveries,
            'success_rate': (self.successful_recoveries / max(1, self.total_recoveries)) * 100,
            'avg_recovery_time': np.mean(self.recovery_times) if self.recovery_times else 0,
            'active_failure_type': self.active_recovery['plan'].failure_type.value if self.active_recovery else None,
            'stuck_duration': time.time() - self.stuck_start_time if self.stuck_start_time else 0
        }
    
    def get_recovery_history(self, limit: int = 50) -> List[RecoveryAttempt]:
        """
        Get recovery attempt history
        
        Args:
            limit: Maximum number of attempts to return
            
        Returns:
            List: Recent recovery attempts
        """
        return list(self.recovery_history)[-limit:]
    
    def force_recovery_abort(self):
        """Force abort current recovery"""
        if self.current_status != RecoveryStatus.IDLE:
            self.logger.warning("🛑 Recovery forcibly aborted")
            self._reset_recovery_state()

def main():
    """Test function"""
    logging.basicConfig(level=logging.INFO)
    
    # Create recovery system
    recovery = RecoveryAutomationSystem()
    recovery.start()
    
    print("🔧 Recovery Automation System Test")
    print("=" * 40)
    
    try:
        # Test stuck vehicle scenario
        print("🚗 Simulating stuck vehicle...")
        for i in range(15):
            vehicle_state = {
                'speed': 0.05,  # Very low speed
                'x': 10.0,
                'y': 5.0,
                'navigation_error': False
            }
            
            sensor_data = {
                'obstacles': [{'distance': 0.8}] if i > 5 else [],
                'healthy_sensors': 3,
                'battery_voltage': 12.0,
                'communication_ok': True
            }
            
            recovery.update_vehicle_state(vehicle_state)
            recovery.update_sensor_data(sensor_data)
            
            time.sleep(1)
            
            if i % 5 == 0:
                status = recovery.get_recovery_status()
                print(f"Status: {status['status']}, Stuck Duration: {status['stuck_duration']:.1f}s")
        
        # Wait for recovery to complete
        time.sleep(5)
        
        # Test sensor failure
        print("\n📡 Simulating sensor failure...")
        recovery.trigger_recovery(FailureType.SENSOR_FAILURE, {'failed_sensors': 3})
        
        time.sleep(3)
        
        # Final status
        final_status = recovery.get_recovery_status()
        print(f"\nFinal Recovery Status:")
        print(f"Total Recoveries: {final_status['total_recoveries']}")
        print(f"Success Rate: {final_status['success_rate']:.1f}%")
        print(f"Avg Recovery Time: {final_status['avg_recovery_time']:.1f}s")
        
        print("\n✅ Recovery Automation System test completed!")
        
    finally:
        recovery.stop()

if __name__ == "__main__":
    main()

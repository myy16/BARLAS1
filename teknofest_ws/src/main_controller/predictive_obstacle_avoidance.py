#!/usr/bin/env python3
"""
🤖 PREDICTIVE OBSTACLE AVOIDANCE SYSTEM
Advanced trajectory prediction, dynamic path planning, and collision avoidance
"""

import json
import time
import numpy as np
import math
from threading import Thread, Lock
from datetime import datetime, timedelta
from collections import deque
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import logging

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

@dataclass
class Point2D:
    """2D Point representation"""
    x: float
    y: float
    
    def distance_to(self, other: 'Point2D') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Point2D') -> float:
        return math.atan2(other.y - self.y, other.x - self.x)

@dataclass
class Obstacle:
    """Dynamic obstacle representation"""
    id: str
    position: Point2D
    velocity: Point2D
    size: float
    confidence: float
    prediction_horizon: float = 3.0
    last_seen: float = 0.0
    
    def predict_position(self, time_delta: float) -> Point2D:
        """Predict obstacle position after time_delta seconds"""
        return Point2D(
            self.position.x + self.velocity.x * time_delta,
            self.position.y + self.velocity.y * time_delta
        )
    
    def get_predicted_path(self, steps: int = 10) -> List[Point2D]:
        """Get predicted path over time horizon"""
        path = []
        time_step = self.prediction_horizon / steps
        for i in range(steps + 1):
            path.append(self.predict_position(i * time_step))
        return path

@dataclass
class VehicleState:
    """Current vehicle state"""
    position: Point2D
    velocity: Point2D
    heading: float
    angular_velocity: float
    acceleration: Point2D
    
    def predict_position(self, time_delta: float) -> Point2D:
        """Predict vehicle position after time_delta seconds"""
        # Simple kinematic model
        x = self.position.x + self.velocity.x * time_delta + 0.5 * self.acceleration.x * time_delta**2
        y = self.position.y + self.velocity.y * time_delta + 0.5 * self.acceleration.y * time_delta**2
        return Point2D(x, y)

@dataclass
class Waypoint:
    """Waypoint for path planning"""
    position: Point2D
    desired_speed: float
    priority: int = 1
    tolerance: float = 0.5

@dataclass
class AvoidanceManeuver:
    """Avoidance maneuver definition"""
    maneuver_type: str  # 'stop', 'slow_down', 'change_lane', 'reverse'
    target_position: Optional[Point2D] = None
    target_speed: float = 0.0
    duration: float = 1.0
    urgency: int = 1  # 1=low, 5=emergency
    
class TrajectoryPredictor:
    """Predicts trajectories for dynamic obstacles"""
    
    def __init__(self):
        self.obstacle_history = {}
        self.prediction_models = {
            'linear': self._predict_linear,
            'curved': self._predict_curved,
            'stopping': self._predict_stopping
        }
    
    def update_obstacle(self, obstacle: Obstacle):
        """Update obstacle in tracking history"""
        if obstacle.id not in self.obstacle_history:
            self.obstacle_history[obstacle.id] = deque(maxlen=10)
        
        self.obstacle_history[obstacle.id].append({
            'timestamp': time.time(),
            'position': obstacle.position,
            'velocity': obstacle.velocity
        })
    
    def _predict_linear(self, obstacle: Obstacle) -> List[Point2D]:
        """Linear motion prediction"""
        return obstacle.get_predicted_path(20)
    
    def _predict_curved(self, obstacle: Obstacle) -> List[Point2D]:
        """Curved motion prediction (for turning obstacles)"""
        if obstacle.id not in self.obstacle_history or len(self.obstacle_history[obstacle.id]) < 3:
            return self._predict_linear(obstacle)
        
        # Calculate curvature from recent positions
        history = list(self.obstacle_history[obstacle.id])[-3:]
        if len(history) < 3:
            return self._predict_linear(obstacle)
        
        # Simple curvature estimation
        p1, p2, p3 = [h['position'] for h in history]
        
        # Calculate turning angle
        angle1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        angle2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
        angular_velocity = (angle2 - angle1) / 0.1  # Assume 0.1s timestep
        
        # Predict with angular velocity
        path = []
        current_pos = obstacle.position
        current_angle = angle2
        speed = math.sqrt(obstacle.velocity.x**2 + obstacle.velocity.y**2)
        
        for i in range(21):
            time_delta = i * obstacle.prediction_horizon / 20
            new_angle = current_angle + angular_velocity * time_delta
            new_x = current_pos.x + speed * math.cos(new_angle) * time_delta
            new_y = current_pos.y + speed * math.sin(new_angle) * time_delta
            path.append(Point2D(new_x, new_y))
        
        return path
    
    def _predict_stopping(self, obstacle: Obstacle) -> List[Point2D]:
        """Predict obstacle that is decelerating to stop"""
        deceleration = 2.0  # m/s²
        speed = math.sqrt(obstacle.velocity.x**2 + obstacle.velocity.y**2)
        
        if speed < 0.1:
            # Already stopped
            return [obstacle.position] * 21
        
        stop_time = speed / deceleration
        stop_distance = speed**2 / (2 * deceleration)
        
        # Calculate direction
        direction_x = obstacle.velocity.x / speed if speed > 0 else 0
        direction_y = obstacle.velocity.y / speed if speed > 0 else 0
        
        path = []
        for i in range(21):
            time_delta = i * obstacle.prediction_horizon / 20
            
            if time_delta <= stop_time:
                # Still moving
                distance = speed * time_delta - 0.5 * deceleration * time_delta**2
                new_x = obstacle.position.x + direction_x * distance
                new_y = obstacle.position.y + direction_y * distance
            else:
                # Stopped
                new_x = obstacle.position.x + direction_x * stop_distance
                new_y = obstacle.position.y + direction_y * stop_distance
            
            path.append(Point2D(new_x, new_y))
        
        return path
    
    def predict_trajectory(self, obstacle: Obstacle, model_type: str = 'linear') -> List[Point2D]:
        """Predict obstacle trajectory using specified model"""
        if model_type in self.prediction_models:
            return self.prediction_models[model_type](obstacle)
        else:
            return self._predict_linear(obstacle)

class PathPlanner:
    """Dynamic path planning with obstacle avoidance"""
    
    def __init__(self, vehicle_width: float = 1.0, safety_margin: float = 0.5):
        self.vehicle_width = vehicle_width
        self.safety_margin = safety_margin
        self.planning_horizon = 5.0  # seconds
        self.path_resolution = 0.2  # meters
        
    def plan_path(self, start: Point2D, goal: Point2D, obstacles: List[Obstacle], 
                  vehicle_state: VehicleState) -> Tuple[List[Point2D], bool]:
        """Plan collision-free path from start to goal"""
        
        # Simple A* like approach for demonstration
        # In practice, would use RRT*, A*, or similar algorithms
        
        # Check if direct path is clear
        direct_path = self._generate_direct_path(start, goal)
        if self._is_path_safe(direct_path, obstacles, vehicle_state):
            return direct_path, True
        
        # Try alternative paths
        alternative_paths = self._generate_alternative_paths(start, goal, obstacles)
        
        for path in alternative_paths:
            if self._is_path_safe(path, obstacles, vehicle_state):
                return path, True
        
        # No safe path found
        return [], False
    
    def _generate_direct_path(self, start: Point2D, goal: Point2D) -> List[Point2D]:
        """Generate direct path between start and goal"""
        distance = start.distance_to(goal)
        num_points = int(distance / self.path_resolution) + 1
        
        path = []
        for i in range(num_points + 1):
            t = i / num_points if num_points > 0 else 0
            x = start.x + t * (goal.x - start.x)
            y = start.y + t * (goal.y - start.y)
            path.append(Point2D(x, y))
        
        return path
    
    def _generate_alternative_paths(self, start: Point2D, goal: Point2D, 
                                  obstacles: List[Obstacle]) -> List[List[Point2D]]:
        """Generate alternative paths avoiding obstacles"""
        alternative_paths = []
        
        # Generate paths with lateral offsets
        for offset in [-2.0, -1.0, 1.0, 2.0]:
            # Calculate perpendicular direction
            direction_x = goal.x - start.x
            direction_y = goal.y - start.y
            length = math.sqrt(direction_x**2 + direction_y**2)
            
            if length > 0:
                norm_x = direction_x / length
                norm_y = direction_y / length
                
                # Perpendicular vector
                perp_x = -norm_y
                perp_y = norm_x
                
                # Waypoint with offset
                mid_x = (start.x + goal.x) / 2 + perp_x * offset
                mid_y = (start.y + goal.y) / 2 + perp_y * offset
                waypoint = Point2D(mid_x, mid_y)
                
                # Create path through waypoint
                path1 = self._generate_direct_path(start, waypoint)
                path2 = self._generate_direct_path(waypoint, goal)
                
                # Combine paths (remove duplicate waypoint)
                full_path = path1 + path2[1:]
                alternative_paths.append(full_path)
        
        return alternative_paths
    
    def _is_path_safe(self, path: List[Point2D], obstacles: List[Obstacle], 
                     vehicle_state: VehicleState) -> bool:
        """Check if path is collision-free"""
        if not path:
            return False
        
        # Check each point in path against obstacle predictions
        for i, point in enumerate(path):
            time_at_point = i * 0.1  # Assume 0.1s between path points
            
            for obstacle in obstacles:
                predicted_pos = obstacle.predict_position(time_at_point)
                
                # Check collision with safety margin
                safety_distance = (self.vehicle_width + obstacle.size) / 2 + self.safety_margin
                distance = point.distance_to(predicted_pos)
                
                if distance < safety_distance:
                    return False
        
        return True

class CollisionDetector:
    """Detects potential collisions and calculates risk"""
    
    def __init__(self, time_horizon: float = 3.0):
        self.time_horizon = time_horizon
        self.risk_levels = {
            'safe': 0.0,
            'low': 0.3,
            'medium': 0.6,
            'high': 0.8,
            'critical': 1.0
        }
    
    def calculate_collision_risk(self, vehicle_state: VehicleState, 
                               obstacles: List[Obstacle]) -> Tuple[float, List[Dict]]:
        """Calculate overall collision risk and identify threats"""
        max_risk = 0.0
        threats = []
        
        for obstacle in obstacles:
            risk, time_to_collision = self._calculate_obstacle_risk(vehicle_state, obstacle)
            
            if risk > 0.1:  # Only consider significant risks
                threats.append({
                    'obstacle_id': obstacle.id,
                    'risk_level': self._risk_to_level(risk),
                    'risk_score': risk,
                    'time_to_collision': time_to_collision,
                    'position': obstacle.position,
                    'velocity': obstacle.velocity
                })
            
            max_risk = max(max_risk, risk)
        
        return max_risk, threats
    
    def _calculate_obstacle_risk(self, vehicle_state: VehicleState, 
                               obstacle: Obstacle) -> Tuple[float, float]:
        """Calculate collision risk with specific obstacle"""
        # Simplified collision risk calculation
        # In practice, would use more sophisticated models
        
        # Calculate closest point of approach
        rel_pos_x = obstacle.position.x - vehicle_state.position.x
        rel_pos_y = obstacle.position.y - vehicle_state.position.y
        rel_vel_x = obstacle.velocity.x - vehicle_state.velocity.x
        rel_vel_y = obstacle.velocity.y - vehicle_state.velocity.y
        
        # Time to closest approach
        rel_speed_sq = rel_vel_x**2 + rel_vel_y**2
        if rel_speed_sq < 0.01:  # Objects moving at similar speed
            distance = math.sqrt(rel_pos_x**2 + rel_pos_y**2)
            if distance < 2.0:  # Close and similar speed = medium risk
                return 0.5, float('inf')
            else:
                return 0.1, float('inf')
        
        time_to_closest = -(rel_pos_x * rel_vel_x + rel_pos_y * rel_vel_y) / rel_speed_sq
        
        if time_to_closest < 0 or time_to_closest > self.time_horizon:
            return 0.0, time_to_closest
        
        # Position at closest approach
        closest_x = rel_pos_x + rel_vel_x * time_to_closest
        closest_y = rel_pos_y + rel_vel_y * time_to_closest
        closest_distance = math.sqrt(closest_x**2 + closest_y**2)
        
        # Risk based on distance and time
        collision_threshold = 1.5  # meters
        if closest_distance > collision_threshold:
            return 0.0, time_to_closest
        
        # Calculate risk score
        distance_factor = max(0, 1 - closest_distance / collision_threshold)
        time_factor = max(0, 1 - time_to_closest / self.time_horizon)
        confidence_factor = obstacle.confidence
        
        risk = distance_factor * time_factor * confidence_factor
        
        return min(risk, 1.0), time_to_closest
    
    def _risk_to_level(self, risk: float) -> str:
        """Convert risk score to level"""
        if risk >= self.risk_levels['critical']:
            return 'critical'
        elif risk >= self.risk_levels['high']:
            return 'high'
        elif risk >= self.risk_levels['medium']:
            return 'medium'
        elif risk >= self.risk_levels['low']:
            return 'low'
        else:
            return 'safe'

class PredictiveObstacleAvoidanceSystem:
    """Main predictive obstacle avoidance system"""
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # System state
        self.active = False
        self.avoidance_thread = None
        self.lock = Lock()
        
        # Components
        self.trajectory_predictor = TrajectoryPredictor()
        self.path_planner = PathPlanner()
        self.collision_detector = CollisionDetector()
        
        # Data
        self.obstacles = {}
        self.current_vehicle_state = VehicleState(
            Point2D(0, 0), Point2D(0, 0), 0, 0, Point2D(0, 0)
        )
        self.target_waypoints = deque()
        self.planned_path = []
        self.current_maneuver = None
        
        # Performance metrics
        self.avoidance_count = 0
        self.collision_warnings = 0
        self.path_replanning_count = 0
        self.start_time = time.time()
        
        # Callbacks
        self.collision_warning_callback = None
        self.path_update_callback = None
        self.emergency_stop_callback = None
        
        self.logger.info("Predictive Obstacle Avoidance System initialized")
    
    def start_avoidance(self):
        """Start predictive obstacle avoidance system"""
        if self.active:
            return
        
        self.active = True
        self.avoidance_thread = Thread(target=self._avoidance_loop, daemon=True)
        self.avoidance_thread.start()
        self.logger.info("Predictive obstacle avoidance started")
    
    def stop_avoidance(self):
        """Stop predictive obstacle avoidance system"""
        self.active = False
        if self.avoidance_thread:
            self.avoidance_thread.join(timeout=1.0)
        self.logger.info("Predictive obstacle avoidance stopped")
    
    def update_vehicle_state(self, position: Tuple[float, float], 
                           velocity: Tuple[float, float], 
                           heading: float, angular_velocity: float = 0.0,
                           acceleration: Tuple[float, float] = (0.0, 0.0)):
        """Update current vehicle state"""
        with self.lock:
            self.current_vehicle_state = VehicleState(
                Point2D(position[0], position[1]),
                Point2D(velocity[0], velocity[1]),
                heading,
                angular_velocity,
                Point2D(acceleration[0], acceleration[1])
            )
    
    def update_obstacles(self, obstacle_data: List[Dict]):
        """Update obstacle information"""
        with self.lock:
            current_time = time.time()
            
            # Update existing obstacles and add new ones
            active_obstacle_ids = set()
            
            for obs_data in obstacle_data:
                obstacle_id = obs_data.get('id', f"obs_{len(self.obstacles)}")
                active_obstacle_ids.add(obstacle_id)
                
                position = Point2D(obs_data['position']['x'], obs_data['position']['y'])
                velocity = Point2D(
                    obs_data.get('velocity', {}).get('x', 0.0),
                    obs_data.get('velocity', {}).get('y', 0.0)
                )
                
                obstacle = Obstacle(
                    id=obstacle_id,
                    position=position,
                    velocity=velocity,
                    size=obs_data.get('size', 1.0),
                    confidence=obs_data.get('confidence', 0.8),
                    last_seen=current_time
                )
                
                self.obstacles[obstacle_id] = obstacle
                self.trajectory_predictor.update_obstacle(obstacle)
            
            # Remove outdated obstacles (not seen for >2 seconds)
            obstacles_to_remove = []
            for obs_id, obstacle in self.obstacles.items():
                if obs_id not in active_obstacle_ids and current_time - obstacle.last_seen > 2.0:
                    obstacles_to_remove.append(obs_id)
            
            for obs_id in obstacles_to_remove:
                del self.obstacles[obs_id]
    
    def set_target_waypoints(self, waypoints: List[Dict]):
        """Set target waypoints for navigation"""
        with self.lock:
            self.target_waypoints.clear()
            for wp_data in waypoints:
                waypoint = Waypoint(
                    position=Point2D(wp_data['position']['x'], wp_data['position']['y']),
                    desired_speed=wp_data.get('speed', 1.0),
                    priority=wp_data.get('priority', 1),
                    tolerance=wp_data.get('tolerance', 0.5)
                )
                self.target_waypoints.append(waypoint)
    
    def _avoidance_loop(self):
        """Main avoidance loop"""
        while self.active:
            try:
                with self.lock:
                    # Calculate collision risks
                    risk_score, threats = self.collision_detector.calculate_collision_risk(
                        self.current_vehicle_state, list(self.obstacles.values())
                    )
                    
                    # Handle high-risk situations
                    if risk_score > 0.8:
                        self._handle_critical_situation(threats)
                    elif risk_score > 0.5:
                        self._handle_high_risk_situation(threats)
                    
                    # Update path planning if needed
                    if self.target_waypoints and (not self.planned_path or risk_score > 0.3):
                        self._update_path_plan()
                    
                    # Log collision warnings
                    if threats:
                        critical_threats = [t for t in threats if t['risk_level'] in ['high', 'critical']]
                        if critical_threats:
                            self.collision_warnings += 1
                            threat_summary = ", ".join([f"{t['obstacle_id']}({t['risk_level']})" 
                                                      for t in critical_threats])
                            self.logger.warning(f"Collision threats detected: {threat_summary}")
                
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                self.logger.error(f"Avoidance loop error: {e}")
                time.sleep(0.1)
    
    def _handle_critical_situation(self, threats: List[Dict]):
        """Handle critical collision risk"""
        critical_threats = [t for t in threats if t['risk_level'] == 'critical']
        
        if critical_threats:
            self.logger.error("CRITICAL COLLISION RISK - Emergency stop initiated")
            
            # Emergency stop maneuver
            self.current_maneuver = AvoidanceManeuver(
                maneuver_type='stop',
                target_speed=0.0,
                duration=2.0,
                urgency=5
            )
            
            # Trigger emergency callback
            if self.emergency_stop_callback:
                self.emergency_stop_callback("critical_collision_risk", critical_threats)
            
            self.avoidance_count += 1
    
    def _handle_high_risk_situation(self, threats: List[Dict]):
        """Handle high collision risk"""
        high_risk_threats = [t for t in threats if t['risk_level'] in ['high', 'medium']]
        
        if high_risk_threats:
            # Determine best avoidance maneuver
            if any(t['time_to_collision'] < 1.0 for t in high_risk_threats):
                # Immediate slow down
                self.current_maneuver = AvoidanceManeuver(
                    maneuver_type='slow_down',
                    target_speed=0.5,
                    duration=3.0,
                    urgency=3
                )
            else:
                # Plan alternative path
                self._replan_path_with_avoidance()
            
            # Trigger warning callback
            if self.collision_warning_callback:
                self.collision_warning_callback("high_collision_risk", high_risk_threats)
            
            self.avoidance_count += 1
    
    def _update_path_plan(self):
        """Update path planning based on current situation"""
        if not self.target_waypoints:
            return
        
        current_pos = self.current_vehicle_state.position
        next_waypoint = self.target_waypoints[0]
        
        # Plan path to next waypoint
        path, success = self.path_planner.plan_path(
            current_pos, 
            next_waypoint.position,
            list(self.obstacles.values()),
            self.current_vehicle_state
        )
        
        if success:
            self.planned_path = path
            if self.path_update_callback:
                path_data = [{'x': p.x, 'y': p.y} for p in path]
                self.path_update_callback(path_data, next_waypoint.desired_speed)
        else:
            self.logger.warning("Failed to find safe path to waypoint")
            self.planned_path = []
    
    def _replan_path_with_avoidance(self):
        """Replan path with explicit obstacle avoidance"""
        if self.target_waypoints:
            self._update_path_plan()
            self.path_replanning_count += 1
            self.logger.info("Path replanned for obstacle avoidance")
    
    def get_avoidance_status(self) -> Dict:
        """Get current avoidance system status"""
        with self.lock:
            # Calculate collision risk
            risk_score, threats = self.collision_detector.calculate_collision_risk(
                self.current_vehicle_state, list(self.obstacles.values())
            )
            
            # Get performance metrics
            uptime = time.time() - self.start_time
            
            status = {
                'system_active': self.active,
                'collision_risk': {
                    'score': round(risk_score, 3),
                    'level': self.collision_detector._risk_to_level(risk_score),
                    'threats_detected': len(threats),
                    'critical_threats': len([t for t in threats if t['risk_level'] == 'critical']),
                    'high_threats': len([t for t in threats if t['risk_level'] == 'high'])
                },
                'obstacles': {
                    'total_tracked': len(self.obstacles),
                    'active_obstacles': len([o for o in self.obstacles.values() 
                                           if time.time() - o.last_seen < 1.0])
                },
                'navigation': {
                    'waypoints_remaining': len(self.target_waypoints),
                    'path_length': len(self.planned_path),
                    'has_valid_path': len(self.planned_path) > 0
                },
                'current_maneuver': {
                    'type': self.current_maneuver.maneuver_type if self.current_maneuver else 'none',
                    'urgency': self.current_maneuver.urgency if self.current_maneuver else 0
                },
                'performance_metrics': {
                    'avoidance_actions': self.avoidance_count,
                    'collision_warnings': self.collision_warnings,
                    'path_replanning_count': self.path_replanning_count,
                    'system_uptime': round(uptime, 1)
                },
                'vehicle_state': {
                    'position': {'x': self.current_vehicle_state.position.x, 
                               'y': self.current_vehicle_state.position.y},
                    'speed': round(math.sqrt(self.current_vehicle_state.velocity.x**2 + 
                                          self.current_vehicle_state.velocity.y**2), 2),
                    'heading': round(math.degrees(self.current_vehicle_state.heading), 1)
                }
            }
            
            return status
    
    def set_collision_warning_callback(self, callback):
        """Set callback for collision warnings"""
        self.collision_warning_callback = callback
    
    def set_path_update_callback(self, callback):
        """Set callback for path updates"""
        self.path_update_callback = callback
    
    def set_emergency_stop_callback(self, callback):
        """Set callback for emergency stops"""
        self.emergency_stop_callback = callback

def test_predictive_obstacle_avoidance():
    """Test the predictive obstacle avoidance system"""
    print("🤖 Testing Predictive Obstacle Avoidance System...")
    
    # Initialize system
    avoidance = PredictiveObstacleAvoidanceSystem()
    
    # Set callbacks
    def collision_warning_handler(risk_type, threats):
        print(f"⚠️  COLLISION WARNING: {risk_type} - {len(threats)} threats")
    
    def path_update_handler(path, speed):
        print(f"🛣️  PATH UPDATED: {len(path)} points, target speed: {speed:.1f} m/s")
    
    def emergency_stop_handler(reason, details):
        print(f"🛑 EMERGENCY STOP: {reason} - {len(details)} critical threats")
    
    avoidance.set_collision_warning_callback(collision_warning_handler)
    avoidance.set_path_update_callback(path_update_handler)
    avoidance.set_emergency_stop_callback(emergency_stop_handler)
    
    avoidance.start_avoidance()
    
    try:
        # Set target waypoints
        waypoints = [
            {'position': {'x': 10.0, 'y': 0.0}, 'speed': 2.0},
            {'position': {'x': 20.0, 'y': 5.0}, 'speed': 1.5},
            {'position': {'x': 25.0, 'y': 10.0}, 'speed': 1.0}
        ]
        avoidance.set_target_waypoints(waypoints)
        
        # Simulation loop
        for i in range(100):
            # Update vehicle state (moving towards first waypoint)
            vehicle_x = i * 0.2
            vehicle_y = 0.0
            vehicle_speed = 2.0
            vehicle_heading = 0.0
            
            avoidance.update_vehicle_state(
                (vehicle_x, vehicle_y), 
                (vehicle_speed, 0.0), 
                vehicle_heading
            )
            
            # Add dynamic obstacles
            obstacles = []
            
            if i > 20:  # Static obstacle
                obstacles.append({
                    'id': 'static_obs',
                    'position': {'x': 8.0, 'y': 0.0},
                    'velocity': {'x': 0.0, 'y': 0.0},
                    'size': 1.5,
                    'confidence': 0.9
                })
            
            if i > 40 and i < 80:  # Moving obstacle
                obs_x = 15.0 - (i - 40) * 0.1
                obstacles.append({
                    'id': 'moving_obs',
                    'position': {'x': obs_x, 'y': 0.5},
                    'velocity': {'x': -1.0, 'y': 0.0},
                    'size': 1.0,
                    'confidence': 0.8
                })
            
            if i > 60:  # Critical collision scenario
                obstacles.append({
                    'id': 'critical_obs',
                    'position': {'x': vehicle_x + 3.0, 'y': 0.0},
                    'velocity': {'x': -3.0, 'y': 0.0},
                    'size': 1.2,
                    'confidence': 0.95
                })
            
            avoidance.update_obstacles(obstacles)
            
            # Print status every 20 iterations
            if i % 20 == 0:
                status = avoidance.get_avoidance_status()
                print(f"\n--- Iteration {i} Status ---")
                print(f"Vehicle Position: ({status['vehicle_state']['position']['x']:.1f}, "
                      f"{status['vehicle_state']['position']['y']:.1f})")
                print(f"Collision Risk: {status['collision_risk']['level']} "
                      f"({status['collision_risk']['score']:.3f})")
                print(f"Obstacles Tracked: {status['obstacles']['total_tracked']}")
                print(f"Active Threats: {status['collision_risk']['threats_detected']}")
                print(f"Path Valid: {status['navigation']['has_valid_path']}")
                print(f"Current Maneuver: {status['current_maneuver']['type']}")
                print(f"Avoidance Actions: {status['performance_metrics']['avoidance_actions']}")
            
            time.sleep(0.05)  # 20Hz simulation
        
        # Final results
        final_status = avoidance.get_avoidance_status()
        print("\n🎯 FINAL AVOIDANCE RESULTS:")
        print(f"✅ System Active: {final_status['system_active']}")
        print(f"🛡️  Total Avoidance Actions: {final_status['performance_metrics']['avoidance_actions']}")
        print(f"⚠️  Collision Warnings: {final_status['performance_metrics']['collision_warnings']}")
        print(f"🔄 Path Replanning Count: {final_status['performance_metrics']['path_replanning_count']}")
        print(f"⏱️  System Uptime: {final_status['performance_metrics']['system_uptime']:.1f}s")
        print(f"📍 Final Position: ({final_status['vehicle_state']['position']['x']:.1f}, "
              f"{final_status['vehicle_state']['position']['y']:.1f})")
        print(f"🎯 Final Risk Level: {final_status['collision_risk']['level']}")
        
        print("\n✅ Predictive Obstacle Avoidance System test completed successfully!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise
    
    finally:
        avoidance.stop_avoidance()

if __name__ == "__main__":
    test_predictive_obstacle_avoidance()

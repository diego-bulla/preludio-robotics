#!/usr/bin/env python3
# main.py - Dual Mode Robot Controller (Line Follower + Object Follower)

import time
import logging
from config import (
    VELOCIDAD_BASE,
    POSICION_CENTRAL,
    OPERATION_MODES,
    MOVEMENT,
    CAMERA,
    OBJECT_DETECTION
)
from controllers.motors import MotorController
from sensors.line_sensor import LineSensor
from controllers.camera import CameraController
from sensors.object_detector import ObjectDetector
from pid import PIDController
from utils.gpio_manager import GPIOManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot.log'),
        logging.StreamHandler()
    ]
)

class RobotController:
    def __init__(self):
        """Initialize all robot components"""
        GPIOManager.init()
        self.motors = MotorController()
        self.line_sensor = LineSensor()
        self.camera = CameraController()
        self.object_detector = ObjectDetector()
        self.pid = PIDController()
        self.current_mode = OPERATION_MODES['LINE_FOLLOWER']
        self.logger = logging.getLogger('RobotController')
        self.camera.start()
        self.logger.info("All components initialized")

    def toggle_mode(self):
        """Switch between operation modes"""
        if self.current_mode == OPERATION_MODES['LINE_FOLLOWER']:
            self.current_mode = OPERATION_MODES['OBJECT_FOLLOWER']
            self.motors.control_motors(0, 0)  # Stop before switching
            self.logger.info("Mode switched to OBJECT FOLLOWER")
        else:
            self.current_mode = OPERATION_MODES['LINE_FOLLOWER']
            self.motors.control_motors(0, 0)
            self.logger.info("Mode switched to LINE FOLLOWER")
        time.sleep(1)  # Pause for mode transition

    def follow_line(self):
        """Line following routine using PID control"""
        try:
            sensor_values = self.line_sensor.leer_sensores()
            position = self.line_sensor.calcular_posicion(sensor_values)

            if position is not None:
                correction = self.pid.calcular_correccion(position)
                left_speed = VELOCIDAD_BASE - correction
                right_speed = VELOCIDAD_BASE + correction
                self.motors.control_motors(left_speed, right_speed)
                self.logger.debug(f"Line Position: {position}, Correction: {correction:.2f}")
            else:
                self.motors.control_motors(0, 0)
                self.logger.warning("Line lost! Stopping motors")

        except Exception as e:
            self.logger.error(f"Line following error: {str(e)}")
            self.motors.control_motors(0, 0)

    def follow_object(self):
        """Object detection and following routine"""
        try:
            frame = self.camera.get_frame()
            detections = self.object_detector.detect(frame)

            if detections['green']:
                self._process_detection(detections['green'], "GREEN")
            elif detections['red']:
                self._process_detection(detections['red'], "RED")
            else:
                self._search_pattern()

        except Exception as e:
            self.logger.error(f"Object following error: {str(e)}")
            self.motors.control_motors(0, 0)

    def _process_detection(self, detection, color):
        """Handle detected object movement logic"""
        x, y, area = detection
        center_x = CAMERA['resolution'][0] // 2
        threshold = MOVEMENT['centering_threshold']

        if abs(x - center_x) < threshold:
            speed = MOVEMENT['approach_speed']
            self.motors.control_motors(speed, speed)
            self.logger.info(f"{color}: Approaching (Area: {area} px²)")
        else:
            turn_speed = MOVEMENT['centering_speed'] * (abs(x-center_x)/center_x
            if x < center_x:
                self.motors.control_motors(turn_speed, -turn_speed)
            else:
                self.motors.control_motors(-turn_speed, turn_speed)
            self.logger.debug(f"{color}: Adjusting position (X: {x}, Target: {center_x})")

    def _search_pattern(self):
        """Search pattern when no objects are detected"""
        left_speed = -int(MOVEMENT['search_speed'] * 0.7)
        right_speed = MOVEMENT['search_speed']
        self.motors.control_motors(left_speed, right_speed)
        self.logger.debug("Searching: Spiral pattern")

    def run(self):
        """Main control loop"""
        self.logger.info("Starting robot controller")
        try:
            while True:
                if self.current_mode == OPERATION_MODES['LINE_FOLLOWER']:
                    self.follow_line()
                else:
                    self.follow_object()
                
                # Small delay to prevent CPU overload
                time.sleep(0.05)

        except KeyboardInterrupt:
            self.logger.info("Shutdown signal received")
            self.shutdown()

    def shutdown(self):
        """Cleanup resources"""
        self.motors.detener()
        self.camera.stop()
        GPIOManager.cleanup()
        self.logger.info("Robot shutdown complete")

if __name__ == "__main__":
    robot = RobotController()
    
    # For testing mode switching (could be triggered by a button in real use)
    # robot.toggle_mode()  
    
    robot.run()

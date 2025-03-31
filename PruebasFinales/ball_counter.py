#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ball Counter with Movement Playback (Headless Mode)
Counts colored balls using OpenCV while playing back pre-recorded movements
Supports headless operation for Raspberry Pi
"""

import time
import threading
import os
import json
import cv2
import numpy as np
import RPi.GPIO as GPIO
from collections import deque
from datetime import datetime

# =============== CONFIGURATION ===============
# Motor and Servo pins
# Left DC Motor
MOTOR_LEFT_EN = 13
MOTOR_LEFT_IN1 = 19
MOTOR_LEFT_IN2 = 16

# Right DC Motor
MOTOR_RIGHT_EN = 21
MOTOR_RIGHT_IN1 = 26
MOTOR_RIGHT_IN2 = 20

# Speed Configuration
MAX_SPEED = 100
BASE_SPEED = 90
TURN_SPEED = 70
SLOW_SPEED = 60

# PWM Configuration
PWM_FREQUENCY = 100

# Servo pins
SERVO_LEFT_PIN = 6
SERVO_RIGHT_PIN = 12

# Camera Configuration
CAMERA_INDEX = 0
CAMERA_RESOLUTION = (640, 480)

# Color Detection Configuration
BALL_MIN_RADIUS = 10  # Minimum ball radius in pixels
BALL_MAX_RADIUS = 100  # Maximum ball radius in pixels

# Color ranges in HSV (these may need calibration for your environment)
COLOR_RANGES = {
    'red': [
        ((0, 100, 100), (10, 255, 255)),  # Lower red range
        ((160, 100, 100), (180, 255, 255))  # Upper red range (wraps around hue)
    ],
    'green': [((35, 100, 100), (85, 255, 255))],
    'blue': [((100, 100, 100), (140, 255, 255))],
    'yellow': [((20, 100, 100), (35, 255, 255))]
}

# Directory to save data
DATA_DIR = "robot_data"
RECORDINGS_DIR = "movement_recordings"  # Changed to match original recorder's directory
DETECTION_DIR = os.path.join(DATA_DIR, "detection_data")

# =============== MOTOR CONTROL CLASS ===============
class MotorControl:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(MOTOR_LEFT_EN, GPIO.OUT)
        GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_LEFT_IN2, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_EN, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_IN2, GPIO.OUT)

        GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
        GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)

        self.pwm_left = GPIO.PWM(MOTOR_LEFT_EN, PWM_FREQUENCY)
        self.pwm_right = GPIO.PWM(MOTOR_RIGHT_EN, PWM_FREQUENCY)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def set_motor_left(self, speed):
        speed = max(-100, min(100, speed))
        if speed > 0:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(speed)
        elif speed < 0:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(speed))
        else:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(0)

    def set_motor_right(self, speed):
        speed = max(-100, min(100, speed))
        if speed > 0:
            GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(speed)
        elif speed < 0:
            GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_RIGHT_IN2, GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(abs(speed))
        else:
            GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(0)

    def move(self, left_speed, right_speed):
        # Invert both motors for correct forward/backward movement
        self.set_motor_left(-left_speed)
        self.set_motor_right(right_speed)  # Already inverted in original code

    def stop(self):
        self.move(0, 0)

    def cleanup(self):
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()

# =============== SERVO CONTROL CLASS ===============
class ServoControl:
    def __init__(self):
        GPIO.setup(SERVO_LEFT_PIN, GPIO.OUT)
        GPIO.setup(SERVO_RIGHT_PIN, GPIO.OUT)

        self.servo_left = GPIO.PWM(SERVO_LEFT_PIN, 50)
        self.servo_right = GPIO.PWM(SERVO_RIGHT_PIN, 50)

        self.servo_left.start(7.5)
        self.servo_right.start(7.5)
        time.sleep(0.5)

        self.servo_left.ChangeDutyCycle(0)
        self.servo_right.ChangeDutyCycle(0)

    def move_servo(self, servo, angle, duration=0.5):
        duty_cycle = 2.5 + (angle / 180.0) * 10.0

        if servo == 'left':
            pwm = self.servo_left
        else:
            pwm = self.servo_right

        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(duration)
        pwm.ChangeDutyCycle(7.5)  # Return to neutral
        time.sleep(0.5)
        pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.servo_left.stop()
        self.servo_right.stop()

# =============== COLOR DETECTOR CLASS ===============
class ColorDetector:
    def __init__(self, camera_index=0, resolution=(640, 480), headless=True):
        self.camera_index = camera_index
        self.resolution = resolution
        self.camera = None
        self.frame_buffer = deque(maxlen=5)  # Buffer for recent frames
        self.headless = headless  # Headless mode flag

        # Detection history for each color
        self.color_counts = {color: 0 for color in COLOR_RANGES.keys()}
        self.total_detections = 0

        # Running flag
        self.running = False
        self.detection_thread = None

        # Create data directory if it doesn't exist
        if not os.path.exists(DETECTION_DIR):
            os.makedirs(DETECTION_DIR)

        # Set up ML parameters
        self.ml_model = None
        self.load_or_create_model()

    def start(self):
        """Start the color detector"""
        if self.running:
            print("Color detector already running")
            return False

        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

            # Check if camera opened successfully
            if not self.camera.isOpened():
                print("Failed to open camera")
                return False

            # Read a test frame to verify camera is working
            ret, test_frame = self.camera.read()
            if not ret or test_frame is None:
                print("Failed to read frame from camera")
                return False

            self.running = True
            self.detection_thread = threading.Thread(target=self._detection_loop)
            self.detection_thread.daemon = True
            self.detection_thread.start()

            print("Color detector started" + (" (headless mode)" if self.headless else ""))
            return True

        except Exception as e:
            print(f"Error starting color detector: {e}")
            if self.camera is not None:
                self.camera.release()
            return False

    def stop(self):
        """Stop the color detector"""
        self.running = False

        if self.detection_thread:
            self.detection_thread.join(timeout=2.0)

        if self.camera:
            self.camera.release()

        if not self.headless:
            cv2.destroyAllWindows()

        print("Color detector stopped")

    def get_detections(self):
        """Get current detection counts"""
        return {
            'color_counts': self.color_counts.copy(),
            'total': self.total_detections
        }

    def save_detection_data(self, filename=None):
        """Save detection data to a file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"detection_data_{timestamp}.json"

        file_path = os.path.join(DETECTION_DIR, filename)

        data = {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'color_counts': self.color_counts,
            'total_detections': self.total_detections
        }

        with open(file_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Detection data saved to {file_path}")
        return file_path

    def reset_counts(self):
        """Reset all detection counts"""
        self.color_counts = {color: 0 for color in COLOR_RANGES.keys()}
        self.total_detections = 0
        print("Detection counts reset")

    def load_or_create_model(self):
        """Load pre-trained model or create a simple one"""
        try:
            from sklearn.ensemble import RandomForestClassifier
            from joblib import dump, load

            model_path = os.path.join(DETECTION_DIR, "color_classifier.joblib")

            if os.path.exists(model_path):
                # Load existing model
                self.ml_model = load(model_path)
                print("Loaded existing color classifier model")
            else:
                # Create a simple model
                self.ml_model = RandomForestClassifier(n_estimators=100, random_state=42)

                # Create some synthetic training data
                # This would typically come from actual labeled training examples
                # For simplicity, we're creating a basic model that just looks at color

                X_train = []  # Features: average H, S, V values
                y_train = []  # Labels: color names

                # Generate synthetic data for each color
                for color_name, ranges in COLOR_RANGES.items():
                    for range_pair in ranges:
                        lower, upper = range_pair

                        # Create 20 random points in this color range
                        for _ in range(20):
                            h = np.random.uniform(lower[0], upper[0])
                            s = np.random.uniform(lower[1], upper[1])
                            v = np.random.uniform(lower[2], upper[2])

                            # Add some noise
                            area = np.random.uniform(500, 5000)  # ball area
                            circularity = np.random.uniform(0.7, 1.0)  # circularity measure

                            X_train.append([h, s, v, area, circularity])
                            y_train.append(color_name)

                # Train the model
                self.ml_model.fit(X_train, y_train)

                # Save the model
                dump(self.ml_model, model_path)
                print("Created and saved new color classifier model")

        except Exception as e:
            print(f"Error loading/creating ML model: {e}")
            self.ml_model = None

    def _detection_loop(self):
        """Main detection loop"""
        last_save_time = time.time()
        save_interval = 60  # Save data every minute
        last_status_time = 0
        status_interval = 5  # Print status every 5 seconds (only in headless mode)

        try:
            while self.running:
                # Read frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to read from camera")
                    time.sleep(0.1)
                    continue

                # Add to buffer
                self.frame_buffer.append(frame)

                # Detect colored balls
                detections = self._detect_balls(frame)

                # If not headless, show the frame with detection visualization
                if not self.headless:
                    # Display results on frame
                    output_frame = frame.copy()

                    for color, balls in detections.items():
                        for (x, y, r) in balls:
                            # Draw circle and label
                            color_bgr = self._get_display_color(color)
                            cv2.circle(output_frame, (x, y), r, color_bgr, 2)
                            cv2.putText(output_frame, color, (x - r, y - r - 10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

                    # Show stats
                    y_pos = 30
                    cv2.putText(output_frame, f"Total balls: {self.total_detections}",
                               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    y_pos += 30
                    for color, count in self.color_counts.items():
                        color_bgr = self._get_display_color(color)
                        cv2.putText(output_frame, f"{color}: {count}",
                                   (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
                        y_pos += 30

                    # Show the frame
                    cv2.imshow("Color Detection", output_frame)

                    # Exit on 'q' key
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    # In headless mode, periodically print status to console
                    current_time = time.time()
                    if current_time - last_status_time >= status_interval:
                        status = f"Total balls: {self.total_detections} - "
                        for color, count in self.color_counts.items():
                            status += f"{color}: {count}, "
                        print(status[:-2])  # Remove the last comma and space
                        last_status_time = current_time

                # Save data periodically
                current_time = time.time()
                if current_time - last_save_time >= save_interval:
                    self.save_detection_data()
                    last_save_time = current_time

                # Small delay
                time.sleep(0.01)

        except Exception as e:
            print(f"Error in detection loop: {e}")
            import traceback
            traceback.print_exc()

    def _detect_balls(self, frame):
        """Detect colored balls in the frame"""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Results dictionary by color
        detections = {}
        all_detected = 0

        # Process each color
        for color, ranges in COLOR_RANGES.items():
            color_mask = None

            # Apply each range for this color
            for range_pair in ranges:
                lower, upper = range_pair

                # Create mask for this range
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

                # Combine with previous masks for this color
                if color_mask is None:
                    color_mask = mask
                else:
                    color_mask = cv2.bitwise_or(color_mask, mask)

            # Process mask to remove noise
            kernel = np.ones((5, 5), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Process each contour
            balls = []
            for contour in contours:
                # Calculate area and perimeter
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)

                # Skip if too small
                if area < 500:  # Minimum area to consider
                    continue

                # Calculate circularity
                circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

                # Skip if not circular enough
                if circularity < 0.7:
                    continue

                # Get enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                # Skip if too small or too large
                if radius < BALL_MIN_RADIUS or radius > BALL_MAX_RADIUS:
                    continue

                # Use ML model for verification if available
                if self.ml_model is not None:
                    # Extract ROI
                    mask_roi = np.zeros_like(color_mask)
                    cv2.drawContours(mask_roi, [contour], 0, 255, -1)
                    hsv_mean = cv2.mean(hsv, mask=mask_roi)[:3]

                    # Create feature vector
                    features = [hsv_mean[0], hsv_mean[1], hsv_mean[2], area, circularity]

                    # Make prediction to verify color
                    try:
                        predicted_color = self.ml_model.predict([features])[0]
                        # Only accept if prediction matches the color being checked
                        if predicted_color != color:
                            continue
                    except Exception as e:
                        print(f"ML prediction error: {e}")

                # Add to balls list
                balls.append(center + (radius,))

            # Update total count
            if balls:
                # Count unique balls (avoiding duplicates)
                num_new_balls = len(balls)
                detections[color] = balls

                # Update global count
                self.color_counts[color] += num_new_balls
                all_detected += num_new_balls

        # Update total detections
        self.total_detections += all_detected

        return detections

    def _get_display_color(self, color_name):
        """Get BGR values for displaying a color"""
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255)
        }
        return color_map.get(color_name, (255, 255, 255))

# =============== BALL COUNTER WITH PLAYBACK ===============
class BallCounter:
    def __init__(self, headless=True):
        # Create directories if they don't exist
        if not os.path.exists(DATA_DIR):
            os.makedirs(DATA_DIR)
        if not os.path.exists(RECORDINGS_DIR):
            # Check if recordings directory exists
            if not os.path.exists(RECORDINGS_DIR):
                print(f"Creating recordings directory: {RECORDINGS_DIR}")
                os.makedirs(RECORDINGS_DIR)

        # Initialize subsystems
        self.motor_controller = MotorControl()
        self.servo_controller = ServoControl()
        self.color_detector = ColorDetector(CAMERA_INDEX, CAMERA_RESOLUTION, headless)
        self.headless = headless

        # State tracking
        self.running = False
        self.playback_in_progress = False

        print("Ball counter initialized" + (" (headless mode)" if headless else ""))

    def start(self):
        """Start the ball counter"""
        if self.running:
            print("Ball counter already running")
            return

        # Start color detector
        if self.color_detector.start():
            # Set running state
            self.running = True
            print("Ball counter started")
        else:
            print("Failed to start ball counter")

    def stop(self):
        """Stop the ball counter"""
        self.running = False

        # Stop color detector
        self.color_detector.stop()

        # Stop motors
        self.motor_controller.stop()

        print("Ball counter stopped")

    def playback_movement(self, movement_file, speed_factor=1.0):
        """Play back a recorded movement sequence"""
        if self.playback_in_progress:
            print("Playback already in progress")
            return False

        self.playback_in_progress = True
        file_path = os.path.join(RECORDINGS_DIR, movement_file)

        try:
            with open(file_path, 'r') as f:
                sequence = json.load(f)

            print(f"Playing back movement sequence '{movement_file}' at speed factor {speed_factor}")
            print(f"Starting with detection counts: {self.color_detector.get_detections()['color_counts']}")

            # Play each movement
            for i, move in enumerate(sequence):
                print(f"Action {i+1}/{len(sequence)}")

                if move['type'] == 'motor':
                    left_speed = move['left_speed']
                    right_speed = move['right_speed']
                    duration = move['duration'] / speed_factor

                    print(f"Motor move: L={left_speed}, R={right_speed}, Duration={duration:.2f}s")
                    self.motor_controller.move(left_speed, right_speed)
                    time.sleep(duration)

                elif move['type'] == 'servo':
                    servo = move['servo']
                    angle = move['angle']
                    duration = move['duration'] / speed_factor

                    print(f"Servo move: {servo} to {angle}°, Duration={duration:.2f}s")
                    self.servo_controller.move_servo(servo, angle, duration)

            # Stop at the end
            self.motor_controller.stop()
            print("Playback complete")
            print(f"Final detection counts: {self.color_detector.get_detections()['color_counts']}")
            self.playback_in_progress = False
            return True

        except KeyboardInterrupt:
            print("Playback interrupted")
            self.motor_controller.stop()
            self.playback_in_progress = False
            return False
        except Exception as e:
            print(f"Error playing back movement: {e}")
            self.motor_controller.stop()
            self.playback_in_progress = False
            return False

    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.motor_controller.cleanup()
        self.servo_controller.cleanup()
        GPIO.cleanup()
        print("Cleanup complete")

# =============== MAIN FUNCTION ===============
def main():
    try:
        # Check for headless mode argument
        import sys
        headless = True  # Default to headless mode

        if len(sys.argv) > 1 and sys.argv[1].lower() == 'gui':
            headless = False
            print("Starting in GUI mode")
        else:
            print("Starting in headless mode (no GUI)")
            print("Run with 'python script.py gui' to enable GUI mode if your system supports it")

        # Initialize ball counter
        counter = BallCounter(headless=headless)

        # Main menu
        while True:
            print("\n=== Ball Counter Menu ===")
            print("1: Start ball detection")
            print("2: Play back a movement recording")
            print("3: Reset ball counts")
            print("4: Save detection data")
            print("5: Check current counts")
            print("6: Exit")

            choice = input("Enter choice: ")

            if choice == '1':
                counter.start()

            elif choice == '2':
                # First, make sure detection is running
                if not counter.running:
                    counter.start()

                # List available recordings
                if not os.path.exists(RECORDINGS_DIR):
                    print(f"Recordings directory not found: {RECORDINGS_DIR}")
                    continue

                recordings = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')]

                if not recordings:
                    print("No recordings found")
                    continue

                print("\nAvailable recordings:")
                for i, rec in enumerate(recordings):
                    print(f"{i+1}: {rec}")

                try:
                    selection = int(input("\nEnter recording number (0 to cancel): "))
                    if selection == 0:
                        continue

                    if 1 <= selection <= len(recordings):
                        speed = input("Enter playback speed (1.0 = normal): ")
                        if not speed:
                            speed = 1.0
                        else:
                            speed = float(speed)
                        counter.playback_movement(recordings[selection-1], speed)
                    else:
                        print("Invalid selection")
                except ValueError:
                    print("Invalid input")

            elif choice == '3':
                if counter.running:
                    counter.color_detector.reset_counts()
                else:
                    print("Detection not running")

            elif choice == '4':
                if counter.running:
                    counter.color_detector.save_detection_data()
                else:
                    print("Detection not running")

            elif choice == '5':
                if counter.running:
                    detections = counter.color_detector.get_detections()
                    print("\nCurrent ball counts:")
                    print(f"Total: {detections['total']}")
                    for color, count in detections['color_counts'].items():
                        print(f"{color}: {count}")
                else:
                    print("Detection not running")

            elif choice == '6':
                print("Exiting...")
                break

            else:
                print("Invalid choice")

    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        try:
            counter.cleanup()
        except:
            pass

if __name__ == "__main__":
    main()

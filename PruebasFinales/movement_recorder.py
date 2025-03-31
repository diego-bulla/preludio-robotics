#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Movement Recorder and Playback Script for Robot"""

import time
import sys
import json
import os
import select
import RPi.GPIO as GPIO
from datetime import datetime

# =============== CONFIGURATION ===============
# Left DC Motor
MOTOR_LEFT_EN = 13
MOTOR_LEFT_IN1 = 19
MOTOR_LEFT_IN2 = 16

# Right DC Motor
MOTOR_RIGHT_EN = 21
MOTOR_RIGHT_IN1 = 26
MOTOR_RIGHT_IN2 = 20

# Speed Configuration (Increased)
MAX_SPEED = 100
BASE_SPEED = 80  # Increased from 70
TURN_SPEED = 40  # Increased from 50
SLOW_SPEED = 30  # Increased from 40

# PWM Configuration
PWM_FREQUENCY = 100

# Servo pins
SERVO_LEFT_PIN = 6
SERVO_RIGHT_PIN = 12

# Directory for saving recordings
# MODIFIED: Changed directory structure to match move_and_count.py
DATA_DIR = "robot_data"
RECORDINGS_DIR = os.path.join(DATA_DIR, "movement_recordings")

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

        # Start PWM and explicitly set servos to 0 degrees
        self.servo_left.start(0)
        self.servo_right.start(0)
        self._set_angle('left', 0)
        self._set_angle('right', 0)
        time.sleep(0.5)

        # Stop PWM to prevent jitter
        self.servo_left.ChangeDutyCycle(0)
        self.servo_right.ChangeDutyCycle(0)

    def _set_angle(self, servo, angle):
        """Set servo to specific angle (0-180)"""
        # Convert angle to duty cycle (0 degrees = 2.5%, 180 degrees = 12.5%)
        duty_cycle = 2.5 + (angle / 180.0) * 10.0

        if servo == 'left':
            self.servo_left.ChangeDutyCycle(duty_cycle)
        else:
            self.servo_right.ChangeDutyCycle(duty_cycle)

    def move_servo(self, servo, angle, duration=0.5):
        """
        Move servo from 0 to specified angle and back to 0
        This creates a complete push-pull motion for the rack
        """
        # Step 1: Ensure we start at 0 degrees
        self._set_angle(servo, 0)
        time.sleep(0.2)  # Short delay to ensure it reaches position

        # Step 2: Move to target angle
        self._set_angle(servo, angle)
        time.sleep(duration)  # Hold at extended position

        # Step 3: Return to 0 degrees
        self._set_angle(servo, 0)
        time.sleep(0.2)  # Short delay to ensure it returns

        # Stop PWM to prevent jitter
        if servo == 'left':
            self.servo_left.ChangeDutyCycle(0)
        else:
            self.servo_right.ChangeDutyCycle(0)

        print(f"{servo} servo completed 0→{angle}→0 degrees movement")

    def cleanup(self):
        # Set both servos to 0 degrees before stopping
        self._set_angle('left', 0)
        self._set_angle('right', 0)
        time.sleep(0.2)

        self.servo_left.stop()
        self.servo_right.stop()

# =============== MOVEMENT RECORDER CLASS ===============
class MovementRecorder:
    def __init__(self, motor_controller, servo_controller):
        self.motor_controller = motor_controller
        self.servo_controller = servo_controller
        self.recording = []
        self.recording_in_progress = False
        self.start_time = None

        # MODIFIED: Create parent directory if it doesn't exist
        if not os.path.exists(DATA_DIR):
            os.makedirs(DATA_DIR)
        if not os.path.exists(RECORDINGS_DIR):
            os.makedirs(RECORDINGS_DIR)

    def start_recording(self):
        self.recording = []
        self.recording_in_progress = True
        self.start_time = time.time()
        print("Recording started. Perform your movements now...")

    def stop_recording(self):
        if self.recording_in_progress:
            self.recording_in_progress = False
            print(f"Recording stopped. Recorded {len(self.recording)} actions.")
            return True
        return False

    def record_movement(self, left_speed, right_speed, duration=None):
        if self.recording_in_progress:
            if duration is None:
                current_time = time.time()
                elapsed = 0 if not self.recording else current_time - self.recording[-1]['timestamp']
                timestamp = current_time
            else:
                elapsed = duration
                timestamp = time.time()

            self.recording.append({
                'type': 'motor',
                'left_speed': left_speed,
                'right_speed': right_speed,
                'duration': elapsed,
                'timestamp': timestamp
            })

    def record_servo(self, servo, angle, duration):
        if self.recording_in_progress:
            self.recording.append({
                'type': 'servo',
                'servo': servo,
                'angle': angle,
                'duration': duration,
                'timestamp': time.time()
            })

    def save_recording(self, name=None):
        if not self.recording:
            print("No recording to save.")
            return None

        if name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"movement_{timestamp}"

        clean_recording = []
        for move in self.recording:
            clean_move = {k: v for k, v in move.items() if k != 'timestamp'}
            clean_recording.append(clean_move)

        file_path = os.path.join(RECORDINGS_DIR, f"{name}.json")

        with open(file_path, 'w') as f:
            json.dump(clean_recording, f, indent=2)

        print(f"Recording saved to {file_path}")
        return file_path

    def export_playback_function(self, file_path, output_file=None):
        try:
            with open(file_path, 'r') as f:
                recording = json.load(f)

            if not recording:
                print("Recording is empty, nothing to export.")
                return None

            base_name = os.path.basename(file_path)
            func_name = os.path.splitext(base_name)[0].lower().replace(' ', '_')

            if output_file is None:
                output_dir = os.path.dirname(file_path)
                output_file = os.path.join(output_dir, f"play_{func_name}.py")

            function_content = f"""#!/usr/bin/env python3
# -*- coding: utf-8 -*-

\"\"\"Auto-generated playback function for {base_name}\"\"\"

import time
import RPi.GPIO as GPIO

def setup_motors_and_servos():
    # Configure GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor pins
    MOTOR_LEFT_EN = 13
    MOTOR_LEFT_IN1 = 19
    MOTOR_LEFT_IN2 = 16
    MOTOR_RIGHT_EN = 21
    MOTOR_RIGHT_IN1 = 26
    MOTOR_RIGHT_IN2 = 20

    # Servo pins
    SERVO_LEFT_PIN = 6
    SERVO_RIGHT_PIN = 12

    # Configure motor pins
    GPIO.setup(MOTOR_LEFT_EN, GPIO.OUT)
    GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_LEFT_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_EN, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_IN2, GPIO.OUT)

    # Configure servo pins
    GPIO.setup(SERVO_LEFT_PIN, GPIO.OUT)
    GPIO.setup(SERVO_RIGHT_PIN, GPIO.OUT)

    # Create PWM objects for motors
    pwm_left = GPIO.PWM(MOTOR_LEFT_EN, 100)
    pwm_right = GPIO.PWM(MOTOR_RIGHT_EN, 100)

    # Create PWM objects for servos
    servo_left = GPIO.PWM(SERVO_LEFT_PIN, 50)
    servo_right = GPIO.PWM(SERVO_RIGHT_PIN, 50)

    # Start PWM
    pwm_left.start(0)
    pwm_right.start(0)
    servo_left.start(7.5)
    servo_right.start(7.5)
    time.sleep(0.5)
    servo_left.ChangeDutyCycle(0)
    servo_right.ChangeDutyCycle(0)

    return {{
        'motors': {{
            'pwm_left': pwm_left,
            'pwm_right': pwm_right,
            'pins': {{
                'MOTOR_LEFT_IN1': MOTOR_LEFT_IN1,
                'MOTOR_LEFT_IN2': MOTOR_LEFT_IN2,
                'MOTOR_RIGHT_IN1': MOTOR_RIGHT_IN1,
                'MOTOR_RIGHT_IN2': MOTOR_RIGHT_IN2
            }}
        }},
        'servos': {{
            'servo_left': servo_left,
            'servo_right': servo_right
        }}
    }}

def move_motors(motors, left_speed, right_speed):
    # Extract motor objects
    pwm_left = motors['pwm_left']
    pwm_right = motors['pwm_right']
    pins = motors['pins']

    # Invert motor directions for correct movement
    left_speed = -left_speed

    # Set left motor
    if left_speed > 0:
        GPIO.output(pins['MOTOR_LEFT_IN1'], GPIO.HIGH)
        GPIO.output(pins['MOTOR_LEFT_IN2'], GPIO.LOW)
        pwm_left.ChangeDutyCycle(left_speed)
    elif left_speed < 0:
        GPIO.output(pins['MOTOR_LEFT_IN1'], GPIO.LOW)
        GPIO.output(pins['MOTOR_LEFT_IN2'], GPIO.HIGH)
        pwm_left.ChangeDutyCycle(abs(left_speed))
    else:
        GPIO.output(pins['MOTOR_LEFT_IN1'], GPIO.LOW)
        GPIO.output(pins['MOTOR_LEFT_IN2'], GPIO.LOW)
        pwm_left.ChangeDutyCycle(0)

    # Set right motor (with direction already inverted)
    if right_speed > 0:
        GPIO.output(pins['MOTOR_RIGHT_IN1'], GPIO.HIGH)
        GPIO.output(pins['MOTOR_RIGHT_IN2'], GPIO.LOW)
        pwm_right.ChangeDutyCycle(right_speed)
    elif right_speed < 0:
        GPIO.output(pins['MOTOR_RIGHT_IN1'], GPIO.LOW)
        GPIO.output(pins['MOTOR_RIGHT_IN2'], GPIO.HIGH)
        pwm_right.ChangeDutyCycle(abs(right_speed))
    else:
        GPIO.output(pins['MOTOR_RIGHT_IN1'], GPIO.LOW)
        GPIO.output(pins['MOTOR_RIGHT_IN2'], GPIO.LOW)
        pwm_right.ChangeDutyCycle(0)

def move_servo(servos, servo_name, angle, duration):
    # Get the correct servo
    if servo_name == 'left':
        servo = servos['servo_left']
    else:
        servo = servos['servo_right']

    # Calculate duty cycle from angle
    duty_cycle = 2.5 + (angle / 180.0) * 10.0

    # Move servo
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(duration)
    servo.ChangeDutyCycle(7.5)  # Return to neutral
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def stop_all(controllers):
    # Stop motors
    move_motors(controllers['motors'], 0, 0)

    # Stop PWM
    controllers['motors']['pwm_left'].stop()
    controllers['motors']['pwm_right'].stop()
    controllers['servos']['servo_left'].stop()
    controllers['servos']['servo_right'].stop()

def play_{func_name}(speed_factor=1.0):
    \"\"\"
    Play back the recorded movement sequence

    Args:
        speed_factor (float): Factor to adjust playback speed (default: 1.0)
    \"\"\"
    print(f"Playing back movement sequence '{func_name}' at speed factor {{speed_factor}}")

    try:
        # Setup motors and servos
        controllers = setup_motors_and_servos()

        # Movement sequence
        sequence = {recording}

        # Play each movement
        for i, move in enumerate(sequence):
            print(f"Action {{i+1}}/{{len(sequence)}}")

            if move['type'] == 'motor':
                left_speed = move['left_speed']
                right_speed = move['right_speed']
                duration = move['duration'] / speed_factor

                print(f"Motor move: L={{left_speed}}, R={{right_speed}}, Duration={{duration:.2f}}s")
                move_motors(controllers['motors'], left_speed, right_speed)
                time.sleep(duration)

            elif move['type'] == 'servo':
                servo = move['servo']
                angle = move['angle']
                duration = move['duration'] / speed_factor

                print(f"Servo move: {{servo}} to {{angle}}°, Duration={{duration:.2f}}s")
                move_servo(controllers['servos'], servo, angle, duration)

        # Stop everything when done
        stop_all(controllers)
        print("Playback complete")

    except KeyboardInterrupt:
        print("Playback interrupted")
        try:
            stop_all(controllers)
        except:
            pass
    except Exception as e:
        print(f"Error during playback: {{e}}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    play_{func_name}()
"""

            with open(output_file, 'w') as f:
                f.write(function_content)

            print(f"Exported playback function to {output_file}")
            return output_file

        except Exception as e:
            print(f"Error exporting playback function: {e}")
            return None

    def load_recording(self, file_path):
        try:
            with open(file_path, 'r') as f:
                self.recording = json.load(f)
            print(f"Loaded recording from {file_path} with {len(self.recording)} actions.")
            return True
        except Exception as e:
            print(f"Error loading recording: {e}")
            return False

    def playback_recording(self, speed_factor=1.0):
        if not self.recording:
            print("No recording to play back.")
            return

        print(f"Playing back recording with speed factor: {speed_factor}")

        try:
            for i, move in enumerate(self.recording):
                print(f"Action {i+1}/{len(self.recording)}")

                if move['type'] == 'motor':
                    print(f"Motor move: L={move['left_speed']}, R={move['right_speed']}, Duration={move['duration']:.2f}s")
                    self.motor_controller.move(move['left_speed'], move['right_speed'])
                    adjusted_duration = move['duration'] / speed_factor
                    time.sleep(adjusted_duration)

                elif move['type'] == 'servo':
                    print(f"Servo move: {move['servo']} to {move['angle']}°, Duration={move['duration']:.2f}s")
                    self.servo_controller.move_servo(move['servo'], move['angle'], move['duration'] / speed_factor)

            self.motor_controller.stop()
            print("Playback complete.")

        except KeyboardInterrupt:
            print("Playback interrupted.")
            self.motor_controller.stop()
        except Exception as e:
            print(f"Error during playback: {e}")
            self.motor_controller.stop()

    def list_recordings(self):
        if not os.path.exists(RECORDINGS_DIR):
            print(f"Recordings directory {RECORDINGS_DIR} does not exist.")
            return []

        recordings = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')]

        if not recordings:
            print("No recordings found.")
        else:
            print("Available recordings:")
            for i, rec in enumerate(recordings):
                print(f"{i+1}. {rec}")

        return recordings

# =============== INTERACTIVE RECORDER CLASS ===============
class InteractiveRecorder:
    def __init__(self, motor_controller, servo_controller):
        self.motor_controller = motor_controller
        self.servo_controller = servo_controller
        self.recorder = MovementRecorder(motor_controller, servo_controller)
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.last_update_time = time.time()
        # New variables for toggle-based movement
        self.active_movements = {
            'w': False,  # Forward
            's': False,  # Backward
            'a': False,  # Left
            'd': False   # Right
        }
        # Last key press time to prevent immediate toggle
        self.last_key_press = {k: 0 for k in self.active_movements.keys()}
        self.key_debounce_time = 0.3  # 300ms debounce to prevent accidental toggles

    def start_interactive_session(self):
        print("\n=== Interactive Movement Recording ===")
        print("Use the following keys to control the robot:")
        print("  W: Toggle Forward")
        print("  S: Toggle Backward")
        print("  A: Toggle Left Turn")
        print("  D: Toggle Right Turn")
        print("  Q: Activate Left Servo (move to 180°)")
        print("  E: Activate Right Servo (move to 180°)")
        print("  X: Stop All Movement")
        print("  R: Start/Stop Recording")
        print("  P: Save Recording")
        print("  ESC or SPACE: End Session")
        print("\nNOTE: Movement keys now work as toggles - press once to start, press again to stop")

        self.motor_controller.stop()
        is_recording = False

        try:
            import termios, tty, sys, threading

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

            last_speed_update = time.time()
            speed_update_interval = 0.1

            print("\nControls active. Press ESC or SPACE to exit.")

            while True:
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    char = sys.stdin.read(1)

                    if char == '\x1b' or char == ' ':
                        print("\nExiting interactive mode...")
                        break

                    current_time = time.time()
                    char = char.lower()

                    # Handle movement toggle keys with debounce
                    if char in self.active_movements:
                        if current_time - self.last_key_press[char] >= self.key_debounce_time:
                            self.active_movements[char] = not self.active_movements[char]

                            # Stop opposing movements
                            if char == 'w' and self.active_movements[char]:
                                self.active_movements['s'] = False
                            elif char == 's' and self.active_movements[char]:
                                self.active_movements['w'] = False

                            # Update last press time
                            self.last_key_press[char] = current_time

                    # Handle other commands
                    if char == 'r':
                        if not is_recording:
                            self.recorder.start_recording()
                            is_recording = True
                        else:
                            self.recorder.stop_recording()
                            is_recording = False
                    elif char == 'p':
                        if is_recording:
                            self.recorder.stop_recording()
                            is_recording = False
                        self.recorder.save_recording()
                    elif char == 'q':
                        # Record and activate left servo
                        if is_recording:
                            self.recorder.record_servo('left', 180, 0.5)
                        servo_thread = threading.Thread(target=self.servo_controller.move_servo, args=('left', 180, 0.5))
                        servo_thread.daemon = True
                        servo_thread.start()
                    elif char == 'e':
                        # Record and activate right servo
                        if is_recording:
                            self.recorder.record_servo('right', 180, 0.5)
                        servo_thread = threading.Thread(target=self.servo_controller.move_servo, args=('right', 180, 0.5))
                        servo_thread.daemon = True
                        servo_thread.start()
                    elif char == 'x':
                        # Reset all active movements
                        for k in self.active_movements:
                            self.active_movements[k] = False
                        self.current_left_speed = 0
                        self.current_right_speed = 0
                        self.motor_controller.stop()

                current_time = time.time()
                if current_time - last_speed_update >= speed_update_interval:
                    # Calculate speeds based on active movements
                    self.current_left_speed = 0
                    self.current_right_speed = 0

                    # Apply forward/backward
                    if self.active_movements['w']:
                        self.current_left_speed = BASE_SPEED
                        self.current_right_speed = BASE_SPEED
                    elif self.active_movements['s']:
                        self.current_left_speed = -BASE_SPEED
                        self.current_right_speed = -BASE_SPEED

                    # Apply left/right turns
                    if self.active_movements['a']:
                        if not self.active_movements['w'] and not self.active_movements['s']:
                            # Pure left turn
                            self.current_left_speed = -TURN_SPEED
                            self.current_right_speed = TURN_SPEED
                        else:
                            # Modify left motor for turning while moving
                            self.current_left_speed = self.current_left_speed * 0.5 if self.current_left_speed != 0 else 0

                    if self.active_movements['d']:
                        if not self.active_movements['w'] and not self.active_movements['s']:
                            # Pure right turn
                            self.current_left_speed = TURN_SPEED
                            self.current_right_speed = -TURN_SPEED
                        else:
                            # Modify right motor for turning while moving
                            self.current_right_speed = self.current_right_speed * 0.5 if self.current_right_speed != 0 else 0

                    # Apply motor speeds
                    self.motor_controller.move(int(self.current_left_speed), int(self.current_right_speed))

                    # Record movement if recording
                    if is_recording:
                        elapsed = current_time - self.last_update_time
                        self.recorder.record_movement(int(self.current_left_speed), int(self.current_right_speed), elapsed)

                    self.last_update_time = current_time
                    last_speed_update = current_time

                    # Display status
                    sys.stdout.write("\r\033[K")
                    status = f"L: {int(self.current_left_speed):3d}, R: {int(self.current_right_speed):3d}"
                    active_keys = " Active: "
                    for k in ['w', 'a', 's', 'd']:
                        if self.active_movements[k]:
                            active_keys += k.upper() + " "
                    status += active_keys
                    if is_recording:
                        status += " [RECORDING]"
                    sys.stdout.write(status)
                    sys.stdout.flush()

                time.sleep(0.01)

        except Exception as e:
            print(f"\nError in interactive mode: {e}")

        finally:
            self.motor_controller.stop()

            if is_recording:
                self.recorder.stop_recording()

            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except:
                pass

# =============== MAIN FUNCTION ===============
def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    try:
        try:
            GPIO.cleanup()
        except:
            pass

        motor_controller = MotorControl()
        servo_controller = ServoControl()
        movement_recorder = MovementRecorder(motor_controller, servo_controller)
        interactive_recorder = InteractiveRecorder(motor_controller, servo_controller)

        while True:
            print("\n=== Movement Recorder Menu ===")
            print("1: Interactive recording (keyboard control)")
            print("2: List and play recordings")
            print("3: Export recording as Python function")
            print("q: Quit")

            choice = input("Enter choice: ")

            if choice == '1':
                interactive_recorder.start_interactive_session()

            elif choice == '2':
                recordings = movement_recorder.list_recordings()

                if recordings:
                    try:
                        selection = int(input("Enter the number of the recording to play (0 to go back): "))
                        if selection > 0 and selection <= len(recordings):
                            file_path = os.path.join(RECORDINGS_DIR, recordings[selection-1])

                            if movement_recorder.load_recording(file_path):
                                try:
                                    speed_factor = float(input("Enter playback speed factor (1.0 = normal speed): "))
                                    if speed_factor <= 0:
                                        speed_factor = 1.0
                                except ValueError:
                                    speed_factor = 1.0

                                movement_recorder.playback_recording(speed_factor)
                    except ValueError:
                        print("Invalid selection.")

            elif choice == '3':
                recordings = movement_recorder.list_recordings()

                if recordings:
                    try:
                        selection = int(input("Enter the number of the recording to export (0 to go back): "))
                        if selection > 0 and selection <= len(recordings):
                            file_path = os.path.join(RECORDINGS_DIR, recordings[selection-1])

                            output_file = input("Enter output file name (or press Enter for default): ")
                            output_file = output_file if output_file else None

                            movement_recorder.export_playback_function(file_path, output_file)
                    except ValueError:
                        print("Invalid selection.")

            elif choice.lower() == 'q':
                break

            else:
                print("Invalid choice!")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            motor_controller.cleanup()
            servo_controller.cleanup()
        except:
            pass

        GPIO.cleanup()
        print("Program exited")

if __name__ == "__main__":
    main()

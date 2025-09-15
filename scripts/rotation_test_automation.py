#!/usr/bin/env python3
"""
Rotation Test Automation Script
- Connects to transmitter serial port
- Runs gyro bias calibration
- Executes rotation tests from tests.txt
- Tracks cumulative rotation accounting for multiple full rotations
- Logs results to CSV file
"""

import serial
import time
import re
import csv
import threading
from datetime import datetime
import os

class RotationTestAutomation:
    def __init__(self, port="COM6", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.serial_thread = None
        
        # Telemetry tracking
        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.cumulative_rotation = 0.0
        self.telemetry_received = False
        self.last_telemetry_time = 0
        
        # Test results
        self.test_results = []
        
        # Tests from tests.txt
        self.rotation_tests = [
            ("T 0.15 1000", 0.15, 1000),
            ("T 0.15 2000", 0.15, 2000),
            ("T 0.15 3000", 0.15, 3000),
            ("T 0.2 500", 0.2, 500),
            ("T 0.2 1000", 0.2, 1000),
            ("T 0.2 1500", 0.2, 1500),
            ("T 0.3 500", 0.3, 500),
            ("T 0.3 1000", 0.3, 1000),
            ("T 0.3 1500", 0.3, 1500),
            ("T 0.4 500", 0.4, 500),
            ("T 0.4 1000", 0.4, 1000),
            ("T 0.4 1500", 0.4, 1500),
        ]
        
    def connect(self):
        """Connect to the transmitter serial port"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
            
            print(f"Connected to {self.port}")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        print("Disconnected")
        
    def read_serial(self):
        """Read serial data in background thread"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.parse_telemetry(line)
                        # print(f"RX: {line}")
                time.sleep(0.01)
            except Exception as e:
                print(f"Serial read error: {e}")
                break
                
    def parse_telemetry(self, line):
        """Parse telemetry messages to extract yaw angle"""
        # Parse: [TEL] rxSeq=X t=Xms yaw=X.XX° gz=X.XX dps uL=X.XX uR=X.XX
        tel_pattern = r'\[TEL\].*?yaw=([-\d\.]+)°'
        match = re.search(tel_pattern, line)
        
        if match:
            new_yaw = float(match.group(1))
            self.update_cumulative_rotation(new_yaw)
            self.telemetry_received = True
            self.last_telemetry_time = time.time()
            
    def update_cumulative_rotation(self, new_yaw):
        """Update cumulative rotation tracking multiple full rotations"""
        if hasattr(self, 'last_yaw'):
            # Calculate the difference, handling wraparound at ±180°
            diff = new_yaw - self.last_yaw
            
            # Handle wraparound cases
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
                
            # Add to cumulative rotation
            self.cumulative_rotation += diff
            
        self.last_yaw = new_yaw
        self.current_yaw = new_yaw
        
    def send_command(self, command):
        """Send command to transmitter"""
        if self.serial_port and self.running:
            try:
                self.serial_port.write((command + '\n').encode())
                print(f"TX: {command}")
                return True
            except Exception as e:
                print(f"Send error: {e}")
                return False
        return False
        
    def wait_for_telemetry(self, timeout=10):
        """Wait for telemetry data to be received"""
        start_time = time.time()
        self.telemetry_received = False
        
        while not self.telemetry_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            
        return self.telemetry_received
        
    def wait_for_stable_telemetry(self, duration=1.0, tolerance=1.0):
        """Wait for telemetry to stabilize (yaw not changing much)"""
        print(f"Waiting for telemetry to stabilize...")
        start_time = time.time()
        last_check_time = time.time()
        last_yaw = self.current_yaw
        
        while (time.time() - start_time) < 10:  # Max 10 second timeout
            time.sleep(0.1)
            
            # Check every 'duration' seconds
            if (time.time() - last_check_time) >= duration:
                yaw_change = abs(self.current_yaw - last_yaw)
                print(f"  Yaw change in last {duration}s: {yaw_change:.2f}°")
                
                if yaw_change < tolerance:
                    print(f"  Telemetry stabilized (change < {tolerance}°)")
                    return True
                    
                last_yaw = self.current_yaw
                last_check_time = time.time()
                
        print("  Warning: Telemetry did not stabilize within timeout")
        return False
        
    def reset_cumulative_rotation(self):
        """Reset cumulative rotation counter"""
        self.cumulative_rotation = 0.0
        self.last_yaw = self.current_yaw
        print(f"Reset cumulative rotation. Current yaw: {self.current_yaw:.2f}°")
        
    def run_calibration(self):
        """Run gyro bias calibration and wait for completion"""
        print("\n" + "="*50)
        print("STARTING GYRO BIAS CALIBRATION")
        print("="*50)
        
        if not self.send_command("C"):
            return False
            
        print("Waiting 5 seconds for calibration to complete...")
        time.sleep(5)
        
        # Wait for telemetry to stabilize after calibration
        self.wait_for_stable_telemetry(duration=1.0, tolerance=0.5)
        
        print("Calibration complete\n")
        return True
        
    def run_single_test(self, test_num, command, power, duration_ms):
        """Run a single rotation test"""
        print(f"\n--- Test {test_num}: {command} ---")
        
        # Wait for stable telemetry before starting
        if not self.wait_for_stable_telemetry(duration=0.5, tolerance=1.0):
            print("Warning: Starting test with unstable telemetry")
            
        # Reset cumulative rotation counter
        start_yaw = self.current_yaw
        self.reset_cumulative_rotation()
        
        # Send the rotation command
        if not self.send_command(command):
            print(f"Failed to send command: {command}")
            return None
            
        # Wait for command duration plus some extra time for settling
        wait_time = (duration_ms / 1000.0) + 2.0
        print(f"Waiting {wait_time:.1f}s for rotation to complete...")
        time.sleep(wait_time)
        
        # Wait for telemetry to stabilize
        self.wait_for_stable_telemetry(duration=1.0, tolerance=0.5)
        
        end_yaw = self.current_yaw
        total_rotation = self.cumulative_rotation
        
        # Calculate theoretical rotation (for comparison)
        # Note: This is just duration * power, actual calculation depends on robot dynamics
        theoretical_rotation = duration_ms * power * 0.1  # Placeholder formula
        
        result = {
            'test_num': test_num,
            'command': command,
            'power': power,
            'duration_ms': duration_ms,
            'start_yaw': start_yaw,
            'end_yaw': end_yaw,
            'total_rotation': total_rotation,
            'theoretical_rotation': theoretical_rotation,
            'timestamp': datetime.now().isoformat()
        }
        
        print(f"  Start yaw: {start_yaw:.2f}°")
        print(f"  End yaw: {end_yaw:.2f}°")
        print(f"  Total rotation: {total_rotation:.2f}°")
        print(f"  Full rotations: {total_rotation/360:.2f}")
        
        self.test_results.append(result)
        return result
        
    def run_all_tests(self):
        """Run all rotation tests"""
        print("\n" + "="*60)
        print("STARTING ROTATION TESTS")
        print("="*60)
        
        test_counter = 1
        for i, (command, power, duration_ms) in enumerate(self.rotation_tests, 1):
            print(f"\n{'='*40}")
            print(f"TEST SET {i}: {command}")
            print(f"{'='*40}")
            
            # Run each test 3 times
            set_results = []
            for run in range(1, 4):
                try:
                    print(f"\n--- Run {run}/3 ---")
                    result = self.run_single_test(test_counter, command, power, duration_ms)
                    if result:
                        set_results.append(result)
                        test_counter += 1
                        # Wait between runs
                        if run < 3:  # Don't wait after the last run
                            print(f"Waiting 2 seconds before next run...")
                            time.sleep(2)
                    else:
                        print(f"Run {run} failed, continuing...")
                        
                except KeyboardInterrupt:
                    print("\nTests interrupted by user")
                    return
                except Exception as e:
                    print(f"Error in run {run}: {e}")
                    continue
            
            # Calculate and display statistics for this test set
            if set_results:
                rotations = [r['total_rotation'] for r in set_results]
                avg_rotation = sum(rotations) / len(rotations)
                min_rotation = min(rotations)
                max_rotation = max(rotations)
                std_dev = (sum((x - avg_rotation)**2 for x in rotations) / len(rotations))**0.5
                
                print(f"\n--- Test Set {i} Summary ---")
                print(f"Command: {command}")
                print(f"Runs completed: {len(set_results)}/3")
                print(f"Average rotation: {avg_rotation:.1f}°")
                print(f"Min rotation: {min_rotation:.1f}°")
                print(f"Max rotation: {max_rotation:.1f}°")
                print(f"Std deviation: {std_dev:.1f}°")
                print(f"Average turns: {avg_rotation/360:.2f}")
            
            # Wait between test sets
            if i < len(self.rotation_tests):
                print(f"\nWaiting 5 seconds before next test set...")
                time.sleep(5)
                
        print("\n" + "="*60)
        print("ALL TESTS COMPLETED")
        print("="*60)
        
    def save_results(self, filename=None):
        """Save test results to CSV file"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"rotation_test_results_{timestamp}.csv"
        
        # Ensure results are written to the scripts/results directory
        base_dir = os.path.dirname(os.path.abspath(__file__))
        results_dir = os.path.join(base_dir, "results")
        os.makedirs(results_dir, exist_ok=True)
        out_path = filename
        if not os.path.isabs(out_path):
            out_path = os.path.join(results_dir, filename)
            
        try:
            with open(out_path, 'w', newline='') as csvfile:
                if self.test_results:
                    fieldnames = self.test_results[0].keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.test_results)
                    
            print(f"\nResults saved to: {out_path}")
            
            # Print summary
            print("\n" + "="*60)
            print("TEST RESULTS SUMMARY")
            print("="*60)
            
            # Group results by command for statistics
            command_groups = {}
            for result in self.test_results:
                cmd = result['command']
                if cmd not in command_groups:
                    command_groups[cmd] = []
                command_groups[cmd].append(result['total_rotation'])
            
            # Print statistics for each command
            for cmd, rotations in command_groups.items():
                if len(rotations) > 1:
                    avg = sum(rotations) / len(rotations)
                    min_rot = min(rotations)
                    max_rot = max(rotations)
                    std_dev = (sum((x - avg)**2 for x in rotations) / len(rotations))**0.5
                    print(f"{cmd:12} -> Avg: {avg:6.1f}° ({avg/360:+.2f} turns) | Min: {min_rot:6.1f}° | Max: {max_rot:6.1f}° | StdDev: {std_dev:4.1f}°")
                else:
                    rotation = rotations[0]
                    print(f"{cmd:12} -> {rotation:6.1f}° ({rotation/360:+.2f} turns)")
            
            # Also print individual results
            print(f"\nIndividual Results:")
            for result in self.test_results:
                rotation_turns = result['total_rotation'] / 360
                print(f"Test {result['test_num']:2d}: {result['command']} -> {result['total_rotation']:7.1f}° ({rotation_turns:+.2f} turns)")
                
        except Exception as e:
            print(f"Error saving results: {e}")
            
    def run_automation(self):
        """Run the complete automation sequence"""
        print("Rotation Test Automation Started")
        print(f"Port: {self.port}")
        print(f"Number of tests: {len(self.rotation_tests)}")
        
        # Connect
        if not self.connect():
            return False
            
        try:
            # Wait for initial telemetry
            print("Waiting for telemetry...")
            if not self.wait_for_telemetry(timeout=10):
                print("No telemetry received, continuing anyway...")
                
            # Run calibration
            if not self.run_calibration():
                print("Calibration failed")
                return False
                
            # Run all tests
            self.run_all_tests()
            
            # Save results
            self.save_results()
            
        except KeyboardInterrupt:
            print("\nAutomation interrupted by user")
        except Exception as e:
            print(f"Automation error: {e}")
        finally:
            self.disconnect()
            
        return True

def main():
    # Configuration
    TRANSMITTER_PORT = "COM6"  # Adjust this to your transmitter port
    
    print("Rotation Test Automation")
    print("=" * 40)
        
    # Run automation
    automation = RotationTestAutomation(port=TRANSMITTER_PORT)
    automation.run_automation()

if __name__ == "__main__":
    main()

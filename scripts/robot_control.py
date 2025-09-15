#!/usr/bin/env python3
"""
Robot Control GUI
- Reads telemetry data from transmitter s        # Quick action buttons
        quick_frame = ttk.Frame(cmd_frame)
        quick_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(quick_frame, text="Stop", command=lambda: self.send_command("S")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Forward", command=lambda: self.send_command("F 0.3 500")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Backward", command=lambda: self.send_command("F -0.3 500")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Left", command=lambda: self.send_command("T 0.4 300")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Right", command=lambda: self.send_command("T -0.4 300")).pack(side=tk.LEFT, padx=2)
        
        # Second row for calibration commands
        cal_frame = ttk.Frame(cmd_frame)
        cal_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(cal_frame, text="Reset Yaw", command=lambda: self.send_command("Y")).pack(side=tk.LEFT, padx=2)
        ttk.Button(cal_frame, text="Recal Gyro", command=lambda: self.send_command("C")).pack(side=tk.LEFT, padx=2)t
- Shows robot orientation with compass-like display
- Provides command interface to send motor commands
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import threading
import time
import math
import re
from collections import deque

class RobotControlGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Control Dashboard")
        self.root.geometry("800x600")
        
        # Serial connection
        self.serial_port = None
        self.serial_thread = None
        self.running = False
        
        # Data storage
        self.yaw_angle = 0.0
        self.gz_rate = 0.0
        self.uL_applied = 0.0
        self.uR_applied = 0.0
        self.telemetry_history = deque(maxlen=100)
        
        self.setup_gui()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side - Compass and telemetry
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Right side - Controls
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Serial connection controls
        conn_frame = ttk.LabelFrame(left_frame, text="Connection")
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(conn_frame, text="COM Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="COM6")  # Default to transmitter port
        port_entry = ttk.Entry(conn_frame, textvariable=self.port_var, width=10)
        port_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # Compass display
        compass_frame = ttk.LabelFrame(left_frame, text="Robot Orientation")
        compass_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.canvas = tk.Canvas(compass_frame, width=300, height=300, bg='white')
        self.canvas.pack(padx=10, pady=10)
        
        # Telemetry display
        tel_frame = ttk.LabelFrame(left_frame, text="Telemetry")
        tel_frame.pack(fill=tk.X)
        
        self.yaw_label = ttk.Label(tel_frame, text="Yaw: 0.0°")
        self.yaw_label.pack(anchor=tk.W)
        
        self.gz_label = ttk.Label(tel_frame, text="Rotation Rate: 0.0 dps")
        self.gz_label.pack(anchor=tk.W)
        
        self.motor_label = ttk.Label(tel_frame, text="Motors: L=0.0, R=0.0")
        self.motor_label.pack(anchor=tk.W)
        
        # Command interface
        cmd_frame = ttk.LabelFrame(right_frame, text="Robot Commands")
        cmd_frame.pack(fill=tk.BOTH, expand=True)
        
        # Quick command buttons
        quick_frame = ttk.Frame(cmd_frame)
        quick_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(quick_frame, text="Stop", command=lambda: self.send_command("S")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Forward", command=lambda: self.send_command("F 0.3 500")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Backward", command=lambda: self.send_command("F -0.3 500")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Left", command=lambda: self.send_command("T 0.4 300")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="Right", command=lambda: self.send_command("T -0.4 300")).pack(side=tk.LEFT, padx=2)
        
        # Custom command entry
        entry_frame = ttk.Frame(cmd_frame)
        entry_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(entry_frame, text="Custom Command:").pack(anchor=tk.W)
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(entry_frame, textvariable=self.cmd_var, width=30)
        cmd_entry.pack(fill=tk.X, pady=2)
        cmd_entry.bind('<Return>', lambda e: self.send_custom_command())
        
        ttk.Button(entry_frame, text="Send", command=self.send_custom_command).pack(pady=2)
        
        # Command examples
        examples_frame = ttk.LabelFrame(cmd_frame, text="Command Examples")
        examples_frame.pack(fill=tk.X, pady=5)
        
        examples = [
            "S - Stop",
            "F 0.4 200 - Forward 40% for 200ms",
            "F -0.4 200 - Backward 40% for 200ms",
            "T 0.5 300 - Turn left 50% for 300ms", 
            "T -0.5 300 - Turn right 50% for 300ms",
            "A 0.3 0.8 500 - Left 30%, Right 80% for 500ms",
            "Y - Reset yaw angle to 0 degrees",
            "C - Recalibrate gyro bias"
        ]
        
        for example in examples:
            ttk.Label(examples_frame, text=example, font=('Courier', 8)).pack(anchor=tk.W)
        
        # Console output
        console_frame = ttk.LabelFrame(cmd_frame, text="Console")
        console_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, height=10, width=40)
        self.console.pack(fill=tk.BOTH, expand=True)
        
        # Start drawing compass
        self.draw_compass()
        
        # Update display periodically
        self.update_display()
        
    def toggle_connection(self):
        if self.running:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        try:
            port = self.port_var.get()
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            self.running = True
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_thread.start()
            
            self.status_label.config(text="Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log_console(f"Connected to {port}")
            
        except Exception as e:
            self.log_console(f"Connection failed: {e}")
            
    def disconnect(self):
        self.running = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log_console("Disconnected")
        
    def read_serial(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.parse_telemetry(line)
                        self.log_console(line)
                time.sleep(0.01)
            except Exception as e:
                self.log_console(f"Serial read error: {e}")
                break
                
    def parse_telemetry(self, line):
        # Parse telemetry messages: [TEL] rxSeq=X t=Xms yaw=X.XX° gz=X.XX dps uL=X.XX uR=X.XX
        tel_pattern = r'\[TEL\].*?yaw=([-\d\.]+)°.*?gz=([-\d\.]+) dps.*?uL=([-\d\.]+).*?uR=([-\d\.]+)'
        match = re.search(tel_pattern, line)
        
        if match:
            self.yaw_angle = float(match.group(1))
            self.gz_rate = float(match.group(2))
            self.uL_applied = float(match.group(3))
            self.uR_applied = float(match.group(4))
            
            # Store in history
            self.telemetry_history.append({
                'time': time.time(),
                'yaw': self.yaw_angle,
                'gz': self.gz_rate,
                'uL': self.uL_applied,
                'uR': self.uR_applied
            })
            
    def send_command(self, command):
        if self.serial_port and self.running:
            try:
                self.serial_port.write((command + '\n').encode())
                self.log_console(f"Sent: {command}")
            except Exception as e:
                self.log_console(f"Send error: {e}")
                
    def send_custom_command(self):
        command = self.cmd_var.get().strip()
        if command:
            self.send_command(command)
            self.cmd_var.set("")
            
    def log_console(self, message):
        def update():
            self.console.insert(tk.END, f"{time.strftime('%H:%M:%S')} {message}\n")
            self.console.see(tk.END)
            # Keep only last 1000 lines
            lines = self.console.get('1.0', tk.END).split('\n')
            if len(lines) > 1000:
                self.console.delete('1.0', f'{len(lines)-1000}.0')
                
        self.root.after(0, update)
        
    def draw_compass(self):
        self.canvas.delete("all")
        
        # Canvas dimensions
        w, h = 300, 300
        cx, cy = w//2, h//2
        radius = 120
        
        # Draw compass circle
        self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, 
                               outline="black", width=2)
        
        # Draw cardinal directions
        directions = [("N", 0), ("E", 90), ("S", 180), ("W", 270)]
        for label, angle in directions:
            rad = math.radians(angle)
            x = cx + (radius + 20) * math.sin(rad)
            y = cy - (radius + 20) * math.cos(rad)
            self.canvas.create_text(x, y, text=label, font=('Arial', 14, 'bold'))
            
        # Draw degree marks
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x1 = cx + (radius - 10) * math.sin(rad)
            y1 = cy - (radius - 10) * math.cos(rad)
            x2 = cx + radius * math.sin(rad)
            y2 = cy - radius * math.cos(rad)
            self.canvas.create_line(x1, y1, x2, y2, width=2)
            
        # Draw robot orientation arrow
        yaw_rad = math.radians(self.yaw_angle)
        arrow_length = radius - 30
        
        # Arrow tip
        tip_x = cx + arrow_length * math.sin(yaw_rad)
        tip_y = cy - arrow_length * math.cos(yaw_rad)
        
        # Arrow base (opposite direction, shorter)
        base_x = cx - (arrow_length * 0.3) * math.sin(yaw_rad)
        base_y = cy + (arrow_length * 0.3) * math.cos(yaw_rad)
        
        # Draw arrow
        self.canvas.create_line(base_x, base_y, tip_x, tip_y, 
                               fill="red", width=4, arrow=tk.LAST, arrowshape=(16, 20, 6))
        
        # Draw center dot
        self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="red", outline="black")
        
        # Add rotation rate indicator
        if abs(self.gz_rate) > 5:  # Show rotation if significant
            rotation_color = "green" if self.gz_rate > 0 else "blue"
            arc_extent = min(abs(self.gz_rate) * 2, 180)  # Scale rotation rate to arc
            start_angle = self.yaw_angle - arc_extent/2
            
            self.canvas.create_arc(cx-radius+40, cy-radius+40, cx+radius-40, cy+radius-40,
                                 start=start_angle, extent=arc_extent, 
                                 outline=rotation_color, width=3, style='arc')
        
    def update_display(self):
        # Update telemetry labels
        self.yaw_label.config(text=f"Yaw: {self.yaw_angle:.1f}°")
        self.gz_label.config(text=f"Rotation Rate: {self.gz_rate:.1f} dps")
        self.motor_label.config(text=f"Motors: L={self.uL_applied:.2f}, R={self.uR_applied:.2f}")
        
        # Redraw compass
        self.draw_compass()
        
        # Schedule next update
        self.root.after(50, self.update_display)  # 20 Hz update rate
        
    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.disconnect()

if __name__ == "__main__":
    app = RobotControlGUI()
    app.run()

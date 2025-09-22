#!/usr/bin/env python3
"""
Robot Control GUI
- Reads telemetry data from transmitter
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
        self.sensor_left = 0.0
        self.sensor_center = 0.0
        self.sensor_right = 0.0
        self.nn_left = 0.0
        self.nn_right = 0.0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
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
        self.port_var = tk.StringVar(value="COM6")
        port_entry = ttk.Entry(conn_frame, textvariable=self.port_var, width=10)
        port_entry.pack(side=tk.LEFT, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)

        # Compass + Accel container
        compass_frame = ttk.LabelFrame(left_frame, text="Robot Orientation")
        compass_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # Compass display
        self.canvas = tk.Canvas(compass_frame, width=300, height=300, bg='white')
        self.canvas.pack(padx=10, pady=(10, 5))

        # Acceleration X/Y display
        self.accel_canvas = tk.Canvas(compass_frame, width=300, height=300, bg='white')
        self.accel_canvas.pack(padx=10, pady=(0, 10))

        # Telemetry display
        tel_frame = ttk.LabelFrame(left_frame, text="Telemetry")
        tel_frame.pack(fill=tk.X)

        fixed_chars = 60
        wrap_pixels = 420

        self.yaw_label = ttk.Label(tel_frame, text="Yaw: 0.0°", width=fixed_chars, anchor='w')
        self.yaw_label.pack(anchor=tk.W)

        self.gz_label = ttk.Label(tel_frame, text="Rotation Rate: 0.0 dps", width=fixed_chars, anchor='w')
        self.gz_label.pack(anchor=tk.W)

        self.motor_label = ttk.Label(tel_frame, text="Motors: L=0.0, R=0.0", width=fixed_chars, anchor='w')
        self.motor_label.pack(anchor=tk.W)

        self.imu_label = ttk.Label(
            tel_frame,
            text="IMU - Accel: X=0, Y=0, Z=0 | Gyro: X=0, Y=0, Z=0",
            width=fixed_chars,
            anchor='w',
            wraplength=wrap_pixels
        )
        self.imu_label.pack(anchor=tk.W)

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

        # Console output (wrap words so long lines don't push width)
        console_frame = ttk.LabelFrame(cmd_frame, text="Console")
        console_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.console = scrolledtext.ScrolledText(console_frame, height=10, width=40, wrap='word')
        self.console.pack(fill=tk.BOTH, expand=True)

        # Initial drawings
        self.draw_compass()
        self.draw_accel_display()

        # Lock minimum window size to current (prevents shake)
        self.root.update_idletasks()
        self.root.minsize(self.root.winfo_width(), self.root.winfo_height())

        # Periodic updates
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
        # Example: [TEL] rxSeq=1 t=1234ms sl=0.500 sc=0.300 sr=0.800 nl=0.450 nr=0.550 yaw=19.65° ax=296 ay=420 az=17460 gx=-32 gy=-6 gz=32
        tel_pattern = r'\[TEL\].*?sl=([-\d\.]+).*?sc=([-\d\.]+).*?sr=([-\d\.]+).*?nl=([-\d\.]+).*?nr=([-\d\.]+).*?yaw=([-\d\.]+)°.*?ax=(-?\d+).*?ay=(-?\d+).*?az=(-?\d+).*?gx=(-?\d+).*?gy=(-?\d+).*?gz=(-?\d+)'
        match = re.search(tel_pattern, line)
        if match:
            self.sensor_left = float(match.group(1))
            self.sensor_center = float(match.group(2))
            self.sensor_right = float(match.group(3))
            self.nn_left = float(match.group(4))
            self.nn_right = float(match.group(5))
            self.yaw_angle = float(match.group(6))
            self.accel_x = int(match.group(7))
            self.accel_y = int(match.group(8))
            self.accel_z = int(match.group(9))
            self.gyro_x = int(match.group(10))
            self.gyro_y = int(match.group(11))
            self.gyro_z = int(match.group(12))

            self.telemetry_history.append({
                'time': time.time(),
                'sensor_left': self.sensor_left,
                'sensor_center': self.sensor_center,
                'sensor_right': self.sensor_right,
                'nn_left': self.nn_left,
                'nn_right': self.nn_right,
                'yaw': self.yaw_angle,
                'ax': self.accel_x,
                'ay': self.accel_y,
                'az': self.accel_z,
                'gx': self.gyro_x,
                'gy': self.gyro_y,
                'gz_raw': self.gyro_z
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
            lines = self.console.get('1.0', tk.END).split('\n')
            if len(lines) > 1000:
                self.console.delete('1.0', f'{len(lines)-1000}.0')
        self.root.after(0, update)

    def draw_compass(self):
        self.canvas.delete("all")
        w, h = 300, 300
        cx, cy = w//2, h//2
        radius = 120

        self.canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="black", width=2)
        directions = [("N", 0), ("E", 90), ("S", 180), ("W", 270)]
        for label, angle in directions:
            rad = math.radians(angle)
            x = cx + (radius + 20) * math.sin(rad)
            y = cy - (radius + 20) * math.cos(rad)
            self.canvas.create_text(x, y, text=label, font=('Arial', 14, 'bold'))

        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x1 = cx + (radius - 10) * math.sin(rad)
            y1 = cy - (radius - 10) * math.cos(rad)
            x2 = cx + radius * math.sin(rad)
            y2 = cy - radius * math.cos(rad)
            self.canvas.create_line(x1, y1, x2, y2, width=2)

        yaw_rad = math.radians(self.yaw_angle)
        arrow_length = radius - 30
        tip_x = cx + arrow_length * math.sin(yaw_rad)
        tip_y = cy - arrow_length * math.cos(yaw_rad)
        base_x = cx - (arrow_length * 0.3) * math.sin(yaw_rad)
        base_y = cy + (arrow_length * 0.3) * math.cos(yaw_rad)
        self.canvas.create_line(base_x, base_y, tip_x, tip_y, fill="red", width=4,
                                arrow=tk.LAST, arrowshape=(16, 20, 6))
        self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="red", outline="black")

        # Removed rotation rate arc display

    def draw_accel_display(self):
        c = self.accel_canvas
        c.delete("all")

        w, h = 300, 300
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 30   # circle radius

        # Title
        c.create_text(w//2, 16, text="Acceleration (X/Y)", font=("Arial", 12, "bold"))

        # Axes
        c.create_line(20, cy, w-20, cy, width=1)   # X-axis
        c.create_line(cx, 20, cx, h-20, width=1)   # Y-axis

        # Bounding circle to show range
        c.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline="black")

        # Use ±25k as range
        tick_range = 25000

        # Ticks at 0, ±12.5k, ±25k
        for v in (-25000, -12500, 0, 12500, 25000):
            # x ticks
            x = cx + (v / tick_range) * radius
            c.create_line(x, cy-4, x, cy+4)
            c.create_text(x, cy+14, text=str(v), font=("Arial", 8))
            # y ticks
            y = cy - (v / tick_range) * radius
            c.create_line(cx-4, y, cx+4, y)
            if v != 0:
                c.create_text(cx-20, y, text=str(v), font=("Arial", 8))

        # Current vector (ax, ay)
        ax, ay = self.accel_x, self.accel_y
        ax = max(-tick_range, min(tick_range, ax))
        ay = max(-tick_range, min(tick_range, ay))

        px = cx + (ax / tick_range) * radius
        py = cy - (ay / tick_range) * radius

        c.create_line(cx, cy, px, py, width=3, fill="red",
                    arrow=tk.LAST, arrowshape=(12, 16, 6))
        c.create_oval(px-5, py-5, px+5, py+5, fill="red", outline="black")

        # Numeric readout
        c.create_text(w//2, h-16,
                    text=f"ax={self.accel_x}   ay={self.accel_y}",
                    font=("Courier", 11))

        # Axis labels
        c.create_text(w-20, cy-12, text="X", font=("Arial", 10))
        c.create_text(cx-12, 28, text="Y", font=("Arial", 10))


    def update_display(self):
        # Update telemetry labels
        self.yaw_label.config(text=f"Yaw: {self.yaw_angle:.1f}°")
        self.gz_label.config(text=f"Gyro Z (raw): {self.gyro_z}")
        self.motor_label.config(text=f"Sensors: L={self.sensor_left:.3f}, C={self.sensor_center:.3f}, R={self.sensor_right:.3f}")
        self.imu_label.config(
            text=f"NN Output: L={self.nn_left:.3f}, R={self.nn_right:.3f} | "
                 f"IMU Accel: X={self.accel_x}, Y={self.accel_y}, Z={self.accel_z}"
        )

        # Redraw visuals
        self.draw_compass()
        self.draw_accel_display()

        # Schedule next update
        self.root.after(50, self.update_display)  # 20 Hz

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.disconnect()

if __name__ == "__main__":
    app = RobotControlGUI()
    app.run()

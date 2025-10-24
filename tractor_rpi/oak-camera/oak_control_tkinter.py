#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import cv2
import depthai as dai
import threading
import time
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import json
import socket

# NEW: UDP client class for receiving GPS/RTK data
class UDPTelemetryClient:
    """Receive telemetry data via UDP from rtcm_server.py"""
    def __init__(self, port, callback):
        self.port = port
        self.callback = callback
        self.socket = None
        self.running = False
        
    def start(self):
        """Start the UDP client in a separate thread"""
        self.running = True
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()
        
    def _run(self):
        """Main loop for receiving UDP telemetry data"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("127.0.0.1", self.port))
            self.socket.settimeout(1.0)  # 1 second timeout for clean shutdown
            print(f"UDP telemetry client listening on port {self.port}")
            
            while self.running:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    try:
                        telemetry_data = json.loads(data.decode('utf-8'))
                        self.callback(telemetry_data)
                    except json.JSONDecodeError as e:
                        print(f"Error parsing UDP telemetry: {e}")
                except socket.timeout:
                    continue  # Normal timeout, continue loop
                    
        except Exception as e:
            print(f"UDP telemetry client error: {e}")
        finally:
            if self.socket:
                self.socket.close()
                    
    def stop(self):
        """Stop the UDP client"""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

# Global variables
current_speed_position = 4  # Neutral (1-10 range, integer)
current_steering = 0.0  # Steering from -1.0 (full left) to 1.0 (full right), float

# Telemetry variables
rtk_status = "Unknown"
battery_voltage = 0.0
actual_speed = 0.0
heading = 0.0

latest_frame = None
frame_lock = threading.Lock()
running = True
steering_slider = None  # Global reference to the slider
udp_telemetry_client = None  # Will be initialized later

# PCA9685 setup for servos
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # standard servo frequency

# Create servo objects for channels 0 (speed) and 1 (steering)
speed_servo = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)  # Channel 0 for speed
steering_servo = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)  # Channel 1 for steering

def create_pipeline():
    """Create simple DepthAI pipeline"""
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)  # Updated to RGB
    cam_rgb.setFps(30)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam_rgb.video.link(xout.input)

    return pipeline

def capture_frames():
    """Capture frames from Oak camera"""
    global latest_frame, running
    
    while running:
        try:
            print("Connecting to Oak camera...")
            with dai.Device(create_pipeline()) as device:
                print("Camera connected successfully")
                q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)

                while running:
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        
                        if frame is not None and frame.size > 0:
                            with frame_lock:
                                latest_frame = frame.copy()
                    
                    time.sleep(0.01)
                    
        except RuntimeError as e:
            print(f"Camera error: {e}")
            print("Retrying in 3 seconds...")
            time.sleep(3)

def get_speed_description(position):
    """Get descriptive text for speed position (1-10 scale)"""
    if position > 4:
        level = position - 4
        desc = f"Forward ({level})"
        if level == 6:
            desc += " Max"
        return desc
    elif position < 4:
        reverse_level = 4 - position
        desc = f"Reverse ({reverse_level})"
        if reverse_level == 3:
            desc += " Max"
        return desc
    else:
        return "NEUTRAL (STOP)"

def update_speed_servo(position):
    """Update speed servo (channel 0) based on position (1-10)"""
    if isinstance(position, str):
        position = int(position)  # Convert string to int if necessary
    if 1 <= position <= 10:
        if position <= 4:
            # Reverse: 1=0°, 4=90°
            angle = 90 + ((position - 4) * 30)  # Each reverse step: -30° from neutral
        else:
            # Forward: 5=120°, 10=180°
            angle = 90 + ((position - 4) * 15)  # Each forward step: +15° from neutral
        angle = max(0, min(180, angle))  # Clamp to 0-180°
        speed_servo.angle = angle
        print(f"Speed servo (ch0) set to angle: {angle}° for position {position}")

def update_steering_servo(steering_value):
    """Update steering servo (channel 1) based on steering (-1.0 to 1.0)"""
    if isinstance(steering_value, str):
        steering_value = float(steering_value)  # Convert string to float if necessary
    if -1.0 <= steering_value <= 1.0:
        # Map -1.0 (full left) to 0°, 0.0 (straight) to 90°, 1.0 (full right) to 180°
        angle = (steering_value + 1.0) * 90
        angle = max(0, min(180, angle))  # Clamp to 0-180°
        steering_servo.angle = angle
        print(f"Steering servo (ch1) set to angle: {angle}° for value {steering_value}")

def video_loop(video_label):
    """Update video in Tkinter label"""
    global latest_frame, running, current_speed_position, current_steering
    
    while running:
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # Resize for display to match window width (1200 pixels)
        frame_resized = cv2.resize(frame, (1200, 900))  # Match video width to 1200
        
        # Use frame_resized directly since it's RGB
        frame_rgb = frame_resized  # No conversion needed
        
        # Convert to PhotoImage
        img = tk.PhotoImage(data=cv2.imencode('.ppm', frame_rgb)[1].tobytes())
        video_label.config(image=img)
        video_label.image = img
        
        time.sleep(0.033)  # ~30fps

def set_speed(value):
    global current_speed_position
    current_speed_position = int(value)  # Ensure integer
    update_speed_servo(current_speed_position)

def set_steering(value):
    global current_steering
    current_steering = float(value)  # Ensure float
    update_steering_servo(current_steering)

def emergency_stop():
    global steering_slider
    set_speed(4)  # Neutral
    set_steering(0.0)  # Straight
    # Also update the slider to reflect the reset position
    if steering_slider:
        steering_slider.set(0.0)
    print("Emergency Stop Activated! Speed set to Neutral, Steering set to Straight.")

def update_steering_preset(value):
    """Update both slider and servo when preset is clicked"""
    global steering_slider
    if steering_slider:
        steering_slider.set(value)
    set_steering(value)

# NEW: UDP telemetry callback function
def handle_udp_telemetry(data):
    """Handle incoming UDP telemetry data from rtcm_server.py"""
    global rtk_status, heading
    
    # Update RTK status
    rtk_status = data.get("fix_quality", "Unknown")
    
    # Update heading (handle None values)
    heading_deg = data.get("heading_deg")
    if heading_deg is not None and data.get("headValid", False):
        heading = heading_deg
    else:
        heading = 0.0

# Create GUI
root = tk.Tk()
root.title("Robot Teleoperation Station (Local GUI)")
# Set fixed size to leave room for Ubuntu toolbars (e.g., 100px reserved)
root.geometry("1600x900")  # Adjusted to fit within 1920x1080, leaving ~100px for toolbars

# Configure grid layout for responsiveness
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=0)  # Fixed width for speed controls
root.rowconfigure(0, weight=1)  # Expand video and steering vertically

# Left frame for video and steering (below video)
left_frame = ttk.Frame(root, width=1200)  # Fixed width to match video
left_frame.grid(row=0, column=0, sticky="nsew", padx=(10, 0))  # Add 10px left padding
left_frame.rowconfigure(0, weight=1)  # Video expands
left_frame.rowconfigure(1, weight=0)  # Steering fixed
left_frame.columnconfigure(0, weight=1)

# Video display
video_label = tk.Label(left_frame)
video_label.grid(row=0, column=0, sticky="nsew")

# Steering control (under video) with width matching video (1200px)
steering_frame = ttk.Frame(left_frame)
steering_frame.grid(row=1, column=0, sticky="ew", pady=10)  # Remove padx, use "ew" for full width
steering_frame.columnconfigure(0, weight=1)

ttk.Label(steering_frame, text="Steering Control").grid(row=0, column=0, sticky="ew")

steering_presets = [
    ("Full Left", -1.0),
    ("1/2 Left", -0.5),
    ("1/4 Left", -0.25),
    ("Straight", 0.0),
    ("1/4 Right", 0.25),
    ("1/2 Right", 0.5),
    ("Full Right", 1.0),
]
preset_frame = ttk.Frame(steering_frame)
preset_frame.grid(row=1, column=0, sticky="ew", padx=5)  # Add small internal padding
for label, value in steering_presets:
    btn = ttk.Button(preset_frame, text=label, command=lambda val=value: update_steering_preset(val))
    btn.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

# Create the steering slider - this is the single, clean definition
steering_slider = ttk.Scale(steering_frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, value=0.0, command=set_steering)
steering_slider.grid(row=2, column=0, sticky="ew", pady=10)

steering_display = ttk.Label(steering_frame, text="Current: 0.00")
steering_display.grid(row=3, column=0, sticky="ew")

# Right frame for emergency stop and speed controls
right_frame = ttk.Frame(root)
right_frame.grid(row=0, column=1, sticky="ns", padx=10, pady=10)

# Emergency stop with red style, doubled size
style = ttk.Style()
style.configure("Red.TButton", background="red", foreground="white")
emergency_button = ttk.Button(right_frame, text="EMERGENCY STOP", command=emergency_stop, style="Red.TButton", width=20)
emergency_button.pack(pady=20, padx=10)  # Increased padding to double effective size

# Speed control
speed_frame = ttk.Frame(right_frame)
speed_frame.pack(pady=10, fill=tk.X)
ttk.Label(speed_frame, text="Speed Control (1-10)").pack(anchor=tk.W)

speed_var = tk.IntVar(value=4)
positions = [
    (1, "Reverse 3 (Max)"),
    (2, "Reverse 2"),
    (3, "Reverse 1"),
    (4, "NEUTRAL (STOP)"),
    (5, "Forward 1 (Min)"),
    (6, "Forward 2"),
    (7, "Forward 3"),
    (8, "Forward 4"),
    (9, "Forward 5"),
    (10, "Forward 6 (Max)"),
]
for pos, label in positions:
    rb = ttk.Radiobutton(speed_frame, text=label, variable=speed_var, value=pos, command=lambda p=pos: set_speed(p))
    rb.pack(anchor=tk.W)

# Telemetry section
telemetry_frame = ttk.Frame(right_frame)
telemetry_frame.pack(pady=10, fill=tk.X)
ttk.Label(telemetry_frame, text="Telemetry Data").pack(anchor=tk.W)

# Create telemetry display labels
rtk_status_label = ttk.Label(telemetry_frame, text="RTK Status: Unknown")
rtk_status_label.pack(anchor=tk.W)

battery_voltage_label = ttk.Label(telemetry_frame, text="Battery Voltage: 0.0V")
battery_voltage_label.pack(anchor=tk.W)

actual_speed_label = ttk.Label(telemetry_frame, text="Actual Speed: 0.0 m/s")
actual_speed_label.pack(anchor=tk.W)

heading_label = ttk.Label(telemetry_frame, text="Heading: 0.0°")
heading_label.pack(anchor=tk.W)

# Start video capture thread
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# Start video display thread
video_thread = threading.Thread(target=video_loop, args=(video_label,), daemon=True)
video_thread.start()

# NEW: Start UDP telemetry client (port 6002)
udp_telemetry_client = UDPTelemetryClient(6002, handle_udp_telemetry)
udp_telemetry_client.start()

# Update steering display loop
def update_display():
    global rtk_status, battery_voltage, actual_speed, heading
    steering_display.config(text=f"Current: {current_steering:.2f}")
    
    # Update telemetry displays
    rtk_status_label.config(text=f"RTK Status: {rtk_status}")
    battery_voltage_label.config(text=f"Battery Voltage: {battery_voltage:.1f}V")
    actual_speed_label.config(text=f"Actual Speed: {actual_speed:.1f} m/s")
    heading_label.config(text=f"Heading: {heading:.1f}°")
    
    root.after(100, update_display)

update_display()

def on_close():
    global running, udp_telemetry_client
    running = False
    if udp_telemetry_client:
        udp_telemetry_client.stop()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()

# Cleanup
pca.deinit()
#!/usr/bin/env python3
"""
Simple GCS with 3D Trajectory Plot and HUD display.

Features:
- 3D plot showing aircraft trajectory (X, Y, Z in NED frame)
- HUD showing attitude, airspeed, altitude, heading

Usage:
1. Start SITL with: sim_vehicle.py -v ArduPlane --out=udp:127.0.0.1:14551
2. Run this script: python3 mini_gcs.py

Requirements:
    pip install matplotlib numpy pymavlink
"""

import math
import time
import threading
from collections import deque
from pymavlink import mavutil

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Circle, Wedge, FancyArrow
import matplotlib.lines as mlines
import numpy as np

# Configuration
DATA_RATE_HZ = 50
MAX_TRAJECTORY_POINTS = 500  # Maximum points to keep in trajectory
UDP_PORT = 14551


class StateMonitor:
    def __init__(self):
        # Position
        self.lat = 0.0
        self.lon = 0.0
        self.alt_msl = 0.0
        self.alt_agl = 0.0
        
        # Attitude (degrees)
        self.phi = 0.0      # Roll
        self.theta = 0.0    # Pitch
        self.psi = 0.0      # Yaw/Heading
        
        # Angular rates (deg/s)
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        
        # Local NED position (meters)
        self.x_ned = 0.0
        self.y_ned = 0.0
        self.z_ned = 0.0
        
        # Velocities
        self.vx_ned = 0.0
        self.vy_ned = 0.0
        self.vz_ned = 0.0
        
        # Airspeed data
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.climb_rate = 0.0
        self.throttle = 0
        
        # Trajectory history
        self.trajectory_x = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self.trajectory_y = deque(maxlen=MAX_TRAJECTORY_POINTS)
        self.trajectory_z = deque(maxlen=MAX_TRAJECTORY_POINTS)
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Connection status
        self.connected = False
        self.last_heartbeat = 0

    def update_from_attitude(self, msg):
        with self.lock:
            self.phi = math.degrees(msg.roll)
            self.theta = math.degrees(msg.pitch)
            self.psi = math.degrees(msg.yaw)
            self.p = math.degrees(msg.rollspeed)
            self.q = math.degrees(msg.pitchspeed)
            self.r = math.degrees(msg.yawspeed)

    def update_from_global_position(self, msg):
        with self.lock:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt_msl = msg.alt / 1000.0
            self.alt_agl = msg.relative_alt / 1000.0

    def update_from_local_position(self, msg):
        with self.lock:
            self.x_ned = msg.x
            self.y_ned = msg.y
            self.z_ned = msg.z
            self.vx_ned = msg.vx
            self.vy_ned = msg.vy
            self.vz_ned = msg.vz
            
            # Add to trajectory
            self.trajectory_x.append(msg.x)
            self.trajectory_y.append(msg.y)
            self.trajectory_z.append(-msg.z)  # Convert Down to Up for display

    def update_from_vfr_hud(self, msg):
        with self.lock:
            self.airspeed = msg.airspeed
            self.groundspeed = msg.groundspeed
            self.climb_rate = msg.climb
            self.throttle = msg.throttle

    def get_state_snapshot(self):
        """Get a thread-safe copy of current state"""
        with self.lock:
            return {
                'phi': self.phi,
                'theta': self.theta,
                'psi': self.psi,
                'alt_msl': self.alt_msl,
                'alt_agl': self.alt_agl,
                'airspeed': self.airspeed,
                'groundspeed': self.groundspeed,
                'climb_rate': self.climb_rate,
                'throttle': self.throttle,
                'x_ned': self.x_ned,
                'y_ned': self.y_ned,
                'z_ned': self.z_ned,
                'trajectory_x': list(self.trajectory_x),
                'trajectory_y': list(self.trajectory_y),
                'trajectory_z': list(self.trajectory_z),
                'connected': self.connected,
            }


def request_data_stream(conn, stream_id, rate_hz):
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        stream_id,
        rate_hz,
        1
    )


def request_message_interval(conn, message_id, interval_us):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def mavlink_thread(state):
    """Thread to handle MAVLink communication"""
    print("Connecting to udpin:127.0.0.1:14551...")
    conn = mavutil.mavlink_connection(f'udpin:127.0.0.1:{UDP_PORT}')
    
    print("Waiting for heartbeat...")
    conn.wait_heartbeat()
    print(f"Connected! (system {conn.target_system}, component {conn.target_component})")
    
    state.connected = True
    state.last_heartbeat = time.time()
    
    # Request data streams
    interval_us = int(1000000 / DATA_RATE_HZ)
    request_data_stream(conn, mavutil.mavlink.MAV_DATA_STREAM_ALL, DATA_RATE_HZ)
    request_message_interval(conn, 30, interval_us)   # ATTITUDE
    request_message_interval(conn, 33, interval_us)   # GLOBAL_POSITION_INT
    request_message_interval(conn, 32, interval_us)   # LOCAL_POSITION_NED
    request_message_interval(conn, 74, interval_us)   # VFR_HUD
    
    last_stream_request = time.time()
    
    while True:
        # Re-request streams periodically
        now = time.time()
        if now - last_stream_request >= 2.0:
            request_data_stream(conn, mavutil.mavlink.MAV_DATA_STREAM_ALL, DATA_RATE_HZ)
            request_message_interval(conn, 30, interval_us)
            request_message_interval(conn, 33, interval_us)
            request_message_interval(conn, 32, interval_us)
            request_message_interval(conn, 74, interval_us)
            last_stream_request = now
        
        msg = conn.recv_match(blocking=True, timeout=0.1)
        
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'HEARTBEAT':
                state.last_heartbeat = time.time()
                state.connected = True
            elif msg_type == 'ATTITUDE':
                state.update_from_attitude(msg)
            elif msg_type == 'GLOBAL_POSITION_INT':
                state.update_from_global_position(msg)
            elif msg_type == 'LOCAL_POSITION_NED':
                state.update_from_local_position(msg)
            elif msg_type == 'VFR_HUD':
                state.update_from_vfr_hud(msg)
        
        # Check connection timeout
        if time.time() - state.last_heartbeat > 3.0:
            state.connected = False


def draw_attitude_indicator(ax, roll, pitch):
    """Draw a simple attitude indicator (artificial horizon)"""
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Clip pitch for display
    pitch_display = np.clip(pitch, -30, 30)
    pitch_offset = pitch_display / 30.0 * 0.5
    
    # Sky and ground
    # Create rotated horizon
    roll_rad = math.radians(roll)
    
    # Background circle
    circle = plt.Circle((0, 0), 0.9, color='#333333', fill=True)
    ax.add_patch(circle)
    
    # Sky (blue upper half, rotated)
    theta1 = 90 + roll
    theta2 = 270 + roll
    sky = Wedge((0, pitch_offset), 0.85, theta1, theta2, color='#4A90D9', fill=True)
    ax.add_patch(sky)
    
    # Ground (brown lower half, rotated)
    ground = Wedge((0, pitch_offset), 0.85, theta2, theta1, color='#8B6914', fill=True)
    ax.add_patch(ground)
    
    # Horizon line
    x1 = -0.85 * math.cos(roll_rad)
    y1 = -0.85 * math.sin(roll_rad) + pitch_offset
    x2 = 0.85 * math.cos(roll_rad)
    y2 = 0.85 * math.sin(roll_rad) + pitch_offset
    ax.plot([x1, x2], [y1, y2], 'w-', linewidth=2)
    
    # Aircraft symbol (fixed)
    ax.plot([-0.3, -0.1, 0, 0.1, 0.3], [0, 0, -0.1, 0, 0], 'y-', linewidth=3)
    ax.plot([0], [0], 'yo', markersize=8)
    
    # Roll indicator arc at top
    arc_angles = np.linspace(math.radians(120), math.radians(60), 50)
    arc_x = 0.75 * np.cos(arc_angles)
    arc_y = 0.75 * np.sin(arc_angles)
    ax.plot(arc_x, arc_y, 'w-', linewidth=2)
    
    # Roll pointer
    pointer_angle = math.radians(90 - roll)
    px = 0.7 * math.cos(pointer_angle)
    py = 0.7 * math.sin(pointer_angle)
    ax.plot([px], [py], 'v', color='yellow', markersize=10)
    
    # Pitch lines
    for p in [-20, -10, 10, 20]:
        if abs(pitch_display - p) < 25:
            py = (p - pitch_display) / 30.0 * 0.5
            line_len = 0.15 if p % 20 == 0 else 0.08
            x1 = -line_len * math.cos(roll_rad)
            y1 = py - line_len * math.sin(roll_rad)
            x2 = line_len * math.cos(roll_rad)
            y2 = py + line_len * math.sin(roll_rad)
            ax.plot([x1, x2], [y1, y2], 'w-', linewidth=1)
    
    ax.set_title(f'Roll: {roll:.1f}°  Pitch: {pitch:.1f}°', fontsize=10, color='white')


def draw_heading_indicator(ax, heading):
    """Draw a simple heading indicator (compass)"""
    ax.clear()
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Compass circle
    circle = plt.Circle((0, 0), 1.0, color='#333333', fill=True)
    ax.add_patch(circle)
    circle2 = plt.Circle((0, 0), 0.95, color='#1a1a1a', fill=True)
    ax.add_patch(circle2)
    
    # Heading in 0-360
    heading = heading % 360
    if heading < 0:
        heading += 360
    
    heading_rad = math.radians(heading)
    
    # Cardinal directions (rotated based on heading)
    directions = [('N', 0), ('E', 90), ('S', 180), ('W', 270)]
    for label, angle in directions:
        angle_rad = math.radians(angle - heading + 90)
        x = 0.75 * math.cos(angle_rad)
        y = 0.75 * math.sin(angle_rad)
        color = 'red' if label == 'N' else 'white'
        ax.text(x, y, label, ha='center', va='center', fontsize=12, 
                fontweight='bold', color=color)
    
    # Tick marks every 30 degrees
    for angle in range(0, 360, 30):
        angle_rad = math.radians(angle - heading + 90)
        x1 = 0.9 * math.cos(angle_rad)
        y1 = 0.9 * math.sin(angle_rad)
        x2 = 0.95 * math.cos(angle_rad)
        y2 = 0.95 * math.sin(angle_rad)
        ax.plot([x1, x2], [y1, y2], 'w-', linewidth=1)
    
    # Aircraft symbol pointing up
    ax.plot([0, 0], [0.1, 0.4], 'y-', linewidth=3)
    ax.plot([-0.15, 0, 0.15], [0, 0.1, 0], 'y-', linewidth=3)
    ax.plot([-0.08, 0, 0.08], [-0.15, -0.1, -0.15], 'y-', linewidth=2)
    
    # Heading pointer at top
    ax.plot([0], [1.05], 'v', color='orange', markersize=12)
    
    ax.set_title(f'HDG: {heading:.0f}°', fontsize=10, color='white')


def draw_speed_altitude_tape(ax, airspeed, altitude, climb_rate, throttle):
    """Draw speed and altitude indicators"""
    ax.clear()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.axis('off')
    ax.set_facecolor('#1a1a1a')
    
    # Speed tape (left side)
    ax.add_patch(FancyBboxPatch((0.5, 1), 2, 8, boxstyle="round,pad=0.1", 
                                 facecolor='#333333', edgecolor='white', linewidth=1))
    ax.text(1.5, 9.3, 'SPD', ha='center', va='center', fontsize=9, color='cyan', fontweight='bold')
    ax.text(1.5, 8.5, 'm/s', ha='center', va='center', fontsize=7, color='gray')
    
    # Airspeed value
    ax.text(1.5, 5, f'{airspeed:.1f}', ha='center', va='center', fontsize=16, 
            color='lime', fontweight='bold')
    
    # Ground speed
    ax.text(1.5, 3, 'GS', ha='center', va='center', fontsize=8, color='gray')
    ax.text(1.5, 2.2, f'{airspeed:.0f}', ha='center', va='center', fontsize=10, color='white')
    
    # Altitude tape (right side)
    ax.add_patch(FancyBboxPatch((7.5, 1), 2, 8, boxstyle="round,pad=0.1",
                                 facecolor='#333333', edgecolor='white', linewidth=1))
    ax.text(8.5, 9.3, 'ALT', ha='center', va='center', fontsize=9, color='cyan', fontweight='bold')
    ax.text(8.5, 8.5, 'm', ha='center', va='center', fontsize=7, color='gray')
    
    # Altitude value
    ax.text(8.5, 5, f'{altitude:.0f}', ha='center', va='center', fontsize=16,
            color='lime', fontweight='bold')
    
    # Climb rate
    climb_arrow = '↑' if climb_rate > 0.5 else ('↓' if climb_rate < -0.5 else '→')
    climb_color = 'lime' if climb_rate > 0 else ('red' if climb_rate < 0 else 'white')
    ax.text(8.5, 3, 'V/S', ha='center', va='center', fontsize=8, color='gray')
    ax.text(8.5, 2.2, f'{climb_arrow}{abs(climb_rate):.1f}', ha='center', va='center', 
            fontsize=10, color=climb_color)
    
    # Throttle bar (center bottom)
    ax.add_patch(FancyBboxPatch((4, 0.5), 2, 1.5, boxstyle="round,pad=0.05",
                                 facecolor='#333333', edgecolor='white', linewidth=1))
    ax.text(5, 1.7, 'THR', ha='center', va='center', fontsize=8, color='gray')
    
    # Throttle bar fill
    throttle_width = 1.8 * (throttle / 100.0)
    throttle_color = 'lime' if throttle < 80 else 'orange' if throttle < 95 else 'red'
    ax.add_patch(plt.Rectangle((4.1, 0.6), throttle_width, 0.6, color=throttle_color))
    ax.text(5, 0.9, f'{throttle}%', ha='center', va='center', fontsize=9, color='white')


def main():
    # Create state monitor
    state = StateMonitor()
    
    # Start MAVLink thread
    mav_thread = threading.Thread(target=mavlink_thread, args=(state,), daemon=True)
    mav_thread.start()
    
    # Wait for connection
    print("Waiting for MAVLink connection...")
    while not state.connected:
        time.sleep(0.1)
    print("Connected! Starting GCS display...")
    
    # Create figure with dark background
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(14, 8), facecolor='#1a1a1a')
    fig.canvas.manager.set_window_title('Mini GCS - ArduPilot SITL')
    
    # Create subplots
    # 3D trajectory plot (left, larger)
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d', facecolor='#1a1a1a')
    
    # HUD panel (right side, multiple subplots)
    ax_attitude = fig.add_subplot(2, 4, 3, facecolor='#1a1a1a')
    ax_heading = fig.add_subplot(2, 4, 4, facecolor='#1a1a1a')
    ax_tapes = fig.add_subplot(2, 2, 4, facecolor='#1a1a1a')
    
    plt.tight_layout()
    plt.ion()
    plt.show()
    
    # Main display loop
    try:
        while True:
            # Get current state
            s = state.get_state_snapshot()
            
            # Update 3D trajectory plot
            ax_3d.clear()
            ax_3d.set_facecolor('#1a1a1a')
            
            if len(s['trajectory_x']) > 1:
                # Plot trajectory
                ax_3d.plot(s['trajectory_x'], s['trajectory_y'], s['trajectory_z'],
                          'c-', linewidth=1.5, alpha=0.7, label='Trajectory')
                
                # Current position marker
                if len(s['trajectory_x']) > 0:
                    ax_3d.scatter([s['trajectory_x'][-1]], [s['trajectory_y'][-1]], 
                                 [s['trajectory_z'][-1]], c='lime', s=100, marker='o',
                                 label='Current')
                
                # Start position marker
                ax_3d.scatter([s['trajectory_x'][0]], [s['trajectory_y'][0]], 
                             [s['trajectory_z'][0]], c='red', s=80, marker='^',
                             label='Start')
            
            ax_3d.set_xlabel('X North (m)', color='white')
            ax_3d.set_ylabel('Y East (m)', color='white')
            ax_3d.set_zlabel('Altitude (m)', color='white')
            ax_3d.set_title('3D Trajectory (NED Frame)', color='white', fontsize=12)
            ax_3d.legend(loc='upper left', fontsize=8)
            
            # Set axis colors
            ax_3d.tick_params(colors='white')
            ax_3d.xaxis.pane.fill = False
            ax_3d.yaxis.pane.fill = False
            ax_3d.zaxis.pane.fill = False
            
            # Update HUD elements
            draw_attitude_indicator(ax_attitude, s['phi'], s['theta'])
            draw_heading_indicator(ax_heading, s['psi'])
            draw_speed_altitude_tape(ax_tapes, s['airspeed'], s['alt_agl'], 
                                    s['climb_rate'], s['throttle'])
            
            # Connection status
            status_color = 'lime' if s['connected'] else 'red'
            status_text = 'CONNECTED' if s['connected'] else 'DISCONNECTED'
            fig.suptitle(f'Mini GCS | Status: {status_text}', color=status_color, fontsize=11)
            
            plt.pause(0.05)  # ~20 Hz display update
            
    except KeyboardInterrupt:
        print("\nExiting...")
        plt.close('all')


if __name__ == "__main__":
    main()

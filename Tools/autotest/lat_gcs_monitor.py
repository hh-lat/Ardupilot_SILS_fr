#!/usr/bin/env python3
"""
Script to monitor SITL full state at ~50Hz in real-time.

States fetched:
- Position: lat, lon, alt_msl, alt_agl
- Attitude: phi (roll), theta (pitch), psi (yaw) in degrees
- Angular rates: p, q, r in deg/s
- Local NED position: x, y, z relative to home
- Airspeed: true airspeed, ground speed, climb rate

Usage:
1. Start SITL with: sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
   Or add output in MAVProxy: output add 127.0.0.1:14551
2. Run this script: python3 attitude_monitor.py
"""

import math
import time
import os
import socket
import struct
from pymavlink import mavutil

# Data rate in Hz
DATA_RATE_HZ = 50

# X-Plane Configuration
XPLANE_IP = "127.0.0.1"      # X-Plane IP address (localhost if on same machine)
XPLANE_PORT = 49000           # X-Plane default UDP port
XPLANE_ENABLED = True         # Set to False to disable X-Plane output
XPLANE_AIRCRAFT_INDEX = 0     # 0 = main aircraft in X-Plane


def rad_to_deg(rad):
    """Convert radians to degrees"""
    return rad * 180.0 / math.pi


class StateMonitor:
    def __init__(self):
        # Position (from GLOBAL_POSITION_INT)
        self.lat = 0.0          # degrees
        self.lon = 0.0          # degrees
        self.alt_msl = 0.0      # meters (above mean sea level)
        self.alt_agl = 0.0      # meters (above ground level / relative alt)
        
        # Attitude angles (from ATTITUDE)
        self.phi = 0.0          # roll in degrees
        self.theta = 0.0        # pitch in degrees
        self.psi = 0.0          # yaw in degrees
        
        # Angular rates (from ATTITUDE)
        self.p = 0.0            # roll rate in deg/s
        self.q = 0.0            # pitch rate in deg/s
        self.r = 0.0            # yaw rate in deg/s
        
        # Local NED position relative to home (from LOCAL_POSITION_NED)
        self.x_ned = 0.0        # North in meters
        self.y_ned = 0.0        # East in meters
        self.z_ned = 0.0        # Down in meters (negative = up)
        
        # Velocities NED (from LOCAL_POSITION_NED)
        self.vx_ned = 0.0       # m/s
        self.vy_ned = 0.0       # m/s
        self.vz_ned = 0.0       # m/s
        
        # Airspeed data (from VFR_HUD)
        self.airspeed = 0.0     # True airspeed in m/s
        self.groundspeed = 0.0  # Ground speed in m/s
        self.climb_rate = 0.0   # Climb rate in m/s
        self.throttle = 0       # Throttle percentage (0-100)
        
        # Timestamps for rate calculation
        self.last_attitude_time = 0
        self.last_position_time = 0
        self.last_local_time = 0
        self.last_vfr_hud_time = 0
        self.attitude_rate = 0.0
        self.position_rate = 0.0
        self.local_rate = 0.0
        self.vfr_hud_rate = 0.0

    def update_from_attitude(self, msg):
        """Update from ATTITUDE message"""
        self.phi = rad_to_deg(msg.roll)
        self.theta = rad_to_deg(msg.pitch)
        self.psi = rad_to_deg(msg.yaw)
        self.p = rad_to_deg(msg.rollspeed)
        self.q = rad_to_deg(msg.pitchspeed)
        self.r = rad_to_deg(msg.yawspeed)
        
        # Calculate rate
        now = time.time()
        if self.last_attitude_time > 0:
            dt = now - self.last_attitude_time
            if dt > 0:
                self.attitude_rate = 1.0 / dt
        self.last_attitude_time = now

    def update_from_global_position(self, msg):
        """Update from GLOBAL_POSITION_INT message"""
        self.lat = msg.lat / 1e7          # Convert from degE7
        self.lon = msg.lon / 1e7          # Convert from degE7
        self.alt_msl = msg.alt / 1000.0   # Convert from mm to m
        self.alt_agl = msg.relative_alt / 1000.0  # Convert from mm to m
        
        # Calculate rate
        now = time.time()
        if self.last_position_time > 0:
            dt = now - self.last_position_time
            if dt > 0:
                self.position_rate = 1.0 / dt
        self.last_position_time = now

    def update_from_local_position(self, msg):
        """Update from LOCAL_POSITION_NED message"""
        self.x_ned = msg.x      # North
        self.y_ned = msg.y      # East
        self.z_ned = msg.z      # Down
        self.vx_ned = msg.vx
        self.vy_ned = msg.vy
        self.vz_ned = msg.vz
        
        # Calculate rate
        now = time.time()
        if self.last_local_time > 0:
            dt = now - self.last_local_time
            if dt > 0:
                self.local_rate = 1.0 / dt
        self.last_local_time = now

    def update_from_vfr_hud(self, msg):
        """Update from VFR_HUD message"""
        self.airspeed = msg.airspeed       # True airspeed m/s
        self.groundspeed = msg.groundspeed # Ground speed m/s
        self.climb_rate = msg.climb        # Climb rate m/s
        self.throttle = msg.throttle       # Throttle 0-100%
        
        # Calculate rate
        now = time.time()
        if self.last_vfr_hud_time > 0:
            dt = now - self.last_vfr_hud_time
            if dt > 0:
                self.vfr_hud_rate = 1.0 / dt
        self.last_vfr_hud_time = now


def request_data_stream(conn, stream_id, rate_hz):
    """Request a specific data stream at given rate"""
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        stream_id,
        rate_hz,
        1  # start sending
    )


def request_message_interval(conn, message_id, interval_us):
    """Request specific message at interval (microseconds)"""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def send_xplane_vehx(xplane_sock, state):
    """
    Send VEHX packet to X-Plane to drive aircraft visuals.
    
    VEHX packet format:
    - 'VEHX' (4 bytes)
    - padding (1 byte)
    - aircraft index (int, 4 bytes)
    - latitude (double, 8 bytes)
    - longitude (double, 8 bytes)
    - elevation MSL (double, 8 bytes)
    - heading/psi (float, 4 bytes) - degrees true
    - pitch/theta (float, 4 bytes) - degrees
    - roll/phi (float, 4 bytes) - degrees
    """
    # Convert yaw from -180..180 to 0..360 for X-Plane heading
    heading = state.psi
    if heading < 0:
        heading += 360.0
    
    # Pack the VEHX message
    # Format: '<4sxidddfff' = little-endian, 4-char string, pad byte, int, 3 doubles, 3 floats
    msg = struct.pack(
        '<4sxidddfff',
        b'VEHX',
        XPLANE_AIRCRAFT_INDEX,      # Aircraft index (0 = main plane)
        state.lat,                   # Latitude in degrees
        state.lon,                   # Longitude in degrees
        state.alt_msl,               # Elevation above sea level in meters
        heading,                     # Heading (psi) in degrees true
        state.theta,                 # Pitch (theta) in degrees
        state.phi                    # Roll (phi) in degrees
    )
    
    try:
        xplane_sock.sendto(msg, (XPLANE_IP, XPLANE_PORT))
    except Exception as e:
        pass  # Silently ignore send errors


def clear_screen():
    """Clear terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')


def print_state(state):
    """Print all state variables"""
    # Move cursor to top
    print("\033[H", end="")
    
    print("=" * 70)
    print("              SITL STATE MONITOR (Target: 50 Hz)")
    print("=" * 70)
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ GLOBAL POSITION                                                 │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  Latitude:    {state.lat:+14.7f}°                              │")
    print(f"│  Longitude:   {state.lon:+14.7f}°                              │")
    print(f"│  Alt (MSL):   {state.alt_msl:+10.2f} m                                  │")
    print(f"│  Alt (AGL):   {state.alt_agl:+10.2f} m                                  │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ ATTITUDE (phi, theta, psi)                                      │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  Roll  (φ):   {state.phi:+8.2f}°                                       │")
    print(f"│  Pitch (θ):   {state.theta:+8.2f}°                                       │")
    print(f"│  Yaw   (ψ):   {state.psi:+8.2f}°                                       │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ ANGULAR RATES (p, q, r)                                         │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  p (roll rate):   {state.p:+8.2f} °/s                                 │")
    print(f"│  q (pitch rate):  {state.q:+8.2f} °/s                                 │")
    print(f"│  r (yaw rate):    {state.r:+8.2f} °/s                                 │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ LOCAL POSITION NED (relative to home)                           │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  X (North):   {state.x_ned:+10.2f} m                                   │")
    print(f"│  Y (East):    {state.y_ned:+10.2f} m                                   │")
    print(f"│  Z (Down):    {state.z_ned:+10.2f} m                                   │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  Vx:          {state.vx_ned:+10.2f} m/s                                │")
    print(f"│  Vy:          {state.vy_ned:+10.2f} m/s                                │")
    print(f"│  Vz:          {state.vz_ned:+10.2f} m/s                                │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ AIRSPEED / SPEED (from VFR_HUD)                                 │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  True Airspeed:   {state.airspeed:+10.2f} m/s                          │")
    print(f"│  Ground Speed:    {state.groundspeed:+10.2f} m/s                          │")
    print(f"│  Climb Rate:      {state.climb_rate:+10.2f} m/s                          │")
    print(f"│  Throttle:        {state.throttle:10d} %                              │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    print("┌─────────────────────────────────────────────────────────────────┐")
    print("│ DATA RATES                                                      │")
    print("├─────────────────────────────────────────────────────────────────┤")
    print(f"│  Attitude:        {state.attitude_rate:5.1f} Hz                                  │")
    print(f"│  Global Position: {state.position_rate:5.1f} Hz                                  │")
    print(f"│  Local Position:  {state.local_rate:5.1f} Hz                                  │")
    print(f"│  VFR_HUD:         {state.vfr_hud_rate:5.1f} Hz                                  │")
    print("└─────────────────────────────────────────────────────────────────┘")
    print()
    
    # X-Plane status
    xplane_status = "ENABLED" if XPLANE_ENABLED else "DISABLED"
    print(f"X-Plane Output: {xplane_status} -> {XPLANE_IP}:{XPLANE_PORT}")
    print()
    print("Press Ctrl+C to exit...")


def main():
    # Connect to SITL on UDP port 14551
    print("Connecting to udpin:127.0.0.1:14551...")
    conn = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
    
    # Wait for heartbeat to confirm connection
    print("Waiting for heartbeat...")
    conn.wait_heartbeat()
    print(f"Connected! (system {conn.target_system}, component {conn.target_component})")
    
    # Initialize X-Plane UDP socket
    xplane_sock = None
    if XPLANE_ENABLED:
        xplane_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"X-Plane output enabled -> {XPLANE_IP}:{XPLANE_PORT}")
    
    # Request data streams at 50 Hz
    print(f"Requesting data streams at {DATA_RATE_HZ} Hz...")
    
    # Request all streams at high rate
    request_data_stream(conn, mavutil.mavlink.MAV_DATA_STREAM_ALL, DATA_RATE_HZ)
    
    # Also request specific messages at 50Hz (20000us interval = 50Hz)
    interval_us = int(1000000 / DATA_RATE_HZ)
    
    # ATTITUDE = 30
    request_message_interval(conn, 30, interval_us)
    # GLOBAL_POSITION_INT = 33
    request_message_interval(conn, 33, interval_us)
    # LOCAL_POSITION_NED = 32
    request_message_interval(conn, 32, interval_us)
    # VFR_HUD = 74 (contains airspeed, groundspeed, climb rate)
    request_message_interval(conn, 74, interval_us)
    
    time.sleep(0.5)  # Wait for stream to start
    
    # Clear screen and hide cursor
    clear_screen()
    print("\033[?25l", end="")  # Hide cursor
    
    state = StateMonitor()
    last_print_time = 0
    print_interval = 0.05  # Update display at 20Hz to avoid flicker
    last_stream_request_time = time.time()
    stream_request_interval = 2.0  # Re-request streams every 2 seconds
    
    try:
        while True:
            # Periodically re-request data streams to maintain 50Hz rate
            now = time.time()
            if now - last_stream_request_time >= stream_request_interval:
                request_data_stream(conn, mavutil.mavlink.MAV_DATA_STREAM_ALL, DATA_RATE_HZ)
                request_message_interval(conn, 30, interval_us)   # ATTITUDE
                request_message_interval(conn, 33, interval_us)   # GLOBAL_POSITION_INT
                request_message_interval(conn, 32, interval_us)   # LOCAL_POSITION_NED
                request_message_interval(conn, 74, interval_us)   # VFR_HUD
                last_stream_request_time = now
            
            # Non-blocking receive
            msg = conn.recv_match(blocking=True, timeout=0.1)
            
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    state.update_from_attitude(msg)
                    # Send to X-Plane on every attitude update (highest rate)
                    if XPLANE_ENABLED and xplane_sock and state.lat != 0:
                        send_xplane_vehx(xplane_sock, state)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    state.update_from_global_position(msg)
                elif msg_type == 'LOCAL_POSITION_NED':
                    state.update_from_local_position(msg)
                elif msg_type == 'VFR_HUD':
                    state.update_from_vfr_hud(msg)
            
            # Update display at lower rate to avoid flicker
            if now - last_print_time >= print_interval:
                print_state(state)
                last_print_time = now
                
    except KeyboardInterrupt:
        print("\033[?25h", end="")  # Show cursor
        print("\n\nExiting...")
        if xplane_sock:
            xplane_sock.close()


if __name__ == "__main__":
    main()

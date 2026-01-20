#!/usr/bin/env python3
"""
Standalone test script to send VEHX packets to X-Plane.
Sends hardcoded position and attitude values to verify X-Plane connection.

Usage:
1. Start X-Plane and load any aircraft
2. Run this script: python3 test_xplane_vehx.py
3. The aircraft in X-Plane should move to the specified location

Press Ctrl+C to stop.
"""

import socket
import struct
import time
import math

# X-Plane Configuration
XPLANE_IP = "127.0.0.1"      # X-Plane IP address
XPLANE_PORT = 49000          # X-Plane default UDP port

# Hardcoded test values
TEST_LATITUDE = 37.6213      # degrees (near San Francisco)
TEST_LONGITUDE = -122.3790   # degrees
TEST_ALTITUDE_MSL = 500.0    # meters above sea level

# Animation parameters
ANIMATE = True               # Set to True to animate, False for static position


def send_vehx(sock, aircraft_idx, lat, lon, alt_msl, heading, pitch, roll):
    """
    Send VEHX packet to X-Plane.
    
    VEHX format:
    - 'VEHX' header (4 bytes)
    - padding (1 byte) 
    - aircraft index (int, 4 bytes)
    - latitude (double, 8 bytes) - degrees
    - longitude (double, 8 bytes) - degrees  
    - elevation MSL (double, 8 bytes) - meters
    - heading/psi (float, 4 bytes) - degrees true
    - pitch/theta (float, 4 bytes) - degrees
    - roll/phi (float, 4 bytes) - degrees
    """
    msg = struct.pack(
        '<4sxidddfff',
        b'VEHX',
        aircraft_idx,
        lat,
        lon,
        alt_msl,
        heading,
        pitch,
        roll
    )
    sock.sendto(msg, (XPLANE_IP, XPLANE_PORT))


def main():
    print("=" * 60)
    print("       X-Plane VEHX Packet Test")
    print("=" * 60)
    print()
    print(f"Target: {XPLANE_IP}:{XPLANE_PORT}")
    print(f"Position: {TEST_LATITUDE:.4f}, {TEST_LONGITUDE:.4f}")
    print(f"Altitude: {TEST_ALTITUDE_MSL:.1f} m MSL")
    print()
    print("Make sure X-Plane is running!")
    print("-" * 60)
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    t = 0.0
    send_rate = 50  # Hz
    dt = 1.0 / send_rate
    
    try:
        print("Sending VEHX packets... (Ctrl+C to stop)")
        print()
        
        while True:
            if ANIMATE:
                # Animate in a circle with gentle banking
                radius = 0.005  # ~500m radius in degrees
                
                # Position - fly in a circle
                lat = TEST_LATITUDE + radius * math.sin(t * 0.5)
                lon = TEST_LONGITUDE + radius * math.cos(t * 0.5)
                alt = TEST_ALTITUDE_MSL + 50 * math.sin(t * 0.3)  # Gentle altitude variation
                
                # Heading - point tangent to circle
                heading = math.degrees(t * 0.5) + 90
                if heading < 0:
                    heading += 360
                heading = heading % 360
                
                # Attitude - bank into the turn
                roll = 20 * math.sin(t * 0.5)      # Bank angle
                pitch = 5 * math.sin(t * 0.7)       # Slight pitch oscillation
            else:
                # Static position
                lat = TEST_LATITUDE
                lon = TEST_LONGITUDE
                alt = TEST_ALTITUDE_MSL
                heading = 90.0    # Facing East
                roll = 0.0
                pitch = 0.0
            
            # Send to X-Plane
            send_vehx(sock, 0, lat, lon, alt, heading, pitch, roll)
            
            # Display current values
            print(f"\rLat: {lat:+10.6f}° | Lon: {lon:+11.6f}° | Alt: {alt:6.1f}m | "
                  f"HDG: {heading:5.1f}° | Roll: {roll:+6.2f}° | Pitch: {pitch:+6.2f}°", 
                  end='', flush=True)
            
            t += dt
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        sock.close()
        print("Done!")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Test script to verify socket.io connection and commands"""

import socketio
import time
import sys

# Create a socket.io client
sio = socketio.Client(reconnection=True)

@sio.event
def connect():
    print("✅ Connected to server")
    print("   Sending throttle command...")
    sio.emit('throttle', {'value': True})
    time.sleep(0.5)
    
    print("   Sending steering command...")
    sio.emit('steering', {'angle': 45})
    time.sleep(0.5)
    
    print("   Sending brake command...")
    sio.emit('brake', {'value': True})
    time.sleep(0.5)
    
    print("   Sending gear command...")
    sio.emit('gear_change', {'gear': 'D'})
    time.sleep(0.5)
    
    print("   Disconnecting...")
    sio.disconnect()

@sio.event
def disconnect():
    print("❌ Disconnected from server")

@sio.on('throttle_response')
def on_throttle_response(data):
    print(f"📡 Throttle Response: {data}")

@sio.on('steering_response')
def on_steering_response(data):
    print(f"📡 Steering Response: {data}")

@sio.on('brake_response')
def on_brake_response(data):
    print(f"📡 Brake Response: {data}")

@sio.on('gear_response')
def on_gear_response(data):
    print(f"📡 Gear Response: {data}")

@sio.on('connection_response')
def on_connection_response(data):
    print(f"📡 Connection Response: {data}")

@sio.on('telemetry_update')
def on_telemetry_update(data):
    print(f"📡 Telemetry Update: {data}")

@sio.event
def connect_error(data):
    print(f"❌ Connection error: {data}")

# Connect to the server
try:
    print("Attempting to connect to http://127.0.0.1:5000...")
    sio.connect('http://127.0.0.1:5000',
                transports=['websocket', 'polling'],
                wait_timeout=10)
    
    # Keep the connection alive for a bit
    time.sleep(2)
except Exception as e:
    print(f"❌ Failed to connect: {e}")
    sys.exit(1)

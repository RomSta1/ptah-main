#!/usr/bin/env python3
"""
Simulator drone backend — own_sim browser simulator via WebSocket.

How it works:
    Browser (sim.dremian.com) <-> WebSocket proxy (:8080) <-> this Python client

Controllers output RC [1000-2000] — this module converts to sim [-1, 1] internally.
WebSocket client from hover2/nn_hover2/sim_client.py.
"""

import os
import sys
import time
import json
import asyncio
import threading
import socket
import subprocess

import websockets  # WebSocket protocol library

RC_MID = 1500   # neutral RC value (center stick)
RC_RANGE = 500  # half of RC range: (2000 - 1000) / 2


def _rc_to_sim(rc_value):
    """Convert RC [1000-2000] to simulator [-1, 1].
    1000 → -1.0, 1500 → 0.0, 2000 → 1.0."""
    return max(-1.0, min(1.0, (rc_value - RC_MID) / RC_RANGE))


# ---------------------------------------------------------------------------
# Drone state — stores latest telemetry from simulator
# ---------------------------------------------------------------------------

class _DroneState:
    """Latest drone telemetry received from simulator."""
    def __init__(self):
        self.roll = 0.0      # degrees
        self.pitch = 0.0     # degrees
        self.yaw = 0.0       # degrees
        self.altitude = 0.0  # meters

    def update(self, data):
        """Parse telemetry message from simulator."""
        o = data['orientation']
        self.roll = o['roll']
        self.pitch = o['pitch']
        self.yaw = o['yaw']
        self.altitude = data['position']['altitude']


# ---------------------------------------------------------------------------
# WebSocket client — talks to simulator through proxy
# From hover2/nn_hover2/sim_client.py
# ---------------------------------------------------------------------------

class _SimClient:
    """WebSocket client for own_sim simulator.
    Runs async event loop in a separate thread so main thread stays synchronous."""

    def __init__(self, url="ws://localhost:8080"):
        self.url = url
        self.ws = None               # WebSocket connection
        self.connected = False       # True after successful connection
        self.running = True          # False when disconnecting
        self.state = _DroneState()   # latest telemetry data
        self._loop = None            # asyncio event loop (runs in thread)
        self._thread = None          # thread running the event loop
        self._frame_count = 0        # how many telemetry frames received

    def connect(self):
        """Connect to WebSocket proxy. Blocking — waits up to 5 seconds."""
        # create new event loop in a daemon thread (dies when main thread exits)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        # schedule async connection and wait for it
        event = threading.Event()
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._connect_async(event))
        )
        event.wait(timeout=5)
        return self.connected

    def _run_loop(self):
        """Thread target: run asyncio event loop forever."""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect_async(self, event):
        """Async: open WebSocket and start receiving telemetry."""
        self.ws = await websockets.connect(self.url)
        self.connected = True
        print(f"Connected to {self.url}")
        asyncio.create_task(self._receive_loop())  # start listening for messages
        event.set()  # unblock the calling thread

    def start_telemetry(self, rate=30, width=64, height=48, camera_angle=0):
        """Request simulator to start sending telemetry at given rate."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._start_telemetry(rate, width, height, camera_angle))
        )

    async def _start_telemetry(self, rate, width, height, camera_angle):
        """Async: send camera stream request — simulator responds with telemetry frames."""
        msg = {
            "type": "camera_stream_multi",
            "data": {
                "cameraId": "camera1", "active": True,
                "rate": rate, "quality": 0.1,
                "width": width, "height": height, "angle": camera_angle,
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))

    def set_position(self, latitude=0, longitude=0, altitude=0, yaw=0):
        """Teleport drone to given position in simulator."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._set_position(latitude, longitude, altitude, yaw))
        )

    async def _set_position(self, lat, lon, alt, yaw):
        """Async: send setPosition command to simulator."""
        msg = {
            "type": "command",
            "data": {
                "command": "setPosition",
                "params": {
                    "latitude": lat, "longitude": lon, "altitude": alt,
                    "roll": 0, "pitch": 0, "yaw": yaw,
                    "velocityNorth": 0, "velocityEast": 0, "velocityDown": 0,
                }
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))

    def send_control(self, roll=0, pitch=0, yaw=0, throttle=0):
        """Send control commands to simulator. Values in [-1, 1]."""
        if not self.connected:
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.ensure_future(self._send_control(roll, pitch, yaw, throttle))
        )

    async def _send_control(self, roll, pitch, yaw, throttle):
        """Async: send control message to simulator."""
        msg = {
            "type": "control",
            "data": {
                "roll": max(-1, min(1, roll)),
                "pitch": max(-1, min(1, pitch)),
                "yaw": max(-1, min(1, yaw)),
                "throttle": max(-1, min(1, throttle)),
            },
            "sender": "client",
            "timestamp": int(time.time() * 1000)
        }
        await self.ws.send(json.dumps(msg))

    async def _receive_loop(self):
        """Async: continuously receive messages from simulator.
        Parses camera frames which contain droneState telemetry."""
        try:
            while self.running and self.connected:
                message = await self.ws.recv()
                data = json.loads(message)

                # telemetry comes inside camera_frame_multi messages
                if data["type"] == "camera_frame_multi":
                    frame_data = data["data"]
                    if frame_data.get("cameraId") == "camera1" and "droneState" in frame_data:
                        self.state.update(frame_data["droneState"])
                        self._frame_count += 1
                        if self._frame_count == 1:
                            print("First telemetry received")
        except websockets.exceptions.ConnectionClosed:
            print("Connection closed")
            self.connected = False

    def disconnect(self):
        """Stop event loop and join thread."""
        self.running = False
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=1)


# ---------------------------------------------------------------------------
# Proxy helpers — auto-starts websocket_proxy.py if not running
# ---------------------------------------------------------------------------

def _is_port_open(host="localhost", port=8080):
    """Check if something is listening on the port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def _ensure_proxy():
    """Start websocket_proxy.py if not already running. Returns process or None."""
    if _is_port_open():
        print("Proxy already running on :8080")
        return None

    # proxy script should be in same directory as this file
    proxy_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "websocket_proxy.py")
    if not os.path.isfile(proxy_path):
        print(f"ERROR: {proxy_path} not found!")
        sys.exit(1)

    print("Starting websocket proxy...")
    proc = subprocess.Popen(
        [sys.executable, proxy_path],       # run with same Python interpreter
        stdout=subprocess.DEVNULL,          # hide proxy output
        stderr=subprocess.DEVNULL,
    )

    # wait up to 2 seconds for proxy to start listening
    for _ in range(20):
        time.sleep(0.1)
        if _is_port_open():
            print("Proxy started")
            return proc

    print("WARNING: Proxy may not have started")
    return proc


# ---------------------------------------------------------------------------
# SimDrone — same interface as BetaflightDrone
# ---------------------------------------------------------------------------

class SimDrone:
    """Simulator drone. Same interface as BetaflightDrone.
    Accepts RC [1000-2000], converts to sim [-1,1] automatically."""

    def __init__(self, start_altitude=10.0):
        self.start_altitude = start_altitude  # where to spawn drone in simulator
        self._client = None       # _SimClient WebSocket connection
        self._proxy_proc = None   # websocket_proxy.py subprocess

    def connect(self):
        """Start proxy, connect to simulator, spawn drone, start telemetry.
        Returns True if everything is ready."""

        # step 1: ensure WebSocket proxy is running
        self._proxy_proc = _ensure_proxy()

        # step 2: connect to proxy via WebSocket
        self._client = _SimClient()
        if not self._client.connect():
            print("Failed to connect! Is browser sim open?")
            return False

        # step 3: teleport drone to starting position
        self._client.set_position(
            latitude=50.45, longitude=30.52,      # coordinates (Kyiv area)
            altitude=self.start_altitude, yaw=0,
        )
        time.sleep(0.3)  # wait for position to apply

        # step 4: start telemetry stream at 50Hz
        self._client.start_telemetry(rate=50, width=400, height=400)
        time.sleep(0.5)  # wait for stream to start

        # step 5: verify we're receiving telemetry
        timeout = time.time() + 5
        while self._client._frame_count == 0 and time.time() < timeout:
            time.sleep(0.05)

        if self._client._frame_count == 0:
            print("No telemetry! Make sure browser simulator is open.")
            self._client.disconnect()
            return False

        print(f"Simulator ready. Drone at {self.start_altitude}m.")
        return True

    def read_state(self):
        """Returns (roll, pitch, yaw, altitude) from latest telemetry."""
        s = self._client.state
        return s.roll, s.pitch, s.yaw, s.altitude

    def send_command(self, roll, pitch, yaw, throttle):
        """Accept RC [1000-2000], convert to sim [-1,1], send to simulator."""
        self._client.send_control(
            roll=_rc_to_sim(roll), pitch=_rc_to_sim(pitch),
            yaw=_rc_to_sim(yaw), throttle=_rc_to_sim(throttle),
        )

    def disconnect(self):
        """Send zero commands, disconnect client, stop proxy."""
        if self._client:
            self._client.send_control(0, 0, 0, 0)  # neutral before disconnect
            time.sleep(0.1)
            self._client.disconnect()
            self._client = None
        if self._proxy_proc:
            self._proxy_proc.terminate()  # kill proxy subprocess
            self._proxy_proc = None
        print('Simulator disconnected.')

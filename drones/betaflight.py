#!/usr/bin/env python3
"""
Real drone backend — Betaflight via MSP protocol.

Hardware: Raspberry Pi UART (GPIO 14/15) → Flight Controller at 115200 baud.
Protocol: MSP (Multiwii Serial Protocol) via yamspy library.

Betaflight config required:
    set msp_override_channels_mask = 15   # allow Pi to override channels 1-4
    aux 3 50 3 1550 2100 0 0              # AUX4 switch enables MSP override
    save

AUX4 detection:
    is_aux4_high() reads RC channels via MSP_RC and checks channel 8 (AUX4).
    When AUX4 > 1550, Betaflight activates MSP override — Pi takes control.
    Channel mapping: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw, 5-8=AUX1-4.
"""

import struct           # for packing RC channel data into MSP binary format
from yamspy import MSPy  # MSP protocol library for Betaflight communication

# serial ports to try — different Raspberry Pi models use different ports
PORTS = ['/dev/ttyAMA0', '/dev/serial0', '/dev/ttyAMA10', '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyAMA1']
BAUDRATE = 115200  # standard MSP baud rate

RC_MID = 1500  # neutral RC value (center stick)
RC_MIN = 1000  # minimum RC value
RC_MAX = 2000  # maximum RC value

# AUX4 = channel 8 = index 7 (0-based) in RC channels array
# threshold 1550 matches Betaflight config: aux 3 50 3 1550 2100 0 0
AUX4_CHANNEL_INDEX = 7
AUX4_THRESHOLD = 1550


def _clamp_rc(value):
    """Keep value within valid RC range [1000, 2000]."""
    return max(RC_MIN, min(RC_MAX, int(value)))


class BetaflightDrone:
    """
    Real drone interface.
    Methods: connect(), read_state(), send_command(), is_aux4_high(), disconnect().
    All MSP functions from stabilizer/nn_stabilizer/stabilize.py.
    """

    def __init__(self):
        self.board = None  # MSPy connection object

    def connect(self):
        """Try each serial port until one works. Returns True if connected."""
        for port in PORTS:
            print(f'Trying {port}...', end=' ')
            try:
                # open serial connection to flight controller
                board = MSPy(device=port, logfilename=None, loglevel='WARNING', baudrate=BAUDRATE)
                board.__enter__()

                # verify connection with API version request
                board.send_RAW_msg(MSPy.MSPCodes['MSP_API_VERSION'])
                msg = board.receive_msg()
                if msg['packet_error'] == 0:
                    print('connected!')
                    self.board = board
                    return True

                # wrong port — close and try next
                board.__exit__(None, None, None)
            except Exception as e:
                print(f'failed ({e})')

        print('No drone found on any port.')
        return False

    def read_state(self):
        """Read current attitude and altitude from flight controller.
        Returns (roll, pitch, yaw, altitude) in degrees and meters."""

        # request attitude (roll, pitch, yaw in degrees)
        self.board.send_RAW_msg(MSPy.MSPCodes['MSP_ATTITUDE'])
        msg = self.board.receive_msg()
        self.board.process_recv_data(msg)
        k = self.board.SENSOR_DATA['kinematics']
        roll, pitch, yaw = k[0], k[1], k[2]

        # request altitude (barometer, in meters)
        self.board.send_RAW_msg(MSPy.MSPCodes['MSP_ALTITUDE'])
        msg = self.board.receive_msg()
        self.board.process_recv_data(msg)
        altitude = self.board.SENSOR_DATA['altitude']

        return roll, pitch, yaw, altitude

    def is_aux4_high(self):
        """Check if AUX4 switch is activated (> 1550).
        When True, Betaflight MSP override is active — Pi controls the drone.
        When False, pilot has manual control via RC transmitter."""

        # read all RC channel values from flight controller
        self.board.send_RAW_msg(MSPy.MSPCodes['MSP_RC'])
        msg = self.board.receive_msg()

        # parse raw response: N × uint16 little-endian
        data = bytes(msg['dataView'])
        n_channels = len(data) // 2
        if n_channels < AUX4_CHANNEL_INDEX + 1:
            return False  # receiver doesn't have enough channels
        channels = struct.unpack(f'<{n_channels}H', data)

        return channels[AUX4_CHANNEL_INDEX] > AUX4_THRESHOLD

    def send_command(self, roll, pitch, yaw, throttle):
        """Send RC override to flight controller. Values in [1000-2000].
        Channel order is AETR — throttle and yaw are swapped vs arguments."""

        channels = [
            _clamp_rc(roll),       # channel 1 = aileron  (roll)
            _clamp_rc(pitch),      # channel 2 = elevator (pitch)
            _clamp_rc(throttle),   # channel 3 = throttle (AETR order!)
            _clamp_rc(yaw),        # channel 4 = rudder   (yaw, AETR order!)
        ]

        # pack as 4 unsigned 16-bit integers, little-endian
        data = struct.pack('<4H', *channels)

        # send via MSP_SET_RAW_RC command
        self.board.send_RAW_msg(MSPy.MSPCodes['MSP_SET_RAW_RC'], data)
        self.board.receive_msg()  # read acknowledgement

    def disconnect(self):
        """Send neutral commands (safety) and close serial connection."""
        if self.board:
            # try to send neutral before closing — prevents drone from holding last command
            try:
                self.send_command(RC_MID, RC_MID, RC_MID, RC_MID)
            except Exception:
                pass  # if send fails, still close connection
            self.board.__exit__(None, None, None)
            self.board = None
        print('Betaflight disconnected.')

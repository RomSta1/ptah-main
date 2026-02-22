#!/usr/bin/env python3
"""
Step 1 controller — fixed commands.
Sends constant values to verify that the control path works.
If OSD doesn't show these values — problem is in MSP, not in controller.
"""

RC_MID = 1500          # neutral position for roll/pitch/yaw
FIXED_THROTTLE = 1300  # below hover — drone descends slowly (safe for testing)


class FixedController:
    """No sensor feedback. Always returns the same RC commands."""

    def reset(self, target_alt):
        # fixed controller ignores altitude — nothing to reset
        pass

    def compute(self, roll, pitch, yaw, altitude, dt):
        # all axes neutral, throttle low — drone should descend slowly
        return RC_MID, RC_MID, RC_MID, FIXED_THROTTLE

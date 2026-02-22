#!/usr/bin/env python3
"""
Step 3 controller — neural network (LSTM) altitude hold.

Model: jax_hover_test.onnx — trained LSTM with hidden state.
Inputs:  obs[1,8] normalized + h[1,256] + c[1,256]
Outputs: action[1,4] in [-1, 1] + h[1,256] + c[1,256]

Only throttle output is used; roll/pitch/yaw stay neutral (1500).
Normalization pattern from hover2/nn_hover2/hover_onnx.py.

Call reset(target_alt) before first compute() — sets target and clears LSTM state.
Model is loaded once in __init__ (slow) — do this BEFORE waiting for AUX4.
"""

import os
import numpy as np
import onnxruntime as ort  # ONNX Runtime — runs .onnx neural network models

RC_MID = 1500  # neutral RC value
RC_MIN = 1000  # minimum RC value
RC_MAX = 2000  # maximum RC value

# model file in bird-main/ root directory
DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),  # controllers/
    '..',                                         # bird-main/
    'jax_hover_test.onnx'                         # LSTM model file
)


def _clamp_rc(value):
    """Keep value within valid RC range [1000, 2000]."""
    return max(RC_MIN, min(RC_MAX, int(value)))


def _sim_to_rc(value):
    """Convert model output [-1, 1] to RC [1000, 2000].
    -1.0 → 1000, 0.0 → 1500, 1.0 → 2000."""
    return _clamp_rc(value * 500 + 1500)


class NNController:
    """LSTM neural network altitude hold.

    The model has memory (hidden state h, c) — it remembers previous steps.
    Each call to compute() updates the hidden state.
    reset() clears the memory for a fresh start (new AUX4 activation).
    """

    def __init__(self, model_path=DEFAULT_MODEL_PATH):
        # load ONNX model — crashes with clear error if file missing or broken
        # this is SLOW (1-5 sec on Raspberry Pi) — do this at startup, not at AUX4
        self.session = ort.InferenceSession(model_path)
        print(f'NN model loaded: {model_path}')

        # verify this is an LSTM model (3 inputs: obs, h_in, c_in)
        input_names = [inp.name for inp in self.session.get_inputs()]
        if len(input_names) != 3:
            raise ValueError(
                f"Expected LSTM model with 3 inputs (obs, h_in, c_in), "
                f"got {len(input_names)}: {input_names}"
            )
        print("  Model type: LSTM")

        # state will be initialized by reset()
        self.target_alt = None
        self.h = None
        self.c = None
        self.prev_action = None

    def reset(self, target_alt):
        """Set target altitude and clear LSTM state. Call before first compute()."""
        self.target_alt = target_alt
        # reset LSTM hidden state to zeros — model starts with no memory
        self.h = np.zeros((1, 256), dtype=np.float32)
        self.c = np.zeros((1, 256), dtype=np.float32)
        # reset previous action feedback to neutral (no prior commands)
        self.prev_action = np.zeros(4, dtype=np.float32)
        print(f"  LSTM reset. Target: {target_alt:.2f}m")

    def compute(self, roll, pitch, yaw, altitude, dt):
        """Run LSTM inference. Returns (roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd)."""

        # normalize inputs to [-1, 1] range (from hover_onnx.py)
        obs = np.array([[
            np.clip(roll / 180, -1, 1),                         # roll degrees → [-1, 1]
            np.clip(pitch / 180, -1, 1),                        # pitch degrees → [-1, 1]
            np.clip(yaw / 180, -1, 1),                          # yaw degrees → [-1, 1]
            np.clip((altitude - self.target_alt) / 50, -1, 1),  # alt error → [-1, 1]
            self.prev_action[0], self.prev_action[1],            # prev roll/pitch commands
            self.prev_action[2], self.prev_action[3],            # prev yaw/throttle commands
        ]], dtype=np.float32)

        # run LSTM — updates hidden state (h, c) with each step
        action, self.h, self.c = self.session.run(
            None, {"obs": obs, "h_in": self.h, "c_in": self.c}
        )
        action = np.clip(action[0], -1, 1)  # clamp output to [-1, 1]

        # update prev_action with what was ACTUALLY SENT to the drone
        # roll/pitch/yaw are forced neutral (0.0), only throttle uses model output
        self.prev_action[0] = 0.0          # neutral roll was sent
        self.prev_action[1] = 0.0          # neutral pitch was sent
        self.prev_action[2] = 0.0          # neutral yaw was sent
        self.prev_action[3] = action[3]    # throttle from model was sent

        # convert model output [-1, 1] to RC [1000, 2000]
        return RC_MID, RC_MID, RC_MID, _sim_to_rc(action[3])

#!/usr/bin/env python3
"""
Bird-Main — drone altitude hold test framework.

Startup flow:
    1. Connect to drone (MSP serial)
    2. Create controller (loads NN model if needed — slow, do this first)
    3. Wait for AUX4 switch activation (pilot flies manually, then flips AUX4)
    4. Capture current altitude as target (drone is already in the air)
    5. Start control loop immediately

Usage:
    python main.py --controller fixed    # Step 1: fixed commands
    python main.py --controller pid      # Step 2: PID altitude hold
    python main.py --controller nn       # Step 3: NN altitude hold
    python main.py --drone sim --controller pid  # simulator (no AUX4 wait)

Crontab autostart (Raspberry Pi):
    @reboot sleep 10 && cd /home/pi/bird-main && python3 main.py --controller fixed >> logs/autostart.log 2>&1
"""

import os
import sys
import csv
import time
import argparse
from datetime import datetime

# line-buffered stdout — makes \r status line visible when output is redirected to file
sys.stdout.reconfigure(line_buffering=True)

# add project root to Python path so "from controllers..." and "from drones..." work
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# CSV flight logger — records every control loop step
# ---------------------------------------------------------------------------

LOG_COLUMNS = [
    "timestamp",     # seconds since start
    "step",          # loop iteration number
    "roll",          # current roll (degrees)
    "pitch",         # current pitch (degrees)
    "yaw",           # current yaw (degrees)
    "altitude",      # current altitude (meters)
    "roll_cmd",      # roll command sent (RC 1000-2000)
    "pitch_cmd",     # pitch command sent
    "yaw_cmd",       # yaw command sent
    "throttle_cmd",  # throttle command sent
    "loop_dt",       # actual loop duration (seconds)
]


class FlightLogger:
    """Writes flight data to CSV file in logs/ directory."""

    def __init__(self, log_dir, prefix):
        os.makedirs(log_dir, exist_ok=True)  # create logs/ if it doesn't exist
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f"{prefix}_{ts}.csv")
        self._file = open(self.filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(LOG_COLUMNS)  # write header row
        self._start_time = time.time()
        print(f"Logging to: {self.filepath}")

    def log(self, step, roll, pitch, yaw, altitude, r_cmd, p_cmd, y_cmd, t_cmd, loop_dt):
        """Write one row of flight data."""
        self._writer.writerow([
            f"{time.time() - self._start_time:.4f}", step,
            f"{roll:.3f}", f"{pitch:.3f}", f"{yaw:.3f}", f"{altitude:.3f}",
            r_cmd, p_cmd, y_cmd, t_cmd, f"{loop_dt:.5f}",
        ])
        # flush to disk every 50 steps (1 second at 50Hz) to prevent data loss
        if step % 50 == 0:
            self._file.flush()

    def close(self):
        """Flush remaining data and close file."""
        self._file.flush()
        self._file.close()
        print(f"Log saved: {self.filepath}")


# ---------------------------------------------------------------------------
# Factory functions — create drone/controller by name
# ---------------------------------------------------------------------------

def create_drone(drone_type):
    """Create drone backend: 'betaflight' (real) or 'sim' (browser simulator)."""
    if drone_type == "betaflight":
        from drones.betaflight import BetaflightDrone
        return BetaflightDrone()
    elif drone_type == "sim":
        from drones.own_sim import SimDrone
        return SimDrone(start_altitude=10.0)  # simulator spawns drone at 10m
    else:
        print(f"ERROR: Unknown drone '{drone_type}'")
        sys.exit(1)


def create_controller(controller_type):
    """Create controller. Call reset(target_alt) before first compute().
    For NN: model is loaded here (slow, 1-5 sec on Pi) — do before AUX4 wait."""
    if controller_type == "fixed":
        from controllers.alt_hold_fixed import FixedController
        return FixedController()
    elif controller_type == "pid":
        from controllers.alt_hold_pid import PIDController
        return PIDController()
    elif controller_type == "nn":
        from controllers.alt_hold_nn import NNController
        return NNController()
    else:
        print(f"ERROR: Unknown controller '{controller_type}'")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # --- parse command line arguments ---
    parser = argparse.ArgumentParser(description="Bird-Main: drone altitude hold")
    parser.add_argument("--drone", default="betaflight", choices=["betaflight", "sim"],
                        help="Drone backend (default: betaflight)")
    parser.add_argument("--controller", required=True, choices=["fixed", "pid", "nn"],
                        help="Controller: fixed (step 1), pid (step 2), nn (step 3)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Control loop frequency (default: 50)")
    parser.add_argument("--log-dir", type=str, default=None,
                        help="Log directory (default: logs/)")
    args = parser.parse_args()

    project_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = args.log_dir or os.path.join(project_dir, "logs")
    dt = 1.0 / args.hz  # time per loop iteration

    # --- step 1: connect to drone ---
    print(f"\n[1/4] Connecting to {args.drone}...")
    drone = create_drone(args.drone)
    if not drone.connect():
        print("FAILED to connect. Exiting.")
        sys.exit(1)

    # --- step 2: verify telemetry works ---
    print(f"[2/4] Checking telemetry...")
    roll, pitch, yaw, altitude = drone.read_state()
    print(f"  OK: roll={roll:.1f}  pitch={pitch:.1f}  yaw={yaw:.1f}  alt={altitude:.2f}m")

    # --- step 3: create controller (loads NN model if needed — BEFORE AUX4 wait) ---
    print(f"[3/4] Loading controller '{args.controller}'...")
    controller = create_controller(args.controller)
    print(f"  OK: controller ready.")

    # --- step 4: main loop ---
    if args.drone == "betaflight":
        # OUTER LOOP: wait for AUX4 → control → AUX4 off → repeat
        # this lets the pilot test multiple times without restarting the program
        run_number = 0
        logger = None
        try:
            while True:
                # === PHASE 1: wait for AUX4 ON ===
                run_number += 1
                print(f"\n{'=' * 50}")
                print(f"  Run #{run_number} — waiting for AUX4")
                print(f"  Fly to desired altitude, then flip AUX4.")
                print(f"{'=' * 50}")
                print(f'  {"ROLL":>6} {"PITCH":>6} {"YAW":>6} {"ALT":>7}  AUX4')

                while not drone.is_aux4_high():
                    # show live telemetry — pilot sees sensors are working
                    roll, pitch, yaw, altitude = drone.read_state()
                    print(f'\r  {roll:6.1f} {pitch:6.1f} {yaw:6.1f} {altitude:7.2f}m  OFF ',
                          end='', flush=True)
                    # send neutral while waiting — keeps MSP link fresh
                    drone.send_command(1500, 1500, 1500, 1300)
                    time.sleep(dt)

                # === PHASE 2: AUX4 just went ON — capture altitude ===
                roll, pitch, yaw, target_alt = drone.read_state()
                controller.reset(target_alt)
                print(f'\r  {roll:6.1f} {pitch:6.1f} {yaw:6.1f} {target_alt:7.2f}m  ON  ')
                print(f"\n>>> AUX4 ON — controlling! Target: {target_alt:.2f}m <<<")

                # new log file for each run
                if logger:
                    logger.close()
                logger = FlightLogger(log_dir=log_dir,
                                      prefix=f"{args.controller}_{args.drone}_run{run_number}")

                print(f'{"ROLL":>7} {"PITCH":>7} {"YAW":>7} {"ALT":>7}  ->  '
                      f'{"R_CMD":>6} {"P_CMD":>6} {"Y_CMD":>6} {"T_CMD":>6}')
                print('-' * 72)

                # === PHASE 3: control loop — runs while AUX4 is ON ===
                step = 0
                # check AUX4 every 10 iterations (5 times/sec at 50Hz)
                aux4_check_interval = max(1, args.hz // 5)

                while True:
                    t0 = time.time()

                    roll, pitch, yaw, altitude = drone.read_state()
                    r, p, y, t = controller.compute(roll, pitch, yaw, altitude, dt)
                    drone.send_command(r, p, y, t)

                    # check AUX4 periodically — if OFF, return to waiting phase
                    if step % aux4_check_interval == 0:
                        if not drone.is_aux4_high():
                            break  # exit inner loop → back to PHASE 1

                    # print status ~2 times per second
                    if step % max(1, args.hz // 2) == 0:
                        print(f'\r{roll:7.1f} {pitch:7.1f} {yaw:7.1f} {altitude:7.2f}'
                              f'  ->  {r:6.0f} {p:6.0f} {y:6.0f} {t:6.0f}',
                              end='', flush=True)

                    step += 1
                    elapsed = time.time() - t0
                    logger.log(step - 1, roll, pitch, yaw, altitude, r, p, y, t, elapsed)

                    if elapsed < dt:
                        time.sleep(dt - elapsed)

                # === AUX4 went OFF ===
                print(f"\n\n>>> AUX4 OFF — manual control. {step} steps in run #{run_number}. <<<")

        except KeyboardInterrupt:
            print('\n\nStopping...')
        finally:
            if logger:
                logger.close()
            drone.disconnect()

    else:
        # SIMULATOR: no AUX4, single run — start immediately
        print(f"\n[4/4] Simulator — starting immediately.")
        _, _, _, target_alt = drone.read_state()
        controller.reset(target_alt)
        print(f"Target altitude: {target_alt:.2f}m")

        logger = FlightLogger(log_dir=log_dir, prefix=f"{args.controller}_{args.drone}")
        print(f'\n{"ROLL":>7} {"PITCH":>7} {"YAW":>7} {"ALT":>7}  ->  '
              f'{"R_CMD":>6} {"P_CMD":>6} {"Y_CMD":>6} {"T_CMD":>6}')
        print('-' * 72)

        step = 0
        try:
            while True:
                t0 = time.time()
                roll, pitch, yaw, altitude = drone.read_state()
                r, p, y, t = controller.compute(roll, pitch, yaw, altitude, dt)
                drone.send_command(r, p, y, t)

                if step % max(1, args.hz // 2) == 0:
                    print(f'\r{roll:7.1f} {pitch:7.1f} {yaw:7.1f} {altitude:7.2f}'
                          f'  ->  {r:6.0f} {p:6.0f} {y:6.0f} {t:6.0f}',
                          end='', flush=True)

                step += 1
                elapsed = time.time() - t0
                logger.log(step - 1, roll, pitch, yaw, altitude, r, p, y, t, elapsed)

                if elapsed < dt:
                    time.sleep(dt - elapsed)
        except KeyboardInterrupt:
            print('\n\nStopping...')
        finally:
            logger.close()
            drone.disconnect()
            print(f'Completed {step} steps.')


if __name__ == '__main__':
    main()

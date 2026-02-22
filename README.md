# Bird-Main

Drone altitude hold testing framework. Three progressive test steps, each with simulator and real drone support.

## Structure

```
bird-main/
├── controllers/
│   ├── alt_hold_fixed.py    # Step 1: fixed commands (1500/1500/1500/1300)
│   ├── alt_hold_pid.py      # Step 2: PID altitude hold
│   └── alt_hold_nn.py       # Step 3: neural network altitude hold
├── drones/
│   ├── betaflight.py        # Real drone via MSP protocol
│   ├── own_sim.py           # Browser simulator via WebSocket
│   └── websocket_proxy.py   # WebSocket proxy for simulator
├── main.py                  # Entry point
├── jax_hover_test.onnx      # LSTM model for Step 3
└── requirements.txt
```

## Setup

```bash
pip install -r requirements.txt
```

## Usage

```bash
# Step 1: fixed commands (verify control path)
python main.py --controller fixed

# Step 2: PID altitude hold
python main.py --controller pid

# Step 3: neural network altitude hold
python main.py --controller nn

# Simulator (no AUX4 wait — starts immediately)
python main.py --drone sim --controller pid
```

## How it works (real drone)

1. Pi boots → connects MSP → loads controller (NN model if step 3)
2. Prints "Waiting for AUX4..." — sends neutral commands in background
3. You fly the drone manually to desired altitude (~3m)
4. You flip AUX4 switch on transmitter
5. Pi instantly detects AUX4 → captures current altitude → starts controlling
6. Target altitude = whatever height the drone was at when AUX4 was flipped
7. Ctrl+C or flip AUX4 off to stop (Betaflight auto-returns to manual)

## Simulator

1. Open browser: https://sim.dremian.com/simulator.html
2. Run with `--drone sim` (proxy starts automatically, no AUX4 wait)

## Real Drone (Raspberry Pi)

### Wiring

| Raspberry Pi | GPIO | Pin | Drone FC |
|---|---|---|---|
| UART TX | GPIO 14 | Pin 8 | Free UART RX |
| UART RX | GPIO 15 | Pin 10 | Free UART TX |
| GND | | | GND |

### Betaflight config

```
set msp_override_channels_mask = 15
aux 3 50 3 1550 2100 0 0
save
```

Enable MSP on the UART port in Betaflight Configurator → Ports tab.

### Crontab autostart (tmux)

Install tmux (once):
```bash
sudo apt install tmux
```

Edit crontab:
```bash
crontab -e
```

Add:
```
@reboot sleep 10 && tmux new-session -d -s bird 'cd /home/pi/bird-main && python3 main.py --controller fixed'
```

Change `--controller fixed` to `pid` or `nn` when ready for next step.

### SSH monitoring

Connect to Pi and attach to the tmux session:
```bash
ssh pi@<IP>
tmux attach -t bird
```

You see everything live: connection status, telemetry, AUX4 state, control output.

Detach without stopping the program: `Ctrl+B`, then `D`.

Re-attach any time: `tmux attach -t bird`.

Check if session exists: `tmux ls`.

Stop the program: attach and press `Ctrl+C`.

### Quick reference

| Action | Command |
|---|---|
| SSH into Pi | `ssh pi@<IP>` |
| See live output | `tmux attach -t bird` |
| Detach (keep running) | `Ctrl+B`, then `D` |
| Stop program | `Ctrl+C` (while attached) |
| Change controller | Edit crontab: `crontab -e` |
| Check logs | `ls ~/bird-main/logs/` |
| Reboot Pi | `sudo reboot` |

## Logs

Flight data saved as CSV in `logs/` directory, one file per AUX4 activation:
- `pid_betaflight_run1_20260222_143052.csv`
- `pid_betaflight_run2_20260222_143120.csv`

Columns: timestamp, step, roll, pitch, yaw, altitude, roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd, loop_dt.

## Testing flow

1. **Step 1 (fixed)**: Verify OSD shows correct values → table test → short flight with AUX4
2. **Step 2 (PID)**: Simulator first → table test → flight with AUX4
3. **Step 3 (NN)**: Same as Step 2 but with trained model

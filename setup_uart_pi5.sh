#!/bin/bash
# Setup UART on Raspberry Pi 5 for Betaflight MSP (GPIO14=TX, GPIO15=RX)
# Run once: sudo bash setup_uart_pi5.sh

set -e

echo "=== Pi 5 UART setup ==="

# 1. Enable UART hardware (puts GPIO14/15 into TXD0/RXD0 mode)
CONFIG=/boot/firmware/config.txt
if grep -q '^enable_uart=1' "$CONFIG"; then
    echo "[1] enable_uart=1 already set"
else
    # Insert after [all] section tag
    sudo sed -i 's/^\[all\]/[all]\nenable_uart=1/' "$CONFIG"
    echo "[1] enable_uart=1 added to [all]"
fi

# 2. Remove serial console from kernel cmdline (frees the port from kernel use)
CMDLINE=/boot/firmware/cmdline.txt
if grep -q 'console=serial0' "$CMDLINE"; then
    sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
    echo "[2] console=serial0 removed from cmdline.txt"
else
    echo "[2] console=serial0 not in cmdline.txt — OK"
fi

# 3. Disable login shell on serial port (frees the port from getty)
if systemctl is-enabled serial-getty@ttyAMA0.service &>/dev/null; then
    sudo systemctl disable serial-getty@ttyAMA0.service
    sudo systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true
    echo "[3] serial-getty@ttyAMA0 disabled"
else
    echo "[3] serial-getty@ttyAMA0 not enabled — OK"
fi
# Also disable ttyAMA10 variant in case it exists
sudo systemctl disable serial-getty@ttyAMA10.service 2>/dev/null || true

# 4. udev rule: set group=dialout and rw permissions on every boot
UDEV=/etc/udev/rules.d/99-ttyAMA0.rules
echo 'KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' | sudo tee "$UDEV" > /dev/null
sudo udevadm control --reload-rules
echo "[4] udev rule created: $UDEV"

# 5. Verify user is in dialout group
USER=${SUDO_USER:-$(whoami)}
if id -nG "$USER" | grep -qw dialout; then
    echo "[5] $USER is in dialout group — OK"
else
    sudo usermod -aG dialout "$USER"
    echo "[5] $USER added to dialout group (re-login needed)"
fi

# 6. Crontab autostart — fixed controller on boot
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CRON_JOB="@reboot sleep 10 && cd $SCRIPT_DIR && python3 main.py --controller fixed >> logs/autostart.log 2>&1"
if crontab -u "$USER" -l 2>/dev/null | grep -q 'main.py'; then
    echo "[6] crontab already has main.py entry — skipping"
else
    (crontab -u "$USER" -l 2>/dev/null; echo "$CRON_JOB") | crontab -u "$USER" -
    echo "[6] crontab autostart added: $CRON_JOB"
fi

echo ""
echo "=== Done. Rebooting in 5s (Ctrl+C to cancel) ==="
sleep 5
sudo reboot

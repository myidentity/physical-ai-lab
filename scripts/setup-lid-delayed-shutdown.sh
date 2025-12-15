#!/bin/bash
#
# Setup script: Configure delayed shutdown on lid close (battery only)
#
# Behavior:
#   - On battery: Wait 15 minutes after lid close, then shutdown
#   - On AC power: Lock screen immediately
#
# Run with: sudo ./setup-lid-delayed-shutdown.sh
#

set -e

DELAY_MINUTES=15
POLL_INTERVAL=10

echo "=============================================="
echo " Lid Delayed Shutdown Setup"
echo "=============================================="
echo ""
echo "This will configure:"
echo "  • On battery + lid close: shutdown after ${DELAY_MINUTES} minutes"
echo "  • On AC power + lid close: lock screen"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: Please run with sudo"
    echo "Usage: sudo ./setup-lid-delayed-shutdown.sh"
    exit 1
fi

# Detect AC power supply name
AC_SUPPLY=""
for supply in /sys/class/power_supply/*/type; do
    if [ -f "$supply" ] && grep -q "Mains" "$supply" 2>/dev/null; then
        AC_SUPPLY=$(dirname "$supply")
        AC_SUPPLY=$(basename "$AC_SUPPLY")
        break
    fi
done

if [ -z "$AC_SUPPLY" ]; then
    echo "Warning: Could not detect AC power supply, defaulting to ACAD"
    AC_SUPPLY="ACAD"
fi

echo "[1/5] Detected AC power supply: $AC_SUPPLY"

# Create the daemon script
echo "[2/5] Creating daemon script..."
cat > /usr/local/bin/lid-delayed-shutdown.sh << 'DAEMON_EOF'
#!/bin/bash
#
# Lid Delayed Shutdown Daemon
# Monitors lid state and shuts down after configured delay on battery
#

CONFIG_FILE="/etc/lid-delayed-shutdown.conf"
DEFAULT_DELAY_SECONDS=900  # 15 minutes
POLL_INTERVAL=10           # Check every 10 seconds
AC_SUPPLY="__AC_SUPPLY__"

shutdown_scheduled=false
shutdown_time=0
prev_ac_state=""
prev_lid_state=""
current_delay=0

log_msg() {
    logger -t "lid-delayed-shutdown" "$1"
}

# Read delay from config file (can change dynamically)
get_delay_seconds() {
    if [ -f "$CONFIG_FILE" ]; then
        local delay=$(grep -E "^DELAY_SECONDS=" "$CONFIG_FILE" 2>/dev/null | cut -d'=' -f2 | tr -d ' ')
        if [[ "$delay" =~ ^[0-9]+$ ]] && [ "$delay" -gt 0 ]; then
            echo "$delay"
            return
        fi
    fi
    echo "$DEFAULT_DELAY_SECONDS"
}

get_lid_state() {
    if [ -f /proc/acpi/button/lid/LID0/state ]; then
        cat /proc/acpi/button/lid/LID0/state | awk '{print $2}'
    else
        echo "open"  # Default to open if can't read
    fi
}

get_ac_state() {
    local ac_file="/sys/class/power_supply/${AC_SUPPLY}/online"
    if [ -f "$ac_file" ]; then
        cat "$ac_file"
    else
        echo "1"  # Default to AC connected if can't read
    fi
}

current_delay=$(get_delay_seconds)
log_msg "Daemon started (delay=${current_delay}s, poll=${POLL_INTERVAL}s, ac=${AC_SUPPLY})"

while true; do
    lid_state=$(get_lid_state)
    ac_online=$(get_ac_state)

    # Log power state changes (once per transition)
    if [[ "$prev_ac_state" != "" && "$prev_ac_state" != "$ac_online" ]]; then
        if [[ "$ac_online" == "1" ]]; then
            log_msg "Power state changed: now on AC power"
        else
            log_msg "Power state changed: now on battery"
        fi
    fi
    prev_ac_state="$ac_online"

    # Log lid state changes (once per transition)
    if [[ "$prev_lid_state" != "" && "$prev_lid_state" != "$lid_state" ]]; then
        log_msg "Lid state changed: now ${lid_state}"
    fi
    prev_lid_state="$lid_state"

    if [[ "$lid_state" == "closed" && "$ac_online" == "0" ]]; then
        # Lid closed + on battery
        if [[ "$shutdown_scheduled" == false ]]; then
            # Read delay fresh when scheduling (picks up config changes)
            current_delay=$(get_delay_seconds)
            shutdown_time=$(($(date +%s) + current_delay))
            shutdown_scheduled=true
            remaining=$((current_delay / 60))
            remaining_sec=$((current_delay % 60))
            if [ "$remaining" -gt 0 ]; then
                log_msg "Lid closed on battery - shutdown scheduled in ${remaining}m ${remaining_sec}s"
            else
                log_msg "Lid closed on battery - shutdown scheduled in ${current_delay}s"
            fi
        else
            now=$(date +%s)
            if [[ $now -ge $shutdown_time ]]; then
                log_msg "Timer elapsed - initiating shutdown"
                systemctl poweroff
                exit 0
            fi
        fi
    else
        # Lid open OR on AC power - cancel any scheduled shutdown
        if [[ "$shutdown_scheduled" == true ]]; then
            if [[ "$lid_state" == "open" ]]; then
                log_msg "Shutdown cancelled - lid opened"
            else
                log_msg "Shutdown cancelled - AC power connected"
            fi
            shutdown_scheduled=false
        fi
    fi

    sleep $POLL_INTERVAL
done
DAEMON_EOF

# Replace placeholder with actual AC supply name
sed -i "s/__AC_SUPPLY__/${AC_SUPPLY}/g" /usr/local/bin/lid-delayed-shutdown.sh
chmod +x /usr/local/bin/lid-delayed-shutdown.sh

# Create config file
echo "[2.5/5] Creating config file..."
cat > /etc/lid-delayed-shutdown.conf << 'CONFIG_EOF'
# Lid Delayed Shutdown Configuration
#
# Change DELAY_SECONDS and the daemon will pick it up on next lid close.
# No need to restart the service!
#
# Examples:
#   DELAY_SECONDS=60    # 1 minute (for testing)
#   DELAY_SECONDS=300   # 5 minutes
#   DELAY_SECONDS=900   # 15 minutes (default)
#   DELAY_SECONDS=1800  # 30 minutes

DELAY_SECONDS=900
CONFIG_EOF

# Create helper command to change delay easily
cat > /usr/local/bin/lid-shutdown-delay << 'HELPER_EOF'
#!/bin/bash
#
# Helper to view/change the lid shutdown delay
# Usage: lid-shutdown-delay [seconds|minutes with 'm' suffix]
#
# Examples:
#   lid-shutdown-delay         # Show current delay
#   lid-shutdown-delay 60      # Set to 60 seconds
#   lid-shutdown-delay 5m      # Set to 5 minutes
#   lid-shutdown-delay 15m     # Set to 15 minutes
#

CONFIG_FILE="/etc/lid-delayed-shutdown.conf"

get_current() {
    if [ -f "$CONFIG_FILE" ]; then
        grep -E "^DELAY_SECONDS=" "$CONFIG_FILE" 2>/dev/null | cut -d'=' -f2 | tr -d ' '
    else
        echo "900"
    fi
}

if [ -z "$1" ]; then
    # Show current delay
    current=$(get_current)
    minutes=$((current / 60))
    seconds=$((current % 60))
    echo "Current delay: ${current} seconds (${minutes}m ${seconds}s)"
    echo ""
    echo "Usage: lid-shutdown-delay [seconds|minutes with 'm' suffix]"
    echo "Examples:"
    echo "  sudo lid-shutdown-delay 60   # 60 seconds"
    echo "  sudo lid-shutdown-delay 5m   # 5 minutes"
    exit 0
fi

# Check for root
if [ "$EUID" -ne 0 ]; then
    echo "Error: Please run with sudo to change delay"
    exit 1
fi

# Parse input
input="$1"
if [[ "$input" =~ ^([0-9]+)m$ ]]; then
    # Minutes format (e.g., "5m")
    delay=$((${BASH_REMATCH[1]} * 60))
elif [[ "$input" =~ ^[0-9]+$ ]]; then
    # Seconds format
    delay="$input"
else
    echo "Error: Invalid format. Use seconds (e.g., 60) or minutes (e.g., 5m)"
    exit 1
fi

if [ "$delay" -lt 10 ]; then
    echo "Error: Delay must be at least 10 seconds"
    exit 1
fi

# Update config file
sed -i "s/^DELAY_SECONDS=.*/DELAY_SECONDS=${delay}/" "$CONFIG_FILE"

minutes=$((delay / 60))
seconds=$((delay % 60))
echo "Delay set to: ${delay} seconds (${minutes}m ${seconds}s)"
echo "Change will take effect on next lid close."
HELPER_EOF
chmod +x /usr/local/bin/lid-shutdown-delay

# Create systemd service
echo "[3/5] Creating systemd service..."
cat > /etc/systemd/system/lid-delayed-shutdown.service << 'SERVICE_EOF'
[Unit]
Description=Delayed shutdown on lid close (battery only)
Documentation=file:///home/rajesh/.claude/plans/dapper-hopping-pike.md
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/lid-delayed-shutdown.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICE_EOF

# Update logind configuration
echo "[4/5] Updating logind configuration..."
mkdir -p /etc/systemd/logind.conf.d/
cat > /etc/systemd/logind.conf.d/lid-shutdown.conf << 'LOGIND_EOF'
[Login]
# On battery: ignore (custom script handles delayed shutdown)
HandleLidSwitch=ignore

# On AC power: lock screen immediately
HandleLidSwitchExternalPower=lock
LOGIND_EOF

# Enable and start services
echo "[5/5] Enabling and starting services..."
systemctl daemon-reload
systemctl enable lid-delayed-shutdown.service
systemctl restart lid-delayed-shutdown.service

# Reload logind (using SIGHUP to avoid session disruption)
systemctl kill -s HUP systemd-logind

echo ""
echo "=============================================="
echo " Setup Complete!"
echo "=============================================="
echo ""
echo "Current configuration:"
echo "  • On battery + lid close: shutdown after 15 minutes"
echo "  • On AC power + lid close: lock screen"
echo ""
echo "Service status:"
systemctl is-active lid-delayed-shutdown.service && echo "  ✓ Daemon is running" || echo "  ✗ Daemon failed to start"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Change Delay (no restart needed!)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  lid-shutdown-delay           # Show current delay"
echo "  sudo lid-shutdown-delay 60   # Set to 60 seconds (testing)"
echo "  sudo lid-shutdown-delay 5m   # Set to 5 minutes"
echo "  sudo lid-shutdown-delay 15m  # Set to 15 minutes (default)"
echo ""
echo "Or edit directly: /etc/lid-delayed-shutdown.conf"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "To check logs:"
echo "  journalctl -t lid-delayed-shutdown -f"
echo ""
echo "To uninstall:"
echo "  sudo systemctl stop lid-delayed-shutdown.service"
echo "  sudo systemctl disable lid-delayed-shutdown.service"
echo "  sudo rm /usr/local/bin/lid-delayed-shutdown.sh"
echo "  sudo rm /usr/local/bin/lid-shutdown-delay"
echo "  sudo rm /etc/lid-delayed-shutdown.conf"
echo "  sudo rm /etc/systemd/system/lid-delayed-shutdown.service"
echo "  sudo rm /etc/systemd/logind.conf.d/lid-shutdown.conf"
echo "  sudo systemctl daemon-reload"
echo "  sudo systemctl kill -s HUP systemd-logind"
echo ""

#!/bin/bash

# Function to get Git repository metadata
get_git_metadata() {
    local repo_path="$(git rev-parse --show-toplevel 2>/dev/null)"
    if [ -n "$repo_path" ]; then
        local branch="$(git rev-parse --abbrev-ref HEAD)"
        local commit="$(git rev-parse HEAD)"
        echo "Git Repository: $repo_path"
        echo "Branch: $branch"
        echo "Commit: $commit"
    fi
}

# Function to check if a serial device is connected
is_device_connected() {
    if [ -e "/dev/tty.usbmodem101" ] || [ -e "/dev/tty.usbmodem1101" ]; then
        return 0  # Device is connected
    else
        return 1  # Device is not connected
    fi
}

# Function to connect to the serial device using screen and log data
connect_to_device() {
    local log_file="serial_log1_$(date +'%Y%m%d_%H%M%S').txt"
    
    # Log Git metadata to the file
    get_git_metadata >> "$log_file"
    
    # Connect to the serial device using screen and log data
    screen -L -Logfile "$log_file" "/dev/tty.usbmodem101" || screen -L -Logfile "$log_file" "/dev/tty.usbmodem1101"
}

# Main loop
while true; do
    if is_device_connected; then
        echo "Serial device detected. Connecting..."
        connect_to_device
        echo "Connection closed. Waiting for device..."
    else
        echo "Serial device not detected. Waiting..."
    fi
    sleep 1  # Adjust the sleep duration as needed
done

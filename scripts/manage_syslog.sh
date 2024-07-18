#!/bin/bash

# Path to the syslog file
SYSLOG_FILE="/var/log/syslog"
SYSLOG_OLD_FILE="/var/log/syslog_old.txt"

while true; do
    # Check if syslog.txt is larger than 20GB (21474836480 bytes)
    if [ $(stat -c %s "$SYSLOG_FILE") -gt 21474836480 ]; then
        # Copy the content of syslog.txt to syslog_old.txt
        cp "$SYSLOG_FILE" "$SYSLOG_OLD_FILE"
        # Empty syslog.txt
        > "$SYSLOG_FILE"
    fi
    # Wait for a specified time interval before checking again
    sleep 60 # Sleep for 60 seconds
done

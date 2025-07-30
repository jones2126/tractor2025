#!/bin/bash

# Script to set up UFW rules on a Raspberry Pi 3 for robotics and ZeroTier VPN
# This script should be run with sudo (e.g., sudo ./setup_ufw_rules.sh)
# Last updated: April 25, 2025
# may need to make executable: $ chmod +x setup_ufw_rules.sh
# or $ chmod +x /home/al/python/BridgevilleRTKBase/setup_ufw_rules.sh
# copy to home directory: $ mv /home/al/python/BridgevilleRTKBase/setup_ufw_rules.sh /home/al/setup_ufw_rules.sh
# run using: $ sudo setup_ufw_rules.sh

# Check if the script is running with root privileges
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root (use sudo)."
    exit 1
fi

# Ensure UFW is installed
if ! command -v ufw &> /dev/null; then
    echo "UFW is not installed. Installing UFW..."
    apt update && apt install -y ufw
fi

# Reset UFW to a clean state (WARNING: This will remove existing rules)
ufw --force reset

# Set default policies
# Deny all incoming traffic by default, allow all outgoing traffic
ufw default deny incoming
ufw default allow outgoing

# Rule 1: Allow TCP port 6001 for RTK GPS RTCM correction data (IPv4)
# This port is used for communication between the Base and Rover in your robotics setup
ufw allow 6001/tcp comment 'Allow RTK GPS RTCM correction data (IPv4)'

# Rule 2: Allow TCP port 22 for SSH access (IPv4)
# This ensures you can remotely access the Raspberry Pi via SSH
ufw allow 22/tcp comment 'Allow SSH access (IPv4)'

# Rule 3: Allow UDP port 9993 for ZeroTier VPN (IPv4)
# This is ZeroTier's primary port for peer-to-peer communication
ufw allow 9993/udp comment 'Allow ZeroTier primary port (IPv4)'

# Rule 4: Allow UDP ports 30000-60000 for ZeroTier VPN (IPv4)
# This range includes ZeroTier's secondary port (37122) and other potential fallback ports
ufw allow 30000:60000/udp comment 'Allow ZeroTier secondary/fallback ports (IPv4)'

# Rule 5: Allow UDP port 65288 for ZeroTier VPN (IPv4)
# This is ZeroTier's tertiary port for peer-to-peer communication
ufw allow 65288/udp comment 'Allow ZeroTier tertiary port (IPv4)'

# Rule 6: Allow IGMP traffic for ZeroTier multicast (IPv4)
# ZeroTier uses multicast for network discovery (e.g., 224.0.0.1); this allows IGMP packets
ufw allow in to any proto igmp comment 'Allow ZeroTier multicast discovery (IPv4)'

# Rule 7: Allow TCP port 6001 for RTK GPS RTCM correction data (IPv6)
# Same as Rule 1, but for IPv6
ufw allow 6001/tcp comment 'Allow RTK GPS RTCM correction data (IPv6)'

# Rule 8: Allow TCP port 22 for SSH access (IPv6)
# Same as Rule 2, but for IPv6
ufw allow 22/tcp comment 'Allow SSH access (IPv6)'

# Rule 9: Allow UDP port 9993 for ZeroTier VPN (IPv6)
# Same as Rule 3, but for IPv6
ufw allow 9993/udp comment 'Allow ZeroTier primary port (IPv6)'

# Rule 10: Allow UDP ports 30000-60000 for ZeroTier VPN (IPv6)
# Same as Rule 4, but for IPv6
ufw allow 30000:60000/udp comment 'Allow ZeroTier secondary/fallback ports (IPv6)'

# Rule 11: Allow UDP port 65288 for ZeroTier VPN (IPv6)
# Same as Rule 5, but for IPv6
ufw allow 65288/udp comment 'Allow ZeroTier tertiary port (IPv6)'

# Enable UFW
ufw enable

# Display the final rules
echo "UFW rules have been set up. Current status:"
ufw status numbered

echo "Setup complete! Verify your SSH access and ZeroTier connectivity."
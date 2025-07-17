#!/usr/bin/env python3

import subprocess
import sys

# sevillabot's MAC address (from `ip link` on the Pi)
ROBOT_MAC = "D8:3A:DD:64:AA:35".lower()

def find_robot_ip(mac_address):
    try:
        print("Scanning network for sevillabot...")
        result = subprocess.check_output(["sudo", "arp-scan", "--localnet"]).decode()
    except Exception as e:
        print(f"Error running arp-scan: {e}")
        sys.exit(1)

    for line in result.splitlines():
        if mac_address in line.lower():
            ip = line.split()[0]
            print(f"✅ Found sevillabot at IP: {ip}")
            return ip

    print("❌ sevillabot not found.")
    return None

if __name__ == "__main__":
    find_robot_ip(ROBOT_MAC)

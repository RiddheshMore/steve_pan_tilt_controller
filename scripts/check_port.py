#!/usr/bin/env python3
"""
Diagnostic tool for checking Dynamixel serial port connection.
Run this before launching the pan-tilt controller to check for port issues.
"""

import os
import sys
import subprocess
import time

def check_port_permissions(device):
    """Check if current user has permission to access the device."""
    if not os.path.exists(device):
        return False, f"Device {device} does not exist!"
    
    # Check if readable and writable
    if os.access(device, os.R_OK | os.W_OK):
        return True, f"✓ {device} is accessible"
    else:
        return False, f"✗ {device} exists but no read/write permission. Run: sudo chmod 666 {device}"

def check_processes_using_port(device):
    """Check if any processes are using the serial port."""
    try:
        result = subprocess.run(['lsof', device], capture_output=True, text=True)
        if result.returncode == 0:
            return False, f"✗ Port is in use:\n{result.stdout}"
        else:
            return True, "✓ No processes using the port"
    except FileNotFoundError:
        return None, "⚠ 'lsof' not found - cannot check port usage. Install with: sudo apt install lsof"

def list_usb_serial_devices():
    """List all USB serial devices."""
    devices = []
    for device in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyACM0', '/dev/ttyACM1']:
        if os.path.exists(device):
            devices.append(device)
    
    if devices:
        return True, f"✓ Found USB serial devices: {', '.join(devices)}"
    else:
        return False, "✗ No USB serial devices found. Check USB connection!"

def check_user_in_dialout():
    """Check if current user is in dialout group."""
    import grp
    try:
        dialout = grp.getgrnam('dialout')
        current_user = os.getenv('USER')
        if current_user in dialout.gr_mem:
            return True, f"✓ User {current_user} is in 'dialout' group"
        else:
            return False, f"✗ User {current_user} is NOT in 'dialout' group. Add with: sudo usermod -a -G dialout {current_user}"
    except KeyError:
        return None, "⚠ 'dialout' group not found"

def main():
    print("=" * 60)
    print("Dynamixel Serial Port Diagnostic Tool")
    print("=" * 60)
    
    # Default device - can be changed via command line
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    print(f"\nChecking device: {device}\n")
    
    # Run diagnostics
    checks = [
        ("USB Devices", list_usb_serial_devices()),
        ("Port Permissions", check_port_permissions(device)),
        ("Port Usage", check_processes_using_port(device)),
        ("User Groups", check_user_in_dialout()),
    ]
    
    print("-" * 60)
    errors = []
    for name, (status, message) in checks:
        print(f"{name}:")
        print(f"  {message}")
        if status == False:
            errors.append(message)
        print()
    
    print("-" * 60)
    
    if errors:
        print("\n❌ ISSUES DETECTED:")
        for i, error in enumerate(errors, 1):
            print(f"{i}. {error}")
        print("\nFix these issues before running the pan-tilt controller.")
        return 1
    else:
        print("\n✅ All checks passed! Port should be ready to use.")
        return 0

if __name__ == '__main__':
    sys.exit(main())

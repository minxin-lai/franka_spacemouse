# Debugging SpaceMouse Connection Issues

## Problem Description
The `spacemouse_publisher` node may fail to start with the error:
```
easyhid.easyhid.HIDException: Failed to open device
```
This typically happens because the `device_path` in `config/fr3_duo_config.yaml` points to a `/dev/hidrawX` path that no longer exists or belongs to a different device after a reboot or USB reconnection.

## Solution: Persistent Udev Rules
To fix this permanently, we use `udev` rules to assign fixed symbolic links to the SpaceMouse devices based on their physical USB ports.

### 1. Identify Physical Ports
Use `udevadm` to find the unique KERNELS path for your USB ports:
```bash
udevadm info -a -n /dev/hidrawX | grep KERNELS
```
In this setup (SzaiLab):
- **Left SpaceMouse**: Port `3-6.1`
- **Right SpaceMouse**: Port `3-6.4`

### 2. Create Udev Rules
File: `99-spacemouse.rules`
```udev
# SpaceMouse Persistent Symlinks
# Left SpaceMouse (Port 3-6.1)
SUBSYSTEM=="hidraw", KERNELS=="3-6.1:1.0", SYMLINK+="spacemouse_left"
# Right SpaceMouse (Port 3-6.4)
SUBSYSTEM=="hidraw", KERNELS=="3-6.4:1.0", SYMLINK+="spacemouse_right"
```

### 3. Install Rules
```bash
sudo cp 99-spacemouse.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Verify
```bash
ls -l /dev/spacemouse*
```
You should see `spacemouse_left` and `spacemouse_right` pointing to valid `hidraw` devices.

### 5. Update Config
Ensure `config/fr3_duo_config.yaml` uses these symlinks:
```yaml
LEFT:
  device_path: /dev/spacemouse_left
RIGHT:
  device_path: /dev/spacemouse_right
```

## Debugging Tools
A script `debug_spacemouse.py` is included to help diagnose connection issues by attempting to open specific device paths using `easyhid` directly.
```bash
python3 debug_spacemouse.py /dev/hidrawX
```

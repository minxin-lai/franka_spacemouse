
import sys
import os
from easyhid import Enumeration

def check_device(path):
    print(f"Checking access to {path}...")
    if not os.path.exists(path):
        print(f"ERROR: {path} does not exist!")
        return
        
    try:
        fd = os.open(path, os.O_RDWR)
        print(f"SUCCESS: Opened {path} with os.open")
        os.close(fd)
    except Exception as e:
        print(f"ERROR: Failed to os.open {path}: {e}")

    try:
        # Mimic pyspacemouse open logic roughly
        hid = Enumeration()
        devices = hid.find()
        match = None
        for dev in devices:
            # We just grab the first one to abuse it like pyspacemouse might, 
            # OR we try to find one that naturally matches if possible.
            # But pyspacemouse logic forces path.
            pass
            
        if not devices:
            print("No HID devices found by Enumeration()")
            return

        # Pick a device - pyspacemouse grabs the first matching supported one.
        # We will just take the first one and try to open it with the forced path
        # to see if that's where the error lies.
        target_dev = devices[0] 
        print(f"Attempting to open path '{path}' via easyhid (using device object for {target_dev.product_string})...")
        
        target_dev.path = path
        target_dev.open()
        print(f"SUCCESS: easyhid opened {path}")
        target_dev.close()
        
    except Exception as e:
        print(f"ERROR: easyhid failed to open {path}: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 debug_spacemouse.py /dev/hidrawX [/dev/hidrawY ...]")
        sys.exit(1)
        
    for p in sys.argv[1:]:
        check_device(p)

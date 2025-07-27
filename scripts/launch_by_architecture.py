#!/usr/bin/env python3

import subprocess
import sys
import os

def get_raspberry_pi_model():
    """Detect Raspberry Pi model and return 'cm4' or 'pi5'"""
    try:
        # Read the model from /proc/device-tree/model
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()
        
        if 'Compute Module 4' in model:
            return 'cm4'
        elif 'Raspberry Pi 5' in model:
            return 'pi5'
        else:
            # Fallback: check for CM4 specific files
            if os.path.exists('/proc/device-tree/soc/gpio@7e200000/compatible'):
                return 'cm4'
            else:
                return 'pi5'
    except Exception as e:
        print(f"Error detecting Raspberry Pi model: {e}")
        return None

def main():
    # Get the architecture
    architecture = get_raspberry_pi_model()
    
    if architecture is None:
        print("Could not determine Raspberry Pi architecture. Using default (pi5)")
        architecture = 'pi5'
    
    print(f"Detected architecture: {architecture}")
    
    # Determine which launch file to use
    if architecture == 'cm4':
        launch_file = 'dexi_bringup_cm4.launch.py'
        print("Using CM4 launch file with camera_ros for CSI camera")
    else:
        launch_file = 'dexi_bringup_pi5.launch.py'
        print("Using Pi5 launch file with dexi_camera for UVC camera")
    
    # Build the ros2 launch command
    cmd = ['ros2', 'launch', 'dexi_bringup', launch_file] + sys.argv[1:]
    
    print(f"Executing: {' '.join(cmd)}")
    
    # Execute the launch command
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Launch failed with exit code: {e.returncode}")
        sys.exit(e.returncode)
    except KeyboardInterrupt:
        print("\nLaunch interrupted by user")
        sys.exit(0)

if __name__ == '__main__':
    main() 
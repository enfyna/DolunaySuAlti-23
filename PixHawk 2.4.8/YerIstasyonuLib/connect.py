
import subprocess

# IP address of the Raspberry Pi
raspberry_pi_ip = "dolunay-desktop"

# Name of the Python script you want to run on the Raspberry Pi
script_name = "ssh_camera.py"

# Password for the 'pi' user on the Raspberry Pi
password = "12345"

# Command to run the Python script on the Raspberry Pi
command = f"ssh dolunay@{raspberry_pi_ip} sudo python Desktop/{script_name}"

# Run the command
subprocess.run(command, shell=True)

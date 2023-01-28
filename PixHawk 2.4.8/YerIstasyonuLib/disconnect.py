
import subprocess

raspberry_pi_ip = "raspberrypi"
password = "123456"
command = f"ssh dolunay@{raspberry_pi_ip} 'sudo pkill python'"
subprocess.run(command, shell=True)
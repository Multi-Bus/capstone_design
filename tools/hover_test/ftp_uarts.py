import sys
from pymavlink import mavutil
m = mavutil.mavlink_connection("COM37", baud=115200); m.wait_heartbeat(timeout=10)
from pymavlink.mavftp import MAVFTP
f = MAVFTP(m, m.target_system, m.target_component)
f.cmd_get(["@SYS/uarts.txt", "uarts4.txt"])
f.process_ftp_reply("OpenFileRO", timeout=15)
print(open("uarts4.txt").read() if __import__("os").path.exists("uarts4.txt") else "no file")

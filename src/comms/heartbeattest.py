from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
print(m.recv_match(type="HEARTBEAT", blocking=True, timeout=5))


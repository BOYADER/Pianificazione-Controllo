if not time_start:
			time_start = time.time()

dt = time.time() - time_start
yaw_ref = eta_2[2] + math.degrees(yaw_angular_velocity)*dt

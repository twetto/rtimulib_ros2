# Emitted once per hardware trigger pulse sent to the camera.
#
# header.stamp is taken immediately after the active edge is driven, so it is
# the closest available estimate of the trigger instant. The two counters let a
# recorded bag be replayed back into IMU-sample / trigger / frame correspondence
# without relying on message ordering alone.

std_msgs/Header header

uint64 trigger_count      # triggers emitted since node start, starting at 1
uint64 imu_sample_count   # index of the IMU sample that produced this trigger
uint32 pulse_us           # commanded pulse width in microseconds

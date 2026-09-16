# tool: make_profile.py
# category: process
# purpose: Generate a sampled J4-only trapezoidal or triangular buffered program without commanding the robot.
# usage: python3 knowledge/tools/make_profile.py OUTPUT --accel RAD_S2 --ramp SECONDS [--cruise SECONDS] [--hold SECONDS] [--open SECONDS --fraction FRACTION]    (run from the session directory)
# inputs/outputs: Reads numeric CLI arguments; writes program JSON to OUTPUT and prints duration, displacement, and peak target velocity.
# assumptions: 0.02 s sampling; durations must be nonnegative multiples of 0.02 s with positive ramp; default hold 0.4 s; default event fraction 1; caller must preview for limits and clearance.
# verified: used successfully in the session that wrote it
# Generates one-axis buffered profiles; does not contact the robot.
import argparse,json,math
p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--accel',type=float,required=True);p.add_argument('--ramp',type=float,required=True);p.add_argument('--cruise',type=float,default=0);p.add_argument('--hold',type=float,default=.4);p.add_argument('--open',type=float);p.add_argument('--fraction',type=float,default=1)
a=p.parse_args(); T=2*a.ramp+a.cruise; times=[round(i*.02,8) for i in range(round((T+a.hold)/.02)+1)]
def pos(t):
 if t<=a.ramp:return .5*a.accel*t*t
 if t<=a.ramp+a.cruise:return .5*a.accel*a.ramp**2+a.accel*a.ramp*(t-a.ramp)
 if t<=T:
  u=t-a.ramp-a.cruise
  return .5*a.accel*a.ramp**2+a.accel*a.ramp*a.cruise+a.accel*a.ramp*u-.5*a.accel*u*u
 return a.accel*a.ramp*(a.ramp+a.cruise)
program={'times_s':times,'joint_deltas_rad':[[0,0,0,pos(t),0,0] for t in times],'gripper_events':[] if a.open is None else [{'time_s':a.open,'fraction':a.fraction}]}
with open(a.output,'w') as f:json.dump({'program':program},f)
print('duration',T,'delta',pos(T),'peak velocity',a.accel*a.ramp)

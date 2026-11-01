"""Real IK + command limiter, with a simulated joint plant and no DDS writer."""
import time
import numpy as np
import pinocchio as pin
from test_arm_following import arm, ik_module

ik = ik_module.G1_29_ArmIK(Visualization=False)
measured = np.array([0.2945, .222, -.0066, .9875, .1277, .0072, .0102,
                     .2948, -.2089, .0056, .9819, -.1884, -.0077, .0116])
desired = measured.copy()
desired[0] += .5
desired[7] -= .5
pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, desired)
left = ik.reduced_robot.data.oMf[ik.L_hand_id].homogeneous.copy()
right = ik.reduced_robot.data.oMf[ik.R_hand_id].homogeneous.copy()
c = arm.G1_29_ArmController.__new__(arm.G1_29_ArmController)
c.get_current_dual_arm_q = lambda: measured.copy()
c.arm_velocity_limit = 3.
c.arm_accel_limit = 10.
c.arm_tracking_error_limit = .15
c._trajectory = None
c._last_delta = None
c._dt_ema = .004
dt = .004
next_ik = 0.
timings = []
command_timings = []
target = measured.copy()
for i in range(500):
    now = i * dt
    if now + 1e-9 >= next_ik:
        begin = time.perf_counter()
        target, _ = ik.solve_ik(left, right, measured.copy(), np.zeros(14))
        timings.append((time.perf_counter() - begin) * 1000)
        next_ik += 1 / 30
    begin = time.perf_counter()
    command = c.clip_arm_q_target(target, dt=dt)
    command_timings.append((time.perf_counter() - begin) * 1000)
    assert np.max(np.abs(command - measured)) <= .15 + 1e-8
    measured += (command - measured) * .1

pin.framesForwardKinematics(ik.reduced_robot.model, ik.reduced_robot.data, measured)
left_error = np.linalg.norm(ik.reduced_robot.data.oMf[ik.L_hand_id].translation - left[:3, 3])
right_error = np.linalg.norm(ik.reduced_robot.data.oMf[ik.R_hand_id].translation - right[:3, 3])
timings = timings[3:]
print("REAL_IK_AND_LIMITER: 2 simulated seconds, 30Hz IK, 250Hz commands, 40ms joint response")
print("wrist_error_m: left=%.5f right=%.5f" % (left_error, right_error))
print("IK_ms: p50=%.3f p95=%.3f max=%.3f" % (np.median(timings), np.percentile(timings, 95), max(timings)))
print("limiter_ms: p50=%.3f p95=%.3f" % (np.median(command_timings), np.percentile(command_timings, 95)))
assert max(left_error, right_error) < .03, "wrist target must be followed within 3cm"
print("PASS: coupled following, bounded tracking error; no robot publisher was constructed")

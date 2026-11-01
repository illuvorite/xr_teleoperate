"""Offline regression tests: never construct a DDS publisher or a robot controller."""
import importlib.util
import os
from pathlib import Path
import sys
import threading
import types
import unittest

import numpy as np

PROJECT = Path(os.environ.get("XR_TEST_PROJECT", "/home/unitree/xr_teleoperate"))
CODE = Path(os.environ.get("XR_TEST_CODE_DIR", str(PROJECT / "teleop/robot_control")))
sys.path.insert(0, str(PROJECT))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


if (CODE / "arm_trajectory.py").exists():
    load("teleop.robot_control.arm_trajectory", CODE / "arm_trajectory.py")
if (CODE / "weighted_moving_filter.py").exists():
    load("teleop.utils.weighted_moving_filter", CODE / "weighted_moving_filter.py")
arm = load("xr_test_arm", CODE / "robot_arm.py")
ik_module = load("xr_test_ik", CODE / "robot_arm_ik.py")


def controller():
    c = arm.G1_29_ArmController.__new__(arm.G1_29_ArmController)
    state = np.zeros(14)
    c.get_current_dual_arm_q = lambda: state.copy()
    c.arm_velocity_limit = 3.0
    c.arm_accel_limit = 10.0
    c.arm_tracking_error_limit = 0.15
    c.control_dt = c._dt_ema = 0.004
    c._last_delta = None
    c._trajectory = None
    return c, state


class FollowingTests(unittest.TestCase):
    def test_one_radian_step_tracks_with_forty_ms_joint_response(self):
        c, measured = controller()
        target = np.zeros(14)
        target[0] = 1.0
        for _ in range(250):
            command = c.clip_arm_q_target(target, dt=0.004)
            measured += (command - measured) * 0.1
        self.assertGreaterEqual(measured[0], 0.95, "1rad target must be reached within 1s, including joint lag")

    def test_starting_acceleration_and_speed_are_bounded(self):
        c, measured = controller()
        target = np.zeros(14)
        target[0] = 1.0
        previous_velocity = np.zeros(14)
        for _ in range(100):
            command = c.clip_arm_q_target(target, dt=0.004)
            velocity = (command - measured) / 0.004
            self.assertLessEqual(np.max(np.abs(velocity)), 3.0 + 1e-8)
            self.assertLessEqual(np.max(np.abs(velocity - previous_velocity)), 10.0 * 0.004 + 1e-8)
            previous_velocity = velocity
            measured[:] = command

    def test_period_changes_preserve_velocity_and_acceleration_units(self):
        c, measured = controller()
        target = np.ones(14) * 1.0
        previous_velocity = np.zeros(14)
        for dt in [0.004, 0.008, 0.003, 0.006] * 15:
            command = c.clip_arm_q_target(target, dt=dt)
            velocity = (command - measured) / dt
            self.assertLessEqual(np.max(np.abs(velocity)), 3.0 + 1e-8)
            self.assertLessEqual(np.max(np.abs(velocity - previous_velocity)), 10.0 * dt + 1e-8)
            previous_velocity = velocity
            measured[:] = command

    def test_stationary_joint_does_not_allow_command_windup(self):
        c, measured = controller()
        target = np.ones(14)
        for _ in range(500):
            command = c.clip_arm_q_target(target, dt=0.004)
            self.assertLessEqual(np.max(np.abs(command - measured)), 0.15 + 1e-9)
        self.assertGreater(command[0], 0.05, "command must advance beyond one measured-position step")

    def test_fixed_target_is_reached_without_overshooting(self):
        c, measured = controller()
        target = np.ones(14)
        for _ in range(1000):
            command = c.clip_arm_q_target(target, dt=0.004)
            self.assertLessEqual(np.max(command), 1.0 + 1e-8)
            measured[:] = command
        np.testing.assert_allclose(measured, target, atol=1e-6)

    def test_reversal_preserves_acceleration_limit(self):
        c, measured = controller()
        previous_velocity = np.zeros(14)
        for i in range(400):
            target = np.ones(14) * (1.0 if i < 75 else -1.0)
            command = c.clip_arm_q_target(target, dt=0.004)
            velocity = (command - measured) / 0.004
            self.assertLessEqual(np.max(np.abs(velocity)), 3.0 + 1e-8)
            self.assertLessEqual(np.max(np.abs(velocity - previous_velocity)), 10.0 * 0.004 + 1e-8)
            previous_velocity = velocity
            measured[:] = command
        np.testing.assert_allclose(measured, -np.ones(14), atol=1e-6)

    def test_invalid_target_holds_and_does_not_poison_next_command(self):
        c, measured = controller()
        target = np.ones(14)
        for _ in range(25):
            measured[:] = c.clip_arm_q_target(target, dt=0.004)
        held = measured.copy()
        np.testing.assert_allclose(c.clip_arm_q_target(np.full(14, np.nan)), held)
        command = c.clip_arm_q_target(target, dt=0.004)
        self.assertTrue(np.all(np.isfinite(command)))
        self.assertLessEqual(np.max(np.abs(command - held)), 10.0 * 0.004**2 + 1e-9)

    def test_dds_staleness_freezes_last_published_command(self):
        c, _ = controller()
        c.motion_mode = False
        c.simulation_mode = False
        c._speed_gradual_max = False
        c._release_motion_weight = False
        c.ctrl_lock = threading.Lock()
        c.q_target = np.ones(14)
        c.tauff_target = np.zeros(14)
        c.msg = types.SimpleNamespace(motor_cmd=[types.SimpleNamespace(q=0., dq=0., tau=0.) for _ in range(35)], crc=0)
        c.crc = types.SimpleNamespace(Crc=lambda msg: 0)
        freshness = [True]
        c.lowstate_buffer = types.SimpleNamespace(IsFresh=lambda: freshness[0])
        sent = []

        class Finished(BaseException):
            pass

        def write(msg):
            sent.append(np.array([msg.motor_cmd[i].q for i in arm.G1_29_JointArmIndex]))
            freshness[0] = False
            if len(sent) == 2:
                raise Finished()

        c.lowcmd_publisher = types.SimpleNamespace(Write=write)
        with self.assertRaises(Finished):
            c._ctrl_motor_state()
        np.testing.assert_array_equal(sent[1], sent[0])


class IKTests(unittest.TestCase):
    def make_ik(self):
        ik = ik_module.G1_29_ArmIK.__new__(ik_module.G1_29_ArmIK)
        ik.Visualization = False
        ik.init_data = np.zeros(14)
        ik._ik_step_reference = None
        try:
            ik.smooth_filter = ik_module.WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14, skip_duplicates=False)
        except TypeError:
            ik.smooth_filter = ik_module.WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)
        ik.reduced_robot = types.SimpleNamespace(model=types.SimpleNamespace(nv=14, nq=14, lowerPositionLimit=np.full(14, -3.), upperPositionLimit=np.full(14, 3.)), data=None)
        ik.var_q = object()
        ik.var_q_last = object()
        ik.param_tf_l = object()
        ik.param_tf_r = object()
        ik.opti = types.SimpleNamespace(set_initial=lambda *a: None, set_value=lambda *a: None, solve=lambda: None, value=lambda *a: np.ones(14))
        ik._update_diagnostics = lambda *a, **kw: None
        return ik

    def solve(self, ik):
        original = ik_module.pin.rnea
        ik_module.pin.rnea = lambda *a: np.zeros(14)
        try:
            return ik.solve_ik(np.eye(4), np.eye(4), np.zeros(14), np.zeros(14))[0]
        finally:
            ik_module.pin.rnea = original

    def test_ik_output_advances_despite_measured_joint_lag(self):
        ik = self.make_ik()
        outputs = [self.solve(ik) for _ in range(35)]
        self.assertGreaterEqual(outputs[-1][0], 0.95, "valid IK targets must advance instead of staying 0.1rad from measured q")
        for previous, current in zip(outputs, outputs[1:]):
            self.assertLessEqual(np.max(np.abs(current - previous)), 0.1 + 1e-8)

    def test_ik_failure_resets_previous_solution_and_filter(self):
        ik = self.make_ik()
        for _ in range(15):
            self.solve(ik)
        def fail():
            raise RuntimeError("test solver failure")
        ik.opti.solve = fail
        np.testing.assert_array_equal(self.solve(ik), np.zeros(14))
        ik.opti.solve = lambda: None
        recovered = self.solve(ik)
        self.assertLessEqual(np.max(np.abs(recovered)), 0.1 + 1e-8)

    def test_invalid_xr_pose_clears_pending_ik_motion(self):
        ik = self.make_ik()
        for _ in range(15):
            self.solve(ik)
        held, _ = ik.solve_ik(np.full((4, 4), np.nan), np.eye(4), np.zeros(14), np.zeros(14))
        np.testing.assert_array_equal(held, np.zeros(14))
        self.assertLessEqual(np.max(np.abs(self.solve(ik))), 0.1 + 1e-8)


if __name__ == "__main__":
    unittest.main(verbosity=2)

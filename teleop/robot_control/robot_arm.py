import os
import numpy as np
import threading
import time
from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import ( LowCmd_  as hg_LowCmd, LowState_ as hg_LowState) # idl for g1, h1_2
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.idl.unitree_go.msg.dds_ import ( LowCmd_  as go_LowCmd, LowState_ as go_LowState)  # idl for h1
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

from teleop.robot_control.dds_utils import wait_for_dds
from teleop.robot_control.arm_trajectory import ArmTrajectoryLimiter

kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

G1_29_Num_Motors = 35
G1_23_Num_Motors = 35
H1_2_Num_Motors = 35
H1_Num_Motors = 20

# ---------------------------------------------------------------------------
# 关节运动安全上限
#
# 原实现把 20 -> 30 rad/s 当作"限速"，但 G1 臂展约 0.7 m，20 rad/s 换算到
# 末端接近 14 m/s，30 rad/s 接近 21 m/s —— 这已经不能提供任何保护，反而会在
# XR 抖动或 IK 跳变时把机械臂甩出去。这里按"人臂快速摆动"的量级收窄到
# 1.5 -> 3.0 rad/s，并增加加速度上限，避免速度指令阶跃。
#
# 允许用环境变量覆盖，方便现场按机型/任务微调而不改代码：
#   XR_ARM_VELOCITY_LIMIT_START / XR_ARM_VELOCITY_LIMIT_MAX / XR_ARM_ACCEL_LIMIT
# ---------------------------------------------------------------------------
ARM_VELOCITY_LIMIT_START = float(os.getenv("XR_ARM_VELOCITY_LIMIT_START", "1.5"))
ARM_VELOCITY_LIMIT_MAX = float(os.getenv("XR_ARM_VELOCITY_LIMIT_MAX", "3.0"))
ARM_VELOCITY_LIMIT_SIM = float(os.getenv("XR_ARM_VELOCITY_LIMIT_SIM", "30.0"))
ARM_ACCEL_LIMIT = float(os.getenv("XR_ARM_ACCEL_LIMIT", "10.0"))
ARM_TRACKING_ERROR_LIMIT = float(os.getenv("XR_ARM_TRACKING_ERROR_LIMIT", "0.15"))
# lowstate 超过该时长没有更新，就认为 DDS 状态已陈旧
ARM_LOWSTATE_TIMEOUT = float(os.getenv("XR_ARM_LOWSTATE_TIMEOUT", "0.5"))

_warn_ts = {}
_warn_lock = threading.Lock()


def _warn_throttled(key: str, msg: str, period: float = 1.0) -> None:
    """限流打错误日志，避免 250Hz 控制回路把日志刷爆（进而拖慢控制回路）。"""
    now = time.monotonic()
    with _warn_lock:
        if now - _warn_ts.get(key, 0.0) < period:
            return
        _warn_ts[key] = now
    logger_mp.error(f"[arm-safety] {msg}")

H2_Num_Motors = 35
R1_A5_Num_Motors = 35
R1_A7_Num_Motors = 35
 

class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None

class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]

class G1_23_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_23_Num_Motors)]

class H1_2_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(H1_2_Num_Motors)]

class H1_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(H1_Num_Motors)]

class H2_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(H2_Num_Motors)]

class R1_A5_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(R1_A5_Num_Motors)]

class R1_A7_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(R1_A7_Num_Motors)]


class DataBuffer:
    def __init__(self):
        self.data = None
        self.ts = 0.0
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data
            self.ts = time.monotonic()

    def IsFresh(self, timeout: float = ARM_LOWSTATE_TIMEOUT) -> bool:
        """lowstate 是否在 timeout 内更新过。用于发现 DDS 断连后的陈旧状态。"""
        with self.lock:
            return self.data is not None and (time.monotonic() - self.ts) < timeout

class G1_29_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger_mp.info("Initialize G1_29_ArmController...")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        self.all_motor_q = None
        self.set_arm_velocity_limit(ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_START)
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        self.arm_tracking_error_limit = ARM_TRACKING_ERROR_LIMIT
        self._trajectory = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

        # 渐变限速状态（speed_gradual_max 会置位；发布线程每周期都会读，
        # 必须在 __init__ 里先初始化，否则线程启动后、R 键之前会 AttributeError）
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "G1_29_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        # 关键：用实测关节角初始化目标值。
        # 原实现 q_target = zeros，而发布线程在 __init__ 末尾就启动，结果进程一
        # 启动、用户还没按 [r]，机械臂就会被 250Hz 地拉向全零位（可能穿过躯干）。
        # 此刻 lowstate 已在上面阻塞等待过，数据一定可用；发布线程尚未启动，
        # 因此这里不需要加锁。
        self.q_target = self.get_current_dual_arm_q().copy()
        self.tauff_target = np.zeros_like(self.q_target)
        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        arm_indices = set(member.value for member in G1_29_JointArmIndex)
        for id in G1_29_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize G1_29_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit=None, dt=None):
        '''从上一条指令生成连续轨迹，独立限制速度、加速度及跟踪误差。'''
        if velocity_limit is None:
            velocity_limit = self.arm_velocity_limit
        if dt is None:
            dt = self._dt_ema

        current_q = self.get_current_dual_arm_q()
        target_q = np.asarray(target_q, dtype=float)
        if self._trajectory is None:
            self._trajectory = ArmTrajectoryLimiter(
                self.arm_accel_limit, self.arm_tracking_error_limit
            )
        if not np.all(np.isfinite(current_q)):
            _warn_throttled("feedback_nan", "non-finite feedback, holding last command")
            held = self._trajectory.hold()
            return held if held is not None else np.full_like(target_q, np.nan)

        if target_q.shape != current_q.shape:
            _warn_throttled(
                "clip_shape",
                f"target shape {target_q.shape} != current {current_q.shape}, holding position",
            )
            self._trajectory.reset(current_q)
            return current_q
        if not np.all(np.isfinite(target_q)) or not np.all(np.isfinite(current_q)):
            _warn_throttled("clip_nan", "non-finite joint target rejected, holding position")
            self._trajectory.reset(current_q)
            return current_q

        return self._trajectory.step(target_q, current_q, dt, velocity_limit)

    def _ctrl_motor_state(self):
        if self.motion_mode:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0

        # 成功发布计数：主程序每秒读取，检查实际发布频率。
        self._pub_count = 0
        last_published_q = np.array([self.msg.motor_cmd[i].q for i in G1_29_JointArmIndex])
        last_published_tau = np.array([self.msg.motor_cmd[i].tau for i in G1_29_JointArmIndex])

        last_tick = time.monotonic()
        while True:
            try:
                start_time = time.monotonic()
                dt = max(start_time - last_tick, 1e-4)
                last_tick = start_time
                # 用实测周期做 EMA：调度抖动时仍按真实时间计算限速
                self._dt_ema = 0.9 * self._dt_ema + 0.1 * dt

                with self.ctrl_lock:
                    arm_q_target     = self.q_target
                    arm_tauff_target = self.tauff_target
                    if not self.lowstate_buffer.IsFresh():
                        _warn_throttled("lowstate_stale", "lowstate stale, holding last valid command")
                        cliped_arm_q_target = last_published_q.copy()
                        arm_tauff_target = last_published_tau.copy()
                        if self._trajectory is not None:
                            self._trajectory.reset(last_published_q)
                    elif self._release_motion_weight:
                        cliped_arm_q_target = self.get_current_dual_arm_q()
                        if self._trajectory is not None and np.all(np.isfinite(cliped_arm_q_target)):
                            self._trajectory.reset(cliped_arm_q_target)
                    elif self.simulation_mode:
                        cliped_arm_q_target = arm_q_target
                    else:
                        cliped_arm_q_target = self.clip_arm_q_target(
                            arm_q_target, velocity_limit=self.arm_velocity_limit, dt=dt
                        )

                if (not np.all(np.isfinite(cliped_arm_q_target))
                        or not np.all(np.isfinite(arm_tauff_target))):
                    _warn_throttled("loop_nan", "non-finite command dropped in publish loop")
                    if self._trajectory is not None:
                        self._trajectory.reset(last_published_q)
                else:
                    if self.motion_mode and self._release_motion_weight:
                        weight = self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q
                        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = max(0.0, weight - dt / 2.0)

                    for idx, id in enumerate(G1_29_JointArmIndex):
                        self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                        self.msg.motor_cmd[id].dq = 0
                        self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

                    self.msg.crc = self.crc.Crc(self.msg)
                    if self.lowcmd_publisher.Write(self.msg) is not False:
                        self._pub_count += 1
                        last_published_q = np.asarray(cliped_arm_q_target).copy()
                        last_published_tau = np.asarray(arm_tauff_target).copy()
                    elif self._trajectory is not None:
                        self._trajectory.reset(last_published_q)

                if self._speed_gradual_max is True:
                    t_elapsed = start_time - self._gradual_start_time
                    ratio = min(1.0, t_elapsed / self._gradual_time) if self._gradual_time else 1.0
                    self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START + (
                        (ARM_VELOCITY_LIMIT_MAX - ARM_VELOCITY_LIMIT_START) * ratio
                    )

                all_t_elapsed = time.monotonic() - start_time
                time.sleep(max(0.0, self.control_dt - all_t_elapsed))
            except Exception:
                with self.ctrl_lock:
                    if self._trajectory is not None:
                        self._trajectory.reset(last_published_q)
                logger_mp.exception("[arm] publish loop error, continuing")
                time.sleep(0.01)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.

        非有限值（NaN/Inf）或形状不符的输入在这里被拦下并保留上一帧有效目标。
        上游 XR 追踪丢失时可能产生 NaN，若不拦会一路传到 CRC 与 DDS。
        '''
        q_target = np.asarray(q_target, dtype=float)
        tauff_target = np.asarray(tauff_target, dtype=float)
        if q_target.shape != self.q_target.shape or tauff_target.shape != self.tauff_target.shape:
            _warn_throttled(
                "cmd_shape",
                f"ctrl_dual_arm shape mismatch: q{q_target.shape} tau{tauff_target.shape}",
            )
            return
        if not np.all(np.isfinite(q_target)) or not np.all(np.isfinite(tauff_target)):
            _warn_throttled("cmd_nan", "ctrl_dual_arm rejected non-finite target, keeping previous")
            return
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        '''Return current dds mode machine.'''
        if self.mode_machine is None:
            raise RuntimeError("G1 low state is not ready.")
        return self.mode_machine
    
    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointIndex])
    
    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointArmIndex])
    
    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_29_JointArmIndex])
    
    def ctrl_dual_arm_go_home(self):
        '''退出时保持当前位姿，并平滑卸载 arm_sdk 权重。

        不再把目标置为全零位，原因：
          * 从任意姿态直线插补到零位会穿过躯干/自碰撞区，且没有碰撞检查；
          * 旧的重力前馈会叠加在零位上，形成力矩偏置；
          * 主线程直接写 self.msg 会与发布线程的 CRC 计算竞争。
        现在只做"停在原地 + 由发布线程逐步卸力"，并且超时也会返回，不阻塞退出。
        '''
        logger_mp.info("[G1_29_ArmController] ctrl_dual_arm_go_home: hold current pose and release motion weight...")
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q().copy()
        if not self.motion_mode:
            logger_mp.info("[G1_29_ArmController] debug mode: holding current pose.")
            return
        self._release_motion_weight = True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q <= 0.01:
                break
            time.sleep(0.05)
        logger_mp.info("[G1_29_ArmController] motion weight released, arms hold pose.")

    def speed_gradual_max(self, t = 5.0):
        '''Parameter t is the total time required for arms velocity to gradually increase to its maximum value, in seconds. The default is 5.0.'''
        # 必须与发布循环里的 time.monotonic() 同源，否则会算出巨大的 elapsed
        self._gradual_start_time = time.monotonic()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        '''set arms velocity to the maximum value immediately, instead of gradually increasing.'''
        # 仿真没有物理约束，保留原量级；真机则用收紧后的上限
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_MAX

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            G1_29_JointIndex.kLeftAnklePitch.value,
            G1_29_JointIndex.kRightAnklePitch.value,
            # Left arm
            G1_29_JointIndex.kLeftShoulderPitch.value,
            G1_29_JointIndex.kLeftShoulderRoll.value,
            G1_29_JointIndex.kLeftShoulderYaw.value,
            G1_29_JointIndex.kLeftElbow.value,
            # Right arm
            G1_29_JointIndex.kRightShoulderPitch.value,
            G1_29_JointIndex.kRightShoulderRoll.value,
            G1_29_JointIndex.kRightShoulderYaw.value,
            G1_29_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors
    
    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            G1_29_JointIndex.kLeftWristRoll.value,
            G1_29_JointIndex.kLeftWristPitch.value,
            G1_29_JointIndex.kLeftWristyaw.value,
            G1_29_JointIndex.kRightWristRoll.value,
            G1_29_JointIndex.kRightWristPitch.value,
            G1_29_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors

class G1_29_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

class G1_29_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    
    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34

class G1_29_Internal_Dex1_JointIndex(IntEnum):
    kLeftDex1_1 = 31
    kRightDex1_1 = 33

class G1_29_Arm_Internal_Dex1_Controller:
    BODY_MOTOR_COUNT = 29
    DELTA_GRIPPER_CMD = 0.18
    THUMB_INDEX_DISTANCE = (5.0, 7.0)
    MAPPED_GRIPPER_RANGE = (0.0, 5.4)

    def __init__(self, left_gripper_value_in, right_gripper_value_in,
                 dual_gripper_data_lock = None, dual_gripper_state_out = None,
                 dual_gripper_action_out = None, simulation_mode = False,
                 xr_motion_data_ready_in = None, motion_mode = False):
        logger_mp.info("Initialize G1_29_Arm_Internal_Dex1_Controller...")

        if motion_mode:
            raise ValueError("Internal Dex1 does not currently support motion mode.")

        self.left_gripper_value_in = left_gripper_value_in
        self.right_gripper_value_in = right_gripper_value_in
        self.dual_gripper_data_lock = dual_gripper_data_lock
        self.dual_gripper_state_out = dual_gripper_state_out
        self.dual_gripper_action_out = dual_gripper_action_out
        self.xr_motion_data_ready_in = xr_motion_data_ready_in

        self.left_gripper_index = G1_29_Internal_Dex1_JointIndex.kLeftDex1_1.value
        self.right_gripper_index = G1_29_Internal_Dex1_JointIndex.kRightDex1_1.value
        self.gripper_kp = 5.0
        self.gripper_kd = 0.05

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.gripper_q_target = np.zeros(2)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode

        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        self.all_motor_q = None
        self.set_arm_velocity_limit()
        self.control_dt = 1.0 / 250.0
        self.running = True
        self.ctrl_lock = threading.Lock()

        if simulation_mode:
            self.gripper_smooth_filter = None
        else:
            from teleop.utils.weighted_moving_filter import WeightedMovingFilter
            self.gripper_smooth_filter = WeightedMovingFilter(np.array([0.5, 0.3, 0.2]), 2)

        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "G1_29_Arm_Internal_Dex1_Controller")

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()

        self._configure_motor_commands()

        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize G1_29_Arm_Internal_Dex1_Controller OK!")

    def _ctrl_motor_state(self):
        while self.running:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target = self.q_target
                arm_tauff_target = self.tauff_target

            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(
                    arm_q_target,
                    velocity_limit=self.arm_velocity_limit,
                )

            for idx, id in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            self._update_internal_dex1_motor_commands()

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            sleep_time = max(0, self.control_dt - (time.time() - start_time))
            time.sleep(sleep_time)

    def _subscribe_motor_state(self):
        while self.running:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def _configure_motor_commands(self):
        self.gripper_q_target = self.get_current_dual_gripper_q()
        arm_indices = {member.value for member in G1_29_JointArmIndex}
        gripper_indices = {self.left_gripper_index, self.right_gripper_index}

        for id in G1_29_JointIndex:
            cmd = self.msg.motor_cmd[id]
            cmd.q = self.all_motor_q[id]
            cmd.dq = 0.0
            cmd.tau = 0.0
            if id.value in gripper_indices:
                cmd.mode = 1
                cmd.kp = self.gripper_kp
                cmd.kd = self.gripper_kd
            elif id.value >= self.BODY_MOTOR_COUNT:
                cmd.mode = 0
                cmd.kp = 0.0
                cmd.kd = 0.0
            elif id.value in arm_indices:
                cmd.mode = 1
                if self._Is_wrist_motor(id):
                    cmd.kp = self.kp_wrist
                    cmd.kd = self.kd_wrist
                else:
                    cmd.kp = self.kp_low
                    cmd.kd = self.kd_low
            else:
                cmd.mode = 1
                if self._Is_weak_motor(id):
                    cmd.kp = self.kp_low
                    cmd.kd = self.kd_low
                else:
                    cmd.kp = self.kp_high
                    cmd.kd = self.kd_high

        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q}\n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info(f"Current internal Dex1 q: {self.gripper_q_target}")

    def _update_internal_dex1_motor_commands(self):
        with self.left_gripper_value_in.get_lock():
            left_gripper_value = self.left_gripper_value_in.value
        with self.right_gripper_value_in.get_lock():
            right_gripper_value = self.right_gripper_value_in.value

        if self.xr_motion_data_ready_in is None:
            xr_motion_data_ready = True
        else:
            with self.xr_motion_data_ready_in.get_lock():
                xr_motion_data_ready = self.xr_motion_data_ready_in.value

        gripper_state = self.get_current_dual_gripper_q()
        if xr_motion_data_ready:
            gripper_q_target = np.array([
                np.interp(left_gripper_value, self.THUMB_INDEX_DISTANCE, self.MAPPED_GRIPPER_RANGE),
                np.interp(right_gripper_value, self.THUMB_INDEX_DISTANCE, self.MAPPED_GRIPPER_RANGE),
            ])
        else:
            gripper_q_target = gripper_state.copy()

        with self.ctrl_lock:
            self.gripper_q_target = gripper_q_target

        if self.simulation_mode:
            gripper_q_cmd = gripper_q_target
        else:
            gripper_q_cmd = np.clip(
                gripper_q_target,
                gripper_state - self.DELTA_GRIPPER_CMD,
                gripper_state + self.DELTA_GRIPPER_CMD,
            )

        if self.gripper_smooth_filter is not None:
            self.gripper_smooth_filter.add_data(gripper_q_cmd)
            gripper_q_cmd = self.gripper_smooth_filter.filtered_data

        for idx, id in enumerate((self.left_gripper_index, self.right_gripper_index)):
            cmd = self.msg.motor_cmd[id]
            cmd.mode = 1
            cmd.q = gripper_q_cmd[idx]
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = self.gripper_kp
            cmd.kd = self.gripper_kd

        if self.dual_gripper_state_out is not None and self.dual_gripper_action_out is not None:
            if self.dual_gripper_data_lock is None:
                self._write_gripper_output(gripper_state, gripper_q_cmd)
            else:
                with self.dual_gripper_data_lock:
                    self._write_gripper_output(gripper_state, gripper_q_cmd)

    def _write_gripper_output(self, state, action):
        self.dual_gripper_state_out[:] = state
        self.dual_gripper_action_out[:] = action

    def get_current_dual_gripper_q(self):
        lowstate = self.lowstate_buffer.GetData()
        return np.array([
            lowstate.motor_state[self.left_gripper_index].q,
            lowstate.motor_state[self.right_gripper_index].q,
        ])

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        return current_q + delta / max(motion_scale, 1.0)

    def ctrl_dual_arm(self, q_target, tauff_target):
        """Set target position and feed-forward torque for both arms."""
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        if self.mode_machine is None:
            raise RuntimeError("G1 low state is not ready.")
        return self.mode_machine

    def get_current_motor_q(self):
        lowstate = self.lowstate_buffer.GetData()
        return np.array([
            lowstate.motor_state[id].q
            for id in G1_29_JointIndex
        ])

    def get_current_dual_arm_q(self):
        lowstate = self.lowstate_buffer.GetData()
        return np.array([
            lowstate.motor_state[id].q
            for id in G1_29_JointArmIndex
        ])

    def get_current_dual_arm_dq(self):
        lowstate = self.lowstate_buffer.GetData()
        return np.array([
            lowstate.motor_state[id].dq
            for id in G1_29_JointArmIndex
        ])

    def ctrl_dual_arm_go_home(self):
        logger_mp.info("[G1_29_Arm_Internal_Dex1_Controller] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(14)

        tolerance = 0.05
        while current_attempts < max_attempts:
            if np.all(np.abs(self.get_current_dual_arm_q()) < tolerance):
                logger_mp.info(
                    "[G1_29_Arm_Internal_Dex1_Controller] both arms have reached the home position."
                )
                break
            current_attempts += 1
            time.sleep(0.05)

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            G1_29_JointIndex.kLeftAnklePitch.value,
            G1_29_JointIndex.kRightAnklePitch.value,
            G1_29_JointIndex.kLeftShoulderPitch.value,
            G1_29_JointIndex.kLeftShoulderRoll.value,
            G1_29_JointIndex.kLeftShoulderYaw.value,
            G1_29_JointIndex.kLeftElbow.value,
            G1_29_JointIndex.kRightShoulderPitch.value,
            G1_29_JointIndex.kRightShoulderRoll.value,
            G1_29_JointIndex.kRightShoulderYaw.value,
            G1_29_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            G1_29_JointIndex.kLeftWristRoll.value,
            G1_29_JointIndex.kLeftWristPitch.value,
            G1_29_JointIndex.kLeftWristyaw.value,
            G1_29_JointIndex.kRightWristRoll.value,
            G1_29_JointIndex.kRightWristPitch.value,
            G1_29_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors


class G1_23_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        self.simulation_mode = simulation_mode
        self.motion_mode = motion_mode

        logger_mp.info("Initialize G1_23_ArmController...")
        self.q_target = np.zeros(10)
        self.tauff_target = np.zeros(10)

        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5

        self.all_motor_q = None
        self.set_arm_velocity_limit(ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_START)
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

        # 渐变限速状态（speed_gradual_max 会置位；发布线程每周期都会读，
        # 必须在 __init__ 里先初始化，否则线程启动后、R 键之前会 AttributeError）
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        
        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "G1_23_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        # 关键：用实测关节角初始化目标值。
        # 原实现 q_target = zeros，而发布线程在 __init__ 末尾就启动，结果进程一
        # 启动、用户还没按 [r]，机械臂就会被 250Hz 地拉向全零位（可能穿过躯干）。
        # 此刻 lowstate 已在上面阻塞等待过，数据一定可用；发布线程尚未启动，
        # 因此这里不需要加锁。
        self.q_target = self.get_current_dual_arm_q().copy()
        self.tauff_target = np.zeros_like(self.q_target)
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        arm_indices = set(member.value for member in G1_23_JointArmIndex)
        for id in G1_23_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize G1_23_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_23_LowState()
                for id in range(G1_23_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit=None, dt=None):
        '''把目标关节角限制在"单周期最大步长"与"加速度上限"之内。

        这是下发前的最后一道数值闸门：
          * 形状不匹配 / NaN / Inf  -> 保持当前实测位置，绝不把坏值送进 DDS；
          * 速度限幅按比例缩放（保持各关节联动方向不变）；
          * 加速度限幅逐关节进行，避免速度指令阶跃。
        '''
        if velocity_limit is None:
            velocity_limit = self.arm_velocity_limit
        if dt is None:
            dt = self._dt_ema

        current_q = self.get_current_dual_arm_q()
        target_q = np.asarray(target_q, dtype=float)

        if target_q.shape != current_q.shape:
            _warn_throttled(
                "clip_shape",
                f"target shape {target_q.shape} != current {current_q.shape}, holding position",
            )
            return current_q
        if not np.all(np.isfinite(target_q)) or not np.all(np.isfinite(current_q)):
            _warn_throttled("clip_nan", "non-finite joint target rejected, holding position")
            return current_q

        delta = target_q - current_q
        max_step = max(velocity_limit * dt, 1e-6)
        peak = float(np.max(np.abs(delta)))
        if peak > max_step:
            delta = delta * (max_step / peak)

        if self._last_delta is not None and self._last_delta.shape == delta.shape:
            max_dv = max(self.arm_accel_limit * dt, 1e-6)
            delta = np.clip(delta, self._last_delta - max_dv, self._last_delta + max_dv)

        self._last_delta = delta
        return current_q + delta

    def _ctrl_motor_state(self):
        if self.motion_mode:
            self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = 1.0

        last_tick = time.monotonic()
        while True:
            try:
                start_time = time.monotonic()
                dt = max(start_time - last_tick, 1e-4)
                last_tick = start_time
                # 用实测周期做 EMA：调度抖动时仍按真实时间计算限速
                self._dt_ema = 0.9 * self._dt_ema + 0.1 * dt

                if not self.lowstate_buffer.IsFresh():
                    # DDS 状态陈旧 -> limiter 用的 current_q 不可信，冻结目标并告警
                    _warn_throttled("lowstate_stale", "lowstate stale, holding last valid command")

                with self.ctrl_lock:
                    arm_q_target     = self.q_target
                    arm_tauff_target = self.tauff_target

                if self.simulation_mode:
                    cliped_arm_q_target = arm_q_target
                else:
                    cliped_arm_q_target = self.clip_arm_q_target(
                        arm_q_target, velocity_limit=self.arm_velocity_limit, dt=self._dt_ema
                    )

                if (not np.all(np.isfinite(cliped_arm_q_target))
                        or not np.all(np.isfinite(arm_tauff_target))):
                    _warn_throttled("loop_nan", "non-finite command dropped in publish loop")
                else:
                    if self.motion_mode and self._release_motion_weight:
                        weight = self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q
                        self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = max(0.0, weight - dt / 2.0)

                    for idx, id in enumerate(G1_23_JointArmIndex):
                        self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                        self.msg.motor_cmd[id].dq = 0
                        self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

                    self.msg.crc = self.crc.Crc(self.msg)
                    self.lowcmd_publisher.Write(self.msg)

                if self._speed_gradual_max is True:
                    t_elapsed = start_time - self._gradual_start_time
                    ratio = min(1.0, t_elapsed / self._gradual_time) if self._gradual_time else 1.0
                    self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START + (
                        (ARM_VELOCITY_LIMIT_MAX - ARM_VELOCITY_LIMIT_START) * ratio
                    )

                all_t_elapsed = time.monotonic() - start_time
                time.sleep(max(0.0, self.control_dt - all_t_elapsed))
            except Exception:
                logger_mp.exception("[arm] publish loop error, continuing")
                time.sleep(0.01)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.

        非有限值（NaN/Inf）或形状不符的输入在这里被拦下并保留上一帧有效目标。
        上游 XR 追踪丢失时可能产生 NaN，若不拦会一路传到 CRC 与 DDS。
        '''
        q_target = np.asarray(q_target, dtype=float)
        tauff_target = np.asarray(tauff_target, dtype=float)
        if q_target.shape != self.q_target.shape or tauff_target.shape != self.tauff_target.shape:
            _warn_throttled(
                "cmd_shape",
                f"ctrl_dual_arm shape mismatch: q{q_target.shape} tau{tauff_target.shape}",
            )
            return
        if not np.all(np.isfinite(q_target)) or not np.all(np.isfinite(tauff_target)):
            _warn_throttled("cmd_nan", "ctrl_dual_arm rejected non-finite target, keeping previous")
            return
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        '''Return current dds mode machine.'''
        if self.mode_machine is None:
            raise RuntimeError("G1 low state is not ready.")
        return self.mode_machine
    
    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointIndex])
    
    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointArmIndex])
    
    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_23_JointArmIndex])
    
    def ctrl_dual_arm_go_home(self):
        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
        logger_mp.info("[G1_23_ArmController] ctrl_dual_arm_go_home: hold current pose and release motion weight...")
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q().copy()
        if not self.motion_mode:
            logger_mp.info("[G1_23_ArmController] debug mode: holding current pose.")
            return
        self._release_motion_weight = True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q <= 0.01:
                break
            time.sleep(0.05)
        logger_mp.info("[G1_23_ArmController] motion weight released, arms hold pose.")

    def speed_gradual_max(self, t = 5.0):
        '''Parameter t is the total time required for arms velocity to gradually increase to its maximum value, in seconds. The default is 5.0.'''
        # 必须与发布循环里的 time.monotonic() 同源，否则会算出巨大的 elapsed
        self._gradual_start_time = time.monotonic()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        '''set arms velocity to the maximum value immediately, instead of gradually increasing.'''
        # 仿真没有物理约束，保留原量级；真机则用收紧后的上限
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_MAX

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            G1_23_JointIndex.kLeftAnklePitch.value,
            G1_23_JointIndex.kRightAnklePitch.value,
            # Left arm
            G1_23_JointIndex.kLeftShoulderPitch.value,
            G1_23_JointIndex.kLeftShoulderRoll.value,
            G1_23_JointIndex.kLeftShoulderYaw.value,
            G1_23_JointIndex.kLeftElbow.value,
            # Right arm
            G1_23_JointIndex.kRightShoulderPitch.value,
            G1_23_JointIndex.kRightShoulderRoll.value,
            G1_23_JointIndex.kRightShoulderYaw.value,
            G1_23_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors
    
    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            G1_23_JointIndex.kLeftWristRoll.value,
            G1_23_JointIndex.kRightWristRoll.value,
        ]
        return motor_index.value in wrist_motors

class G1_23_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26

class G1_23_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRollNotUsed = 13
    kWaistPitchNotUsed = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitchNotUsed = 20
    kLeftWristyawNotUsed = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitchNotUsed = 27
    kRightWristYawNotUsed = 28
    
    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34

class H1_2_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        self.simulation_mode = simulation_mode
        self.motion_mode = motion_mode
        
        logger_mp.info("Initialize H1_2_ArmController...")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)

        self.kp_high = 300.0
        self.kd_high = 5.0
        self.kp_low = 140.0
        self.kd_low = 3.0
        self.kp_wrist = 50.0
        self.kd_wrist = 2.0

        self.all_motor_q = None
        self.set_arm_velocity_limit(ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_START)
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

        # 渐变限速状态（speed_gradual_max 会置位；发布线程每周期都会读，
        # 必须在 __init__ 里先初始化，否则线程启动后、R 键之前会 AttributeError）
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None


        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "H1_2_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        # 关键：用实测关节角初始化目标值。
        # 原实现 q_target = zeros，而发布线程在 __init__ 末尾就启动，结果进程一
        # 启动、用户还没按 [r]，机械臂就会被 250Hz 地拉向全零位（可能穿过躯干）。
        # 此刻 lowstate 已在上面阻塞等待过，数据一定可用；发布线程尚未启动，
        # 因此这里不需要加锁。
        self.q_target = self.get_current_dual_arm_q().copy()
        self.tauff_target = np.zeros_like(self.q_target)
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        arm_indices = set(member.value for member in H1_2_JointArmIndex)
        for id in H1_2_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize H1_2_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = H1_2_LowState()
                for id in range(H1_2_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit=None, dt=None):
        '''把目标关节角限制在"单周期最大步长"与"加速度上限"之内。

        这是下发前的最后一道数值闸门：
          * 形状不匹配 / NaN / Inf  -> 保持当前实测位置，绝不把坏值送进 DDS；
          * 速度限幅按比例缩放（保持各关节联动方向不变）；
          * 加速度限幅逐关节进行，避免速度指令阶跃。
        '''
        if velocity_limit is None:
            velocity_limit = self.arm_velocity_limit
        if dt is None:
            dt = self._dt_ema

        current_q = self.get_current_dual_arm_q()
        target_q = np.asarray(target_q, dtype=float)

        if target_q.shape != current_q.shape:
            _warn_throttled(
                "clip_shape",
                f"target shape {target_q.shape} != current {current_q.shape}, holding position",
            )
            return current_q
        if not np.all(np.isfinite(target_q)) or not np.all(np.isfinite(current_q)):
            _warn_throttled("clip_nan", "non-finite joint target rejected, holding position")
            return current_q

        delta = target_q - current_q
        max_step = max(velocity_limit * dt, 1e-6)
        peak = float(np.max(np.abs(delta)))
        if peak > max_step:
            delta = delta * (max_step / peak)

        if self._last_delta is not None and self._last_delta.shape == delta.shape:
            max_dv = max(self.arm_accel_limit * dt, 1e-6)
            delta = np.clip(delta, self._last_delta - max_dv, self._last_delta + max_dv)

        self._last_delta = delta
        return current_q + delta

    def _ctrl_motor_state(self):
        if self.motion_mode:
            self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q = 1.0

        last_tick = time.monotonic()
        while True:
            try:
                start_time = time.monotonic()
                dt = max(start_time - last_tick, 1e-4)
                last_tick = start_time
                # 用实测周期做 EMA：调度抖动时仍按真实时间计算限速
                self._dt_ema = 0.9 * self._dt_ema + 0.1 * dt

                if not self.lowstate_buffer.IsFresh():
                    # DDS 状态陈旧 -> limiter 用的 current_q 不可信，冻结目标并告警
                    _warn_throttled("lowstate_stale", "lowstate stale, holding last valid command")

                with self.ctrl_lock:
                    arm_q_target     = self.q_target
                    arm_tauff_target = self.tauff_target

                if self.simulation_mode:
                    cliped_arm_q_target = arm_q_target
                else:
                    cliped_arm_q_target = self.clip_arm_q_target(
                        arm_q_target, velocity_limit=self.arm_velocity_limit, dt=self._dt_ema
                    )

                if (not np.all(np.isfinite(cliped_arm_q_target))
                        or not np.all(np.isfinite(arm_tauff_target))):
                    _warn_throttled("loop_nan", "non-finite command dropped in publish loop")
                else:
                    if self.motion_mode and self._release_motion_weight:
                        weight = self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q
                        self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q = max(0.0, weight - dt / 2.0)

                    for idx, id in enumerate(H1_2_JointArmIndex):
                        self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                        self.msg.motor_cmd[id].dq = 0
                        self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

                    self.msg.crc = self.crc.Crc(self.msg)
                    self.lowcmd_publisher.Write(self.msg)

                if self._speed_gradual_max is True:
                    t_elapsed = start_time - self._gradual_start_time
                    ratio = min(1.0, t_elapsed / self._gradual_time) if self._gradual_time else 1.0
                    self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START + (
                        (ARM_VELOCITY_LIMIT_MAX - ARM_VELOCITY_LIMIT_START) * ratio
                    )

                all_t_elapsed = time.monotonic() - start_time
                time.sleep(max(0.0, self.control_dt - all_t_elapsed))
            except Exception:
                logger_mp.exception("[arm] publish loop error, continuing")
                time.sleep(0.01)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.

        非有限值（NaN/Inf）或形状不符的输入在这里被拦下并保留上一帧有效目标。
        上游 XR 追踪丢失时可能产生 NaN，若不拦会一路传到 CRC 与 DDS。
        '''
        q_target = np.asarray(q_target, dtype=float)
        tauff_target = np.asarray(tauff_target, dtype=float)
        if q_target.shape != self.q_target.shape or tauff_target.shape != self.tauff_target.shape:
            _warn_throttled(
                "cmd_shape",
                f"ctrl_dual_arm shape mismatch: q{q_target.shape} tau{tauff_target.shape}",
            )
            return
        if not np.all(np.isfinite(q_target)) or not np.all(np.isfinite(tauff_target)):
            _warn_throttled("cmd_nan", "ctrl_dual_arm rejected non-finite target, keeping previous")
            return
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        '''Return current dds mode machine.'''
        if self.mode_machine is None:
            raise RuntimeError("H1-2 low state is not ready.")
        return self.mode_machine
    
    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_2_JointIndex])
    
    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_2_JointArmIndex])
    
    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in H1_2_JointArmIndex])
    
    def ctrl_dual_arm_go_home(self):
        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
        logger_mp.info("[H1_2_ArmController] ctrl_dual_arm_go_home: hold current pose and release motion weight...")
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q().copy()
        if not self.motion_mode:
            logger_mp.info("[H1_2_ArmController] debug mode: holding current pose.")
            return
        self._release_motion_weight = True
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q <= 0.01:
                break
            time.sleep(0.05)
        logger_mp.info("[H1_2_ArmController] motion weight released, arms hold pose.")

    def speed_gradual_max(self, t = 5.0):
        '''Parameter t is the total time required for arms velocity to gradually increase to its maximum value, in seconds. The default is 5.0.'''
        # 必须与发布循环里的 time.monotonic() 同源，否则会算出巨大的 elapsed
        self._gradual_start_time = time.monotonic()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        '''set arms velocity to the maximum value immediately, instead of gradually increasing.'''
        # 仿真没有物理约束，保留原量级；真机则用收紧后的上限
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_MAX

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            H1_2_JointIndex.kLeftAnkle.value,
            H1_2_JointIndex.kRightAnkle.value,
            # Left arm
            H1_2_JointIndex.kLeftShoulderPitch.value,
            H1_2_JointIndex.kLeftShoulderRoll.value,
            H1_2_JointIndex.kLeftShoulderYaw.value,
            H1_2_JointIndex.kLeftElbowPitch.value,
            # Right arm
            H1_2_JointIndex.kRightShoulderPitch.value,
            H1_2_JointIndex.kRightShoulderRoll.value,
            H1_2_JointIndex.kRightShoulderYaw.value,
            H1_2_JointIndex.kRightElbowPitch.value,
        ]
        return motor_index.value in weak_motors
    
    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            H1_2_JointIndex.kLeftElbowRoll.value,
            H1_2_JointIndex.kLeftWristPitch.value,
            H1_2_JointIndex.kLeftWristyaw.value,
            H1_2_JointIndex.kRightElbowRoll.value,
            H1_2_JointIndex.kRightWristPitch.value,
            H1_2_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors
    
class H1_2_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 13
    kLeftShoulderRoll = 14
    kLeftShoulderYaw = 15
    kLeftElbowPitch = 16
    kLeftElbowRoll = 17
    kLeftWristPitch = 18
    kLeftWristyaw = 19

    # Right arm
    kRightShoulderPitch = 20
    kRightShoulderRoll = 21
    kRightShoulderYaw = 22
    kRightElbowPitch = 23
    kRightElbowRoll = 24
    kRightWristPitch = 25
    kRightWristYaw = 26

class H1_2_JointIndex(IntEnum):
    # Left leg
    kLeftHipYaw = 0
    kLeftHipRoll = 1
    kLeftHipPitch = 2
    kLeftKnee = 3
    kLeftAnkle = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipYaw = 6
    kRightHipRoll = 7
    kRightHipPitch = 8
    kRightKnee = 9
    kRightAnkle = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12

    # Left arm
    kLeftShoulderPitch = 13
    kLeftShoulderRoll = 14
    kLeftShoulderYaw = 15
    kLeftElbowPitch = 16
    kLeftElbowRoll = 17
    kLeftWristPitch = 18
    kLeftWristyaw = 19

    # Right arm
    kRightShoulderPitch = 20
    kRightShoulderRoll = 21
    kRightShoulderYaw = 22
    kRightElbowPitch = 23
    kRightElbowRoll = 24
    kRightWristPitch = 25
    kRightWristYaw = 26

    kNotUsedJoint0 = 27
    kNotUsedJoint1 = 28
    kNotUsedJoint2 = 29
    kNotUsedJoint3 = 30
    kNotUsedJoint4 = 31
    kNotUsedJoint5 = 32
    kNotUsedJoint6 = 33
    kNotUsedJoint7 = 34

class H1_ArmController:
    def __init__(self, simulation_mode = False):
        self.simulation_mode = simulation_mode
        
        logger_mp.info("Initialize H1_ArmController...")
        self.q_target = np.zeros(8)
        self.tauff_target = np.zeros(8)

        self.kp_high = 300.0
        self.kd_high = 5.0
        self.kp_low = 140.0
        self.kd_low = 3.0

        self.all_motor_q = None
        self.set_arm_velocity_limit(ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_START)
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

        # 渐变限速状态（speed_gradual_max 会置位；发布线程每周期都会读，
        # 必须在 __init__ 里先初始化，否则线程启动后、R 键之前会 AttributeError）
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, go_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, go_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "H1_ArmController")

        # initialize h1's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_go_msg_dds__LowCmd_()
        self.msg.head[0] = 0xFE
        self.msg.head[1] = 0xEF
        self.msg.level_flag = 0xFF
        self.msg.gpio = 0

        self.all_motor_q = self.get_current_motor_q()
        # 关键：用实测关节角初始化目标值。
        # 原实现 q_target = zeros，而发布线程在 __init__ 末尾就启动，结果进程一
        # 启动、用户还没按 [r]，机械臂就会被 250Hz 地拉向全零位（可能穿过躯干）。
        # 此刻 lowstate 已在上面阻塞等待过，数据一定可用；发布线程尚未启动，
        # 因此这里不需要加锁。
        self.q_target = self.get_current_dual_arm_q().copy()
        self.tauff_target = np.zeros_like(self.q_target)
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        for id in H1_JointIndex:
            if self._Is_weak_motor(id):
                self.msg.motor_cmd[id].kp = self.kp_low
                self.msg.motor_cmd[id].kd = self.kd_low
                self.msg.motor_cmd[id].mode = 0x01
            else:
                self.msg.motor_cmd[id].kp = self.kp_high
                self.msg.motor_cmd[id].kd = self.kd_high
                self.msg.motor_cmd[id].mode = 0x0A
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize H1_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = H1_LowState()
                for id in range(H1_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit=None, dt=None):
        '''把目标关节角限制在"单周期最大步长"与"加速度上限"之内。

        这是下发前的最后一道数值闸门：
          * 形状不匹配 / NaN / Inf  -> 保持当前实测位置，绝不把坏值送进 DDS；
          * 速度限幅按比例缩放（保持各关节联动方向不变）；
          * 加速度限幅逐关节进行，避免速度指令阶跃。
        '''
        if velocity_limit is None:
            velocity_limit = self.arm_velocity_limit
        if dt is None:
            dt = self._dt_ema

        current_q = self.get_current_dual_arm_q()
        target_q = np.asarray(target_q, dtype=float)

        if target_q.shape != current_q.shape:
            _warn_throttled(
                "clip_shape",
                f"target shape {target_q.shape} != current {current_q.shape}, holding position",
            )
            return current_q
        if not np.all(np.isfinite(target_q)) or not np.all(np.isfinite(current_q)):
            _warn_throttled("clip_nan", "non-finite joint target rejected, holding position")
            return current_q

        delta = target_q - current_q
        max_step = max(velocity_limit * dt, 1e-6)
        peak = float(np.max(np.abs(delta)))
        if peak > max_step:
            delta = delta * (max_step / peak)

        if self._last_delta is not None and self._last_delta.shape == delta.shape:
            max_dv = max(self.arm_accel_limit * dt, 1e-6)
            delta = np.clip(delta, self._last_delta - max_dv, self._last_delta + max_dv)

        self._last_delta = delta
        return current_q + delta

    def _ctrl_motor_state(self):
        last_tick = time.monotonic()
        while True:
            try:
                start_time = time.monotonic()
                dt = max(start_time - last_tick, 1e-4)
                last_tick = start_time
                # 用实测周期做 EMA：调度抖动时仍按真实时间计算限速
                self._dt_ema = 0.9 * self._dt_ema + 0.1 * dt

                if not self.lowstate_buffer.IsFresh():
                    # DDS 状态陈旧 -> limiter 用的 current_q 不可信，冻结目标并告警
                    _warn_throttled("lowstate_stale", "lowstate stale, holding last valid command")

                with self.ctrl_lock:
                    arm_q_target     = self.q_target
                    arm_tauff_target = self.tauff_target

                if self.simulation_mode:
                    cliped_arm_q_target = arm_q_target
                else:
                    cliped_arm_q_target = self.clip_arm_q_target(
                        arm_q_target, velocity_limit=self.arm_velocity_limit, dt=self._dt_ema
                    )

                if (not np.all(np.isfinite(cliped_arm_q_target))
                        or not np.all(np.isfinite(arm_tauff_target))):
                    _warn_throttled("loop_nan", "non-finite command dropped in publish loop")
                else:
                    for idx, id in enumerate(H1_JointArmIndex):
                        self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                        self.msg.motor_cmd[id].dq = 0
                        self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

                    self.msg.crc = self.crc.Crc(self.msg)
                    self.lowcmd_publisher.Write(self.msg)

                if self._speed_gradual_max is True:
                    t_elapsed = start_time - self._gradual_start_time
                    ratio = min(1.0, t_elapsed / self._gradual_time) if self._gradual_time else 1.0
                    self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START + (
                        (ARM_VELOCITY_LIMIT_MAX - ARM_VELOCITY_LIMIT_START) * ratio
                    )

                all_t_elapsed = time.monotonic() - start_time
                time.sleep(max(0.0, self.control_dt - all_t_elapsed))
            except Exception:
                logger_mp.exception("[arm] publish loop error, continuing")
                time.sleep(0.01)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.

        非有限值（NaN/Inf）或形状不符的输入在这里被拦下并保留上一帧有效目标。
        上游 XR 追踪丢失时可能产生 NaN，若不拦会一路传到 CRC 与 DDS。
        '''
        q_target = np.asarray(q_target, dtype=float)
        tauff_target = np.asarray(tauff_target, dtype=float)
        if q_target.shape != self.q_target.shape or tauff_target.shape != self.tauff_target.shape:
            _warn_throttled(
                "cmd_shape",
                f"ctrl_dual_arm shape mismatch: q{q_target.shape} tau{tauff_target.shape}",
            )
            return
        if not np.all(np.isfinite(q_target)) or not np.all(np.isfinite(tauff_target)):
            _warn_throttled("cmd_nan", "ctrl_dual_arm rejected non-finite target, keeping previous")
            return
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target
    
    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_JointIndex])
    
    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_JointArmIndex])
    
    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in H1_JointArmIndex])
    
    def ctrl_dual_arm_go_home(self):
        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
        logger_mp.info("[H1_ArmController] ctrl_dual_arm_go_home: hold current pose...")
        with self.ctrl_lock:
            self.q_target = self.get_current_dual_arm_q().copy()
        # H1 只走 debug 通道，没有 arm_sdk 权重需要卸载
        logger_mp.info("[H1_ArmController] holding current pose.")

    def speed_gradual_max(self, t = 5.0):
        '''Parameter t is the total time required for arms velocity to gradually increase to its maximum value, in seconds. The default is 5.0.'''
        # 必须与发布循环里的 time.monotonic() 同源，否则会算出巨大的 elapsed
        self._gradual_start_time = time.monotonic()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        '''set arms velocity to the maximum value immediately, instead of gradually increasing.'''
        # 仿真没有物理约束，保留原量级；真机则用收紧后的上限
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_SIM if self.simulation_mode else ARM_VELOCITY_LIMIT_MAX

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            H1_JointIndex.kLeftAnkle.value,
            H1_JointIndex.kRightAnkle.value,
            # Left arm
            H1_JointIndex.kLeftShoulderPitch.value,
            H1_JointIndex.kLeftShoulderRoll.value,
            H1_JointIndex.kLeftShoulderYaw.value,
            H1_JointIndex.kLeftElbow.value,
            # Right arm
            H1_JointIndex.kRightShoulderPitch.value,
            H1_JointIndex.kRightShoulderRoll.value,
            H1_JointIndex.kRightShoulderYaw.value,
            H1_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors
    
class H1_JointArmIndex(IntEnum):
    # Unlike G1 and H1_2, the arm order in DDS messages for H1 is right then left. 
    # Therefore, the purpose of switching the order here is to maintain consistency with G1 and H1_2.
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19
    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15

class H1_JointIndex(IntEnum):
    kRightHipRoll = 0
    kRightHipPitch = 1
    kRightKnee = 2
    kLeftHipRoll = 3
    kLeftHipPitch = 4
    kLeftKnee = 5
    kWaistYaw = 6
    kLeftHipYaw = 7
    kRightHipYaw = 8
    kNotUsedJoint = 9
    kLeftAnkle = 10
    kRightAnkle = 11
    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19

class H2_ArmController:
    def __init__(self, motion_mode=False, simulation_mode=False):
        logger_mp.info("Initialize H2_ArmController...")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 300.0
        self.kd_high = 5.0
        self.kp_low = 140.0
        self.kd_low = 3.0
        self.kp_wrist = 50.0
        self.kd_wrist = 2.0

        self.all_motor_q = None
        self.set_arm_velocity_limit()
        self.control_dt = 1.0 / 250.0
        
        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "H2_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        arm_indices = set(member.value for member in H2_JointArmIndex)
        for id in H2_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            logger_mp.info(
                f"Motor {id.value} ({id.name}): kp={self.msg.motor_cmd[id].kp}, kd={self.msg.motor_cmd[id].kd}"
            )
            self.msg.motor_cmd[id].q = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize H2_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = H2_LowState()
                for id in range(35):
                    lowstate.motor_state[id].q = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self):
        if self.motion_mode:
            self.msg.motor_cmd[H2_JointIndex.kNotUsedJoint0].q = 1.0

        while True:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target = self.q_target
                arm_tauff_target = self.tauff_target

            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit=self.arm_velocity_limit)

            for idx, id in enumerate(H2_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)

    def ctrl_dual_arm(self, q_target, tauff_target):
        """Set control target values q & tau of the left and right arm motors."""
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        """Return current dds mode machine."""
        if self.mode_machine is None:
            raise RuntimeError("H2 low state is not ready.")
        return self.mode_machine

    def get_current_motor_q(self):
        """Return current state q of all body motors."""
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H2_JointIndex])

    def get_current_dual_arm_q(self):
        """Return current state q of the left and right arm motors."""
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H2_JointArmIndex])

    def get_current_dual_arm_dq(self):
        """Return current state dq of the left and right arm motors."""
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in H2_JointArmIndex])

    def ctrl_dual_arm_go_home(self):
        """Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero."""
        logger_mp.info("[H2_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
        tolerance = 0.05
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                if self.motion_mode:
                    for weight in np.linspace(1, 0, num=101):
                        self.msg.motor_cmd[H2_JointIndex.kNotUsedJoint0].q = weight
                        time.sleep(0.02)
                logger_mp.info("[H2_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            H2_JointIndex.kLeftAnklePitch.value,
            H2_JointIndex.kRightAnklePitch.value,
            # Left arm
            H2_JointIndex.kLeftShoulderPitch.value,
            H2_JointIndex.kLeftShoulderRoll.value,
            H2_JointIndex.kLeftShoulderYaw.value,
            H2_JointIndex.kLeftElbow.value,
            # Right arm
            H2_JointIndex.kRightShoulderPitch.value,
            H2_JointIndex.kRightShoulderRoll.value,
            H2_JointIndex.kRightShoulderYaw.value,
            H2_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            H2_JointIndex.kLeftWristRoll.value,
            H2_JointIndex.kLeftWristPitch.value,
            H2_JointIndex.kLeftWristyaw.value,
            H2_JointIndex.kRightWristRoll.value,
            H2_JointIndex.kRightWristPitch.value,
            H2_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors

class H2_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


class H2_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnkleRoll = 4
    kLeftAnklePitch = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnkleRoll = 10
    kRightAnklePitch = 11

    kWaistRoll = 12
    kWaistPitch = 13
    kWaistYaw = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

    # Head
    kHeadPitch = 29
    kHeadYaw = 30

    # not used
    kNotUsedJoint0 = 31
    kNotUsedJoint1 = 32
    kNotUsedJoint2 = 33
    kNotUsedJoint3 = 34

class R1_A5_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger_mp.info("Initialize R1_A5_ArmController...")
        self.q_target = np.zeros(10)
        self.tauff_target = np.zeros(10)
        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.kp_high = 200.0
        self.kd_high = 3.0
        self.kp_low = 50.0
        self.kd_low = 2.0
        self.kp_medium = 40.0
        self.kd_medium = 2.0
        self.kp_wrist = 30.0
        self.kd_wrist = 2.0
        self.kp_head = 15.0
        self.kd_head = 1.0

        self.all_motor_q = None
        self.set_arm_velocity_limit()
        self.control_dt = 1.0 / 250.0

        if self.motion_mode:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "R1_A5_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        # R1/R1-A5 arm_sdk uses mode_pr as a percentage weight.  This differs from
        # G1/H1_2, which use an otherwise notused joint command as the weight.
        self.msg.mode_pr = 100 if self.motion_mode else 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Initialize R1 A5 joint commands...")
        arm_indices = set(member.value for member in R1_A5_JointArmIndex)
        head_indices = set(member.value for member in R1_A5_JointHeadIndex)
        # Slot 12 is unused on R1-A5 and is waist roll on R1; slot 13 is waist
        # yaw on both. Neither waist joint is updated by teleoperation.
        waist_indices = {
            R1_A5_JointIndex.kWaistRollNotUsed.value,
            R1_A5_JointIndex.kWaistYaw.value,
        }
        arm_sdk_indices = arm_indices | head_indices | waist_indices
        for id in R1_A5_JointIndex:
            # rt/lowcmd still needs a complete low-level command.
            if self.motion_mode and id.value not in arm_sdk_indices:
                continue
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                elif self._Is_medium_arm_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_medium
                    self.msg.motor_cmd[id].kd = self.kd_medium
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            elif id.value in head_indices:
                self.msg.motor_cmd[id].kp = self.kp_head
                self.msg.motor_cmd[id].kd = self.kd_head
            elif id.value in waist_indices:
                self.msg.motor_cmd[id].kp = self.kp_low
                self.msg.motor_cmd[id].kd = self.kd_high
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q = self.all_motor_q[id]
            self.msg.motor_cmd[id].dq = 0.0
            self.msg.motor_cmd[id].tau = 0.0
        logger_mp.info("R1 A5 joint commands initialized.")

        # Head and available waist joints gradually return to zero at startup.
        self.ctrl_head_and_waist_go_home()

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize R1_A5_ArmController OK!")

    def _set_arm_sdk_weight(self, weight):
        """Set the R1 arm_sdk blend weight, where mode_pr stores 0..100."""
        self.msg.mode_pr = int(np.clip(weight, 0.0, 1.0) * 100.0)

    def release_arm_sdk(self, duration=2.0):
        """Smoothly release R1 arm_sdk control back to ai_sport."""
        if not self.motion_mode:
            return
        steps = max(1, int(duration / self.control_dt))
        for weight in np.linspace(self.msg.mode_pr / 100.0, 0.0, num=steps + 1):
            self._set_arm_sdk_weight(weight)
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(self.control_dt)
        self._set_arm_sdk_weight(0.0)

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = R1_A5_LowState()
                for id in range(R1_A5_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def ctrl_head_and_waist_go_home(self, duration = 3.0):
        '''Linearly move the head and available waist joints to zero at startup.'''
        logger_mp.info("[R1_A5_ArmController] head and waist returning to zero...")
        waist_indices = (
            R1_A5_JointIndex.kWaistRollNotUsed,
            R1_A5_JointIndex.kWaistYaw,
        )
        start_head_q = self.get_current_head_q()
        start_waist_q = self.all_motor_q[[id.value for id in waist_indices]]
        steps = max(1, int(duration / self.control_dt))
        for step in range(1, steps + 1):
            scale = 1.0 - step / steps
            head_q = start_head_q * scale
            waist_q = start_waist_q * scale
            for idx, id in enumerate(R1_A5_JointHeadIndex):
                self.msg.motor_cmd[id].q = head_q[idx]
            for idx, id in enumerate(waist_indices):
                self.msg.motor_cmd[id].q = waist_q[idx]
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(self.control_dt)
        logger_mp.info("[R1_A5_ArmController] head and waist return to zero OK!")

    def _ctrl_motor_state(self):
        while True:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            for idx, id in enumerate(R1_A5_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.'''
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        '''Return current dds mode machine.'''
        if self.mode_machine is None:
            raise RuntimeError("R1-A5 low state is not ready.")
        return self.mode_machine

    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A5_JointIndex])

    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A5_JointArmIndex])

    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in R1_A5_JointArmIndex])

    def get_current_head_q(self):
        '''Return current state q of the head pitch/yaw motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A5_JointHeadIndex])

    def ctrl_dual_arm_go_home(self):
        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
        logger_mp.info("[R1_A5_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(10)
            # self.tauff_target = np.zeros(10)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        reached_home = False
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                reached_home = True
                logger_mp.info("[R1_A5_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)
        if not reached_home:
            logger_mp.warning("[R1_A5_ArmController] timed out while returning arms home.")
        # Always release the overlay on shutdown, even when homing times out.
        self.release_arm_sdk()

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            R1_A5_JointIndex.kLeftAnklePitch.value,
            R1_A5_JointIndex.kRightAnklePitch.value,
            # Left arm
            R1_A5_JointIndex.kLeftShoulderPitch.value,
            R1_A5_JointIndex.kLeftShoulderRoll.value,
            R1_A5_JointIndex.kLeftShoulderYaw.value,
            R1_A5_JointIndex.kLeftElbow.value,
            # Right arm
            R1_A5_JointIndex.kRightShoulderPitch.value,
            R1_A5_JointIndex.kRightShoulderRoll.value,
            R1_A5_JointIndex.kRightShoulderYaw.value,
            R1_A5_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            R1_A5_JointIndex.kLeftWristRoll.value,
            R1_A5_JointIndex.kRightWristRoll.value,
        ]
        return motor_index.value in wrist_motors

    def _Is_medium_arm_motor(self, motor_index):
        medium_arm_motors = [
            R1_A5_JointIndex.kLeftShoulderYaw.value,
            R1_A5_JointIndex.kLeftElbow.value,
            R1_A5_JointIndex.kRightShoulderYaw.value,
            R1_A5_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in medium_arm_motors

class R1_A5_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26

class R1_A5_JointHeadIndex(IntEnum):
    kHeadPitch = 29
    kHeadYaw = 30

class R1_A5_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistRollNotUsed = 12
    kWaistYaw = 13
    kWaistPitchNotUsed = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitchNotUsed = 20
    kLeftWristyawNotUsed = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitchNotUsed = 27
    kRightWristYawNotUsed = 28

    # Head
    kHeadPitch = 29
    kHeadYaw = 30

    # not used
    kNotUsedJoint0 = 31
    kNotUsedJoint1 = 32
    kNotUsedJoint2 = 33
    kNotUsedJoint3 = 34

class R1_A7_ArmController:
    def __init__(self, motion_mode = False, simulation_mode = False):
        logger_mp.info("Initialize R1_A7_ArmController...")
        if motion_mode:
            raise ValueError("R1_A7_ArmController does not support motion mode.")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.simulation_mode = simulation_mode
        self.kp_high = 200.0
        self.kd_high = 3.0
        self.kp_low = 50.0
        self.kd_low = 2.0
        self.kp_medium = 40.0
        self.kd_medium = 2.0
        self.kp_wrist = 30.0
        self.kd_wrist = 2.0
        self.kp_head = 15.0
        self.kd_head = 1.0

        self.all_motor_q = None
        self.set_arm_velocity_limit()
        self.control_dt = 1.0 / 250.0

        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()
        self.mode_machine = None
        self.lowstate_sub_ready = False

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        wait_for_dds(lambda: self.lowstate_sub_ready, "R1_A7_ArmController")

        # initialize hg's lowcmd msg
        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms and head...")

        arm_indices = set(member.value for member in R1_A7_JointArmIndex)
        head_indices = set(member.value for member in R1_A7_JointHeadIndex)
        waist_indices = {
            R1_A7_JointIndex.kWaistRollNotUsed.value,
            R1_A7_JointIndex.kWaistYaw.value,
        }
        for id in R1_A7_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                elif self._Is_medium_arm_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_medium
                    self.msg.motor_cmd[id].kd = self.kd_medium
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            elif id.value in head_indices:
                self.msg.motor_cmd[id].kp = self.kp_head
                self.msg.motor_cmd[id].kd = self.kd_head
            elif id.value in waist_indices:
                self.msg.motor_cmd[id].kp = self.kp_low
                self.msg.motor_cmd[id].kd = self.kd_high
            else:
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # Head and available waist joints gradually return to zero at startup.
        self.ctrl_head_and_waist_go_home()

        # initialize publish thread
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize R1_A7_ArmController OK!")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = R1_A7_LowState()
                for id in range(R1_A7_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
                self.mode_machine = msg.mode_machine
                self.lowstate_sub_ready = True
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def ctrl_head_and_waist_go_home(self, duration = 3.0):
        '''Linearly move the head and available waist joints to zero at startup.'''
        logger_mp.info("[R1_A7_ArmController] head and waist returning to zero...")
        waist_indices = (
            R1_A7_JointIndex.kWaistRollNotUsed,
            R1_A7_JointIndex.kWaistYaw,
        )
        start_head_q = self.get_current_head_q()
        start_waist_q = self.all_motor_q[[id.value for id in waist_indices]]
        steps = max(1, int(duration / self.control_dt))
        for step in range(1, steps + 1):
            scale = 1.0 - step / steps
            head_q = start_head_q * scale
            waist_q = start_waist_q * scale
            for idx, id in enumerate(R1_A7_JointHeadIndex):
                self.msg.motor_cmd[id].q = head_q[idx]
            for idx, id in enumerate(waist_indices):
                self.msg.motor_cmd[id].q = waist_q[idx]
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(self.control_dt)
        logger_mp.info("[R1_A7_ArmController] head and waist return to zero OK!")

    def _ctrl_motor_state(self):
        while True:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            for idx, id in enumerate(R1_A7_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)

    def ctrl_dual_arm(self, q_target, tauff_target):
        '''Set control target values q & tau of the left and right arm motors.'''
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self):
        '''Return current dds mode machine.'''
        if self.mode_machine is None:
            raise RuntimeError("R1-A7 low state is not ready.")
        return self.mode_machine

    def get_current_motor_q(self):
        '''Return current state q of all body motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A7_JointIndex])

    def get_current_dual_arm_q(self):
        '''Return current state q of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A7_JointArmIndex])

    def get_current_dual_arm_dq(self):
        '''Return current state dq of the left and right arm motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in R1_A7_JointArmIndex])

    def get_current_head_q(self):
        '''Return current state q of the head pitch/yaw motors.'''
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in R1_A7_JointHeadIndex])

    def ctrl_dual_arm_go_home(self):
        '''Move both the left and right arms of the robot to their home position by setting the target joint angles (q) and torques (tau) to zero.'''
        logger_mp.info("[R1_A7_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
            # self.tauff_target = np.zeros(14)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                logger_mp.info("[R1_A7_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def set_arm_velocity_limit(self, velocity_limit = 30.0):
        '''Set the arm joint velocity limit in radians per second.'''
        velocity_limit = float(velocity_limit)
        if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
            raise ValueError("arm_velocity_limit must be a positive finite value.")
        self.arm_velocity_limit = velocity_limit

    def _Is_weak_motor(self, motor_index):
        weak_motors = [
            R1_A7_JointIndex.kLeftAnklePitch.value,
            R1_A7_JointIndex.kRightAnklePitch.value,
            # Left arm
            R1_A7_JointIndex.kLeftShoulderPitch.value,
            R1_A7_JointIndex.kLeftShoulderRoll.value,
            R1_A7_JointIndex.kLeftShoulderYaw.value,
            R1_A7_JointIndex.kLeftElbow.value,
            # Right arm
            R1_A7_JointIndex.kRightShoulderPitch.value,
            R1_A7_JointIndex.kRightShoulderRoll.value,
            R1_A7_JointIndex.kRightShoulderYaw.value,
            R1_A7_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index):
        wrist_motors = [
            R1_A7_JointIndex.kLeftWristRoll.value,
            R1_A7_JointIndex.kLeftWristPitch.value,
            R1_A7_JointIndex.kLeftWristyaw.value,
            R1_A7_JointIndex.kRightWristRoll.value,
            R1_A7_JointIndex.kRightWristPitch.value,
            R1_A7_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors

    def _Is_medium_arm_motor(self, motor_index):
        medium_arm_motors = [
            R1_A7_JointIndex.kLeftShoulderYaw.value,
            R1_A7_JointIndex.kLeftElbow.value,
            R1_A7_JointIndex.kRightShoulderYaw.value,
            R1_A7_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in medium_arm_motors

class R1_A7_JointArmIndex(IntEnum):
    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

class R1_A7_JointHeadIndex(IntEnum):
    kHeadPitch = 29
    kHeadYaw = 30

class R1_A7_JointIndex(IntEnum):
    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistRollNotUsed = 12
    kWaistYaw = 13
    kWaistPitchNotUsed = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

    # Head
    kHeadPitch = 29
    kHeadYaw = 30

    # not used
    kNotUsedJoint0 = 31
    kNotUsedJoint1 = 32
    kNotUsedJoint2 = 33
    kNotUsedJoint3 = 34

if __name__ == "__main__":
    from robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
    import pinocchio as pin

    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation

    # arm_ik = G1_29_ArmIK(Unit_Test = True, Visualization = False)
    # arm = G1_29_ArmController(simulation_mode=True)
    # arm_ik = G1_23_ArmIK(Unit_Test = True, Visualization = False)
    # arm = G1_23_ArmController()
    # arm_ik = H1_2_ArmIK(Unit_Test = True, Visualization = False)
    # arm = H1_2_ArmController()
    # arm_ik = H1_ArmIK(Unit_Test = True, Visualization = True)
    # arm = H1_ArmController()
    arm_ik = H2_ArmIK(Unit_Test = True, Visualization = False)
    arm = H2_ArmController()


    # initial positon
    L_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, +0.25, 0.1]),
    )

    R_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, -0.25, 0.1]),
    )

    rotation_speed = 0.005  # Rotation speed in radians per iteration

    user_input = input("Please enter the start signal (enter 's' to start the subsequent program): \n")
    if user_input.lower() == 's':
        step = 0
        while True:
            if step <= 120:
                angle = rotation_speed * step
                L_quat = pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)  # y axis
                R_quat = pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))  # z axis

                L_tf_target.translation += np.array([0.001,  0.001, 0.001])
                R_tf_target.translation += np.array([0.001, -0.001, 0.001])
            else:
                angle = rotation_speed * (240 - step)
                L_quat = pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)  # y axis
                R_quat = pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))  # z axis

                L_tf_target.translation -= np.array([0.001,  0.001, 0.001])
                R_tf_target.translation -= np.array([0.001, -0.001, 0.001])

            L_tf_target.rotation = L_quat.toRotationMatrix()
            R_tf_target.rotation = R_quat.toRotationMatrix()

            current_lr_arm_q  = arm.get_current_dual_arm_q()
            current_lr_arm_dq = arm.get_current_dual_arm_dq()

            sol_q, sol_tauff = arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous, current_lr_arm_q, current_lr_arm_dq)

            arm.ctrl_dual_arm(sol_q, sol_tauff)

            step += 1
            if step > 240:
                step = 0
            time.sleep(0.01)

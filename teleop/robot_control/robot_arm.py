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
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

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

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("[G1_29_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[G1_29_ArmController] Subscribe dds ok.")

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
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0

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
                        weight = self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q
                        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = max(0.0, weight - dt / 2.0)

                    for idx, id in enumerate(G1_29_JointArmIndex):
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
        return self.lowstate_subscriber.Read().mode_machine
    
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
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

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

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("[G1_23_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[G1_23_ArmController] Subscribe dds ok.")

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
        return self.lowstate_subscriber.Read().mode_machine
    
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
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

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

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("[H1_2_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[H1_2_ArmController] Subscribe dds ok.")

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
        return self.lowstate_subscriber.Read().mode_machine
    
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
        self.arm_velocity_limit = ARM_VELOCITY_LIMIT_START
        self.control_dt = 1.0 / 250.0
        self.arm_accel_limit = ARM_ACCEL_LIMIT
        # 上一周期的实际步长，用于加速度限幅；None 表示首周期不限制
        self._last_delta = None
        # 实测控制周期的指数滑动平均，替代理想值 control_dt
        self._dt_ema = self.control_dt
        # 退出时由 go_home 置位，发布线程负责把 arm_sdk 权重平滑降到 0
        self._release_motion_weight = False

        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, go_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, go_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        # initialize subscribe thread
        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            logger_mp.warning("[H1_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[H1_ArmController] Subscribe dds ok.")

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

if __name__ == "__main__":
    from robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
    import pinocchio as pin

    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation

    arm_ik = G1_29_ArmIK(Unit_Test = True, Visualization = False)
    arm = G1_29_ArmController(simulation_mode=True)
    # arm_ik = G1_23_ArmIK(Unit_Test = True, Visualization = False)
    # arm = G1_23_ArmController()
    # arm_ik = H1_2_ArmIK(Unit_Test = True, Visualization = False)
    # arm = H1_2_ArmController()
    # arm_ik = H1_ArmIK(Unit_Test = True, Visualization = True)
    # arm = H1_ArmController()

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
        arm.speed_gradual_max()
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
import json
import sys
import threading
import time
from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber # dds
from unitree_sdk2py.g1.loco import g1_loco_api
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

try:
    from unitree_sdk2py.g1.loco.g1_loco_api import InternalFsmMode
except ImportError: # sdk version without InternalFsmMode
    class InternalFsmMode(IntEnum):
        LAST = 0     # back to the internal mode which was active before user development mode
        PASSIVE = 1  # damping mode
        WALKRUN = 2  # walk-run mode

try: # SportModeState_ idl is available since firmware 1.5.1
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import SportModeState_
except ImportError:
    SportModeState_ = None

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

kTopicSportModeState = "rt/sportmodestate"

# apis which are missing in old sdk2py versions
_API_ID_GET_FSM_ID              = getattr(g1_loco_api, "ROBOT_API_ID_LOCO_GET_FSM_ID", 7001)
_API_ID_GET_FSM_MODE            = getattr(g1_loco_api, "ROBOT_API_ID_LOCO_GET_FSM_MODE", 7002)
_API_ID_SET_SPEED_MODE          = getattr(g1_loco_api, "ROBOT_API_ID_LOCO_SET_SPEED_MODE", 7107)
_API_ID_SWITCH_TO_USER_CTRL     = getattr(g1_loco_api, "ROBOT_API_ID_LOCO_SWITCH_TO_USER_CTRL", 7110)
_API_ID_SWITCH_TO_INTERNAL_CTRL = getattr(g1_loco_api, "ROBOT_API_ID_LOCO_SWITCH_TO_INTERNAL_CTRL", 7111)

kFsmSwitchTimeout = -1 # returned when the robot does not reach the expected state in time


class G1FsmId(IntEnum):
    # modes without balance control
    kZeroTorque = 0    # zero torque
    kDamp = 1          # damping
    kSquat = 2         # position controlled squat
    kSit = 3           # position controlled sit down
    kLockStand = 4     # locked standing
    # modes with balance control
    kLie2StandUp = 702    # lie down -> stand up
    kSquat2StandUp = 706  # squat -> stand up
    kAiSport = 500        # ai sport
    kAiSport3DofWaist = 501 # ai sport, 3dof waist
    kTeleop = 505         # teleop locomotion control
    kWalkRun = 801        # walk-run mode
    kWalkRunNew = 802     # walk-run mode, 29dof device with ai_sport 8.6.x.x or newer

class G1FsmMode(IntEnum):
    kStatic = 0  # static: standing, allowed to switch to other modes
    kDynamic = 1 # dynamic: moving, most of the modes are forbidden

class G1SpeedMode(IntEnum):
    kSpeed1mps = 0    # 1.0 m/s
    kSpeed2mps = 1    # 2.0 m/s
    kSpeed2_7mps = 2  # 2.7 m/s
    kSpeed3mps = 3    # 3.0 m/s

class SportModeState:
    def __init__(self):
        self.fsm_id = 0     # current mode id, see G1FsmId
        self.fsm_mode = 0   # 0: static, 1: dynamic, see G1FsmMode
        self.task_id = 0    # upper limb interaction action id
        self.task_time = 0. # elapsed time of the upper limb action, in seconds

class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data

class G1FsmController:
    '''G1 locomotion control mode(fsm) controller: query state, switch mode, move and take over the robot.'''

    def __init__(self, timeout = 1.0, enable_state_subscriber = True, walk_run_fsm_id = G1FsmId.kWalkRunNew):
        logger_mp.info("Initialize G1FsmController...")
        self.lock = threading.RLock()
        self.walk_run_fsm_id = walk_run_fsm_id
        self.sport_mode_state_buffer = DataBuffer()

        self.client = LocoClient()
        self.client.SetTimeout(timeout)
        self.client.Init()
        for api_id in (_API_ID_SET_SPEED_MODE, _API_ID_SWITCH_TO_USER_CTRL, _API_ID_SWITCH_TO_INTERNAL_CTRL):
            self.client._RegistApi(api_id, 0) # keep working with old sdk2py versions

        self.enable_state_subscriber = enable_state_subscriber and SportModeState_ is not None
        if self.enable_state_subscriber:
            self.sportmodestate_subscriber = ChannelSubscriber(kTopicSportModeState, SportModeState_)
            self.sportmodestate_subscriber.Init()

            # initialize subscribe thread
            self.subscribe_thread = threading.Thread(target=self._subscribe_sport_mode_state)
            self.subscribe_thread.daemon = True
            self.subscribe_thread.start()

            while not self.sport_mode_state_buffer.GetData():
                time.sleep(0.1)
                logger_mp.warning("[G1FsmController] Waiting to subscribe dds...")
            logger_mp.info("[G1FsmController] Subscribe dds ok.")
        else:
            logger_mp.warning("[G1FsmController] rt/sportmodestate is disabled.")

        logger_mp.info("Initialize G1FsmController OK!")

    def _subscribe_sport_mode_state(self):
        while True:
            msg = self.sportmodestate_subscriber.Read()
            if msg is not None:
                sport_mode_state = SportModeState()
                sport_mode_state.fsm_id = msg.fsm_id
                sport_mode_state.fsm_mode = msg.fsm_mode
                sport_mode_state.task_id = msg.task_id
                sport_mode_state.task_time = msg.task_time
                self.sport_mode_state_buffer.SetData(sport_mode_state)
            time.sleep(0.002)

    def _call(self, api_id, parameter = "{}"):
        '''Send a raw rpc request and return (code, data). The caller must hold self.lock.'''
        return self.client._Call(api_id, parameter)

    def get_sport_mode_state(self):
        '''Return the latest state published on rt/sportmodestate, None if the topic is disabled.'''
        return self.sport_mode_state_buffer.GetData()

    def _rpc_get_fsm_id(self):
        '''Query the current mode id by rpc. Return (code, fsm_id). The caller must hold self.lock.'''
        if hasattr(self.client, "GetFsmId"):
            try:
                return 0, int(self.client.GetFsmId())
            except (TypeError, ValueError):
                return -1, None
        code, data = self._call(_API_ID_GET_FSM_ID)
        if code != 0 or data is None:
            return code, None
        try:
            return 0, int(json.loads(data)["data"])
        except (ValueError, TypeError, KeyError):
            return code, None

    def _rpc_get_fsm_mode(self):
        '''Query the current mode state by rpc. Return (code, fsm_mode). The caller must hold self.lock.'''
        if hasattr(self.client, "GetFsmMode"):
            try:
                return 0, int(self.client.GetFsmMode())
            except (TypeError, ValueError):
                return -1, None
        code, data = self._call(_API_ID_GET_FSM_MODE)
        if code != 0 or data is None:
            return code, None
        try:
            return 0, int(json.loads(data)["data"])
        except (ValueError, TypeError, KeyError):
            return code, None

    def get_fsm_id(self):
        '''Return current mode id, see G1FsmId. None if the query failed.'''
        with self.lock:
            code, fsm_id = self._rpc_get_fsm_id()
        if fsm_id is not None:
            return fsm_id
        sport_mode_state = self.get_sport_mode_state()
        if sport_mode_state is not None:
            return sport_mode_state.fsm_id
        logger_mp.warning(f"[G1FsmController] GetFsmId failed, code: {code}")
        return None

    def get_fsm_mode(self):
        '''Return current mode state: 0 static(standing), 1 dynamic(moving), see G1FsmMode.
        None if the query failed.'''
        sport_mode_state = self.get_sport_mode_state()
        if sport_mode_state is not None:
            return sport_mode_state.fsm_mode
        with self.lock:
            code, fsm_mode = self._rpc_get_fsm_mode()
        if fsm_mode is not None:
            return fsm_mode
        logger_mp.warning(f"[G1FsmController] GetFsmMode failed, code: {code}")
        return None

    def get_task_state(self):
        '''Return (task_id, task_time) of the running upper limb interaction action.'''
        sport_mode_state = self.get_sport_mode_state()
        if sport_mode_state is None:
            return None, 0.
        return sport_mode_state.task_id, sport_mode_state.task_time

    def wait_fsm_id(self, fsm_id, timeout = 5.0, old_fsm_id = None):
        '''Wait until the robot reports the target mode id. If old_fsm_id is given, any change also succeeds.'''
        fsm_id = int(fsm_id)
        deadline = time.time() + timeout
        while time.time() < deadline:
            current_fsm_id = self.get_fsm_id()
            if current_fsm_id is not None:
                if current_fsm_id == fsm_id or (old_fsm_id is not None and current_fsm_id != old_fsm_id):
                    return True
            time.sleep(0.05)
        return False

    def wait_fsm_mode(self, fsm_mode = G1FsmMode.kStatic, timeout = 5.0):
        '''Wait until the robot reports the target mode state, see G1FsmMode.'''
        fsm_mode = int(fsm_mode)
        deadline = time.time() + timeout
        while time.time() < deadline:
            current_fsm_mode = self.get_fsm_mode()
            if current_fsm_mode == fsm_mode:
                return True
            time.sleep(0.05)
        return False

    def wait_fsm_static(self, timeout = 5.0):
        '''Wait until the robot is static, then it is allowed to switch to other modes.'''
        return self.wait_fsm_mode(G1FsmMode.kStatic, timeout)

    def set_fsm_id(self, fsm_id):
        '''Switch to the target mode id without any state check. Return 0 on success, otherwise the error code.'''
        fsm_id = int(fsm_id)
        with self.lock:
            code = self.client.SetFsmId(fsm_id)
        if code != 0:
            logger_mp.warning(f"[G1FsmController] SetFsmId {fsm_id} failed, code: {code}")
        return code

    def switch_mode(self, fsm_id, wait = True, timeout = 5.0, wait_static = True):
        '''Switch to the target mode id, damping is always allowed while other switches require a static robot.
        Return 0 on success, otherwise the error code, kFsmSwitchTimeout if the robot was not ready in time.'''
        fsm_id = int(fsm_id)
        if wait_static and fsm_id != G1FsmId.kDamp:
            if not self.wait_fsm_static(timeout):
                logger_mp.warning(f"[G1FsmController] Robot is not static, cancel switching to {fsm_id}.")
                return kFsmSwitchTimeout

        old_fsm_id = self.get_fsm_id() if wait else None
        code = self.set_fsm_id(fsm_id)
        if code != 0 or not wait:
            return code

        if not self.wait_fsm_id(fsm_id, timeout, old_fsm_id):
            logger_mp.warning(f"[G1FsmController] Switch to fsm id {fsm_id} timeout.")
            return kFsmSwitchTimeout
        logger_mp.info(f"[G1FsmController] Switch to fsm id {fsm_id} ok.")
        return 0

    def set_velocity(self, vx, vy, omega, duration = 1.0):
        '''Set velocity command: vx forward, vy lateral, omega yaw, duration is the command duration.
        The program clips the command to the allowed range. Return 0 on success, otherwise the error code.'''
        with self.lock:
            code = self.client.SetVelocity(vx, vy, omega, duration)
        if code != 0:
            logger_mp.warning(f"[G1FsmController] SetVelocity failed, code: {code}")
        return code

    def move(self, vx, vy, vyaw, continuous_move = False):
        '''Move the robot. If continuous_move is False, the command only lasts one second.'''
        duration = 864000.0 if continuous_move else 1.0
        return self.set_velocity(vx, vy, vyaw, duration)

    def stop_move(self):
        '''Stop the robot.'''
        return self.set_velocity(0., 0., 0.)

    def set_speed_mode(self, speed_mode):
        '''Set the max speed of walk-run mode, see G1SpeedMode. Return 0 on success, otherwise the error code.'''
        speed_mode = int(speed_mode)
        with self.lock:
            if hasattr(self.client, "SetSpeedMode"):
                code = self.client.SetSpeedMode(speed_mode)
            else:
                code, _ = self._call(_API_ID_SET_SPEED_MODE, json.dumps({"data": speed_mode}))
        if code != 0:
            logger_mp.warning(f"[G1FsmController] SetSpeedMode {speed_mode} failed, code: {code}")
        return code

    def switch_to_user_ctrl(self):
        '''Switch to user development mode, all the motors are taken over by topic rt/user_lowcmd.
        The first and the last action of the user controller must be the standard standing pose.
        Return 0 on success, otherwise the error code.'''
        with self.lock:
            if hasattr(self.client, "SwitchToUserCtrl"):
                code = self.client.SwitchToUserCtrl()
            else:
                code, _ = self._call(_API_ID_SWITCH_TO_USER_CTRL)
        if code != 0:
            logger_mp.warning(f"[G1FsmController] SwitchToUserCtrl failed, code: {code}")
        return code

    def switch_to_internal_ctrl(self, mode = InternalFsmMode.LAST):
        '''Switch from user development mode back to the internal locomotion control, see InternalFsmMode.
        Return 0 on success, otherwise the error code.'''
        mode = int(InternalFsmMode(mode))
        with self.lock:
            if hasattr(self.client, "SwitchToInternalCtrl"):
                code = self.client.SwitchToInternalCtrl(InternalFsmMode(mode))
            else:
                code, _ = self._call(_API_ID_SWITCH_TO_INTERNAL_CTRL, json.dumps({"data": mode}))
        if code != 0:
            logger_mp.warning(f"[G1FsmController] SwitchToInternalCtrl {mode} failed, code: {code}")
        return code

    def zero_torque(self, wait = True, timeout = 5.0):
        '''Enter zero torque mode, no balance control.'''
        return self.switch_mode(G1FsmId.kZeroTorque, wait, timeout)

    def damp(self, wait = True, timeout = 5.0):
        '''Enter damping mode, no balance control. It is always allowed as the last resort mode.'''
        return self.switch_mode(G1FsmId.kDamp, wait, timeout)

    def squat(self, wait = True, timeout = 5.0):
        '''Enter position controlled squat mode, no balance control.'''
        return self.switch_mode(G1FsmId.kSquat, wait, timeout)

    def sit(self, wait = True, timeout = 5.0):
        '''Enter position controlled sit down mode, no balance control.'''
        return self.switch_mode(G1FsmId.kSit, wait, timeout)

    def lock_stand(self, wait = True, timeout = 5.0):
        '''Enter locked standing mode, no balance control.'''
        return self.switch_mode(G1FsmId.kLockStand, wait, timeout)

    def lie2_stand_up(self, wait = True, timeout = 10.0):
        '''Stand the robot up from the sitting or squatting pose.'''
        return self.switch_mode(G1FsmId.kLie2StandUp, wait, timeout)

    def squat2_stand_up(self, wait = True, timeout = 10.0):
        '''Squat down and stand the robot up from damping mode.'''
        return self.switch_mode(G1FsmId.kSquat2StandUp, wait, timeout)

    def stand_up(self, wait = True, timeout = 10.0):
        '''Stand the robot up, the motion is selected according to the current mode id.'''
        if self.get_fsm_id() in (G1FsmId.kSit, G1FsmId.kSquat):
            return self.lie2_stand_up(wait, timeout)
        return self.squat2_stand_up(wait, timeout)

    def ai_sport(self, wait = True, timeout = 5.0):
        '''Enter ai sport mode, i.e. the conventional locomotion control.'''
        return self.switch_mode(G1FsmId.kAiSport, wait, timeout)

    def teleop(self, wait = True, timeout = 5.0):
        '''Enter the teleop locomotion control mode used for teleoperation.'''
        return self.switch_mode(G1FsmId.kTeleop, wait, timeout)

    def walk_run(self, speed_mode = None, wait = True, timeout = 5.0):
        '''Enter walk-run mode and optionally set its max speed, see G1SpeedMode.'''
        code = self.switch_mode(self.walk_run_fsm_id, wait, timeout)
        if code == 0 and speed_mode is not None:
            code = self.set_speed_mode(speed_mode)
        return code

    def _print_state(self):
        sport_mode_state = self.get_sport_mode_state()
        if sport_mode_state is None:
            print(f"fsm_id: {self.get_fsm_id()}, fsm_mode: {self.get_fsm_mode()}")
            return
        fsm_id_name = G1FsmId(sport_mode_state.fsm_id).name if sport_mode_state.fsm_id in tuple(G1FsmId) else "unknown"
        print(f"fsm_id: {sport_mode_state.fsm_id}({fsm_id_name}), "
              f"fsm_mode: {sport_mode_state.fsm_mode}, "
              f"task_id: {sport_mode_state.task_id}, task_time: {sport_mode_state.task_time:.2f}")

if __name__ == "__main__":
    network_interface = sys.argv[1] if len(sys.argv) > 1 else None
    if network_interface is None:
        ChannelFactoryInitialize(0) # 0 for real robot, 1 for simulation
    else:
        ChannelFactoryInitialize(0, networkInterface=network_interface) # e.g. python robot_fsm.py eth0

    fsm = G1FsmController()
    commands = {
        "zero": fsm.zero_torque,
        "damp": fsm.damp,
        "squat": fsm.squat,
        "sit": fsm.sit,
        "lock": fsm.lock_stand,
        "stand": fsm.stand_up,
        "squat2stand": fsm.squat2_stand_up,
        "lie2stand": fsm.lie2_stand_up,
        "ai": fsm.ai_sport,
        "teleop": fsm.teleop,
        "walkrun": fsm.walk_run,
        "user": fsm.switch_to_user_ctrl,
        "internal": fsm.switch_to_internal_ctrl,
    }
    print(f"commands: {list(commands.keys())}, speed <mode>, <fsm_id>, help, quit")
    user_input = input("Please enter the start signal (enter 's' to start the subsequent program): \n")
    if user_input.lower() == 's':
        while True:
            command = input("Please enter the command: \n")
            if command in commands:
                commands[command]()
            elif command.startswith("speed"):
                fsm.set_speed_mode(int(command.split()[-1]))
            elif command.isdigit():
                fsm.switch_mode(int(command)) # raw fsm id, e.g. 505
            elif command == "help":
                for name, fsm_id in G1FsmId.__members__.items():
                    print(f"    {name}: {fsm_id.value}")
            elif command == "quit":
                break
            else:
                print(f"[{command}] is not a valid command, enter 'help' to list all the fsm ids.")
            fsm._print_state()

import math

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.toyota.values import ToyotaFlags

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

#新処理をTSS2で使用
# EU guidelines
dc_get_lag_adjusted_curvature = False
CT_get_lag_adjusted_curvature = 0
MAX_LATERAL_JERK = 5.0

#k_vs_47700 =     [1.0, 0.96, 0.92, 0.92 , 0.92] #47700用減少補正。ツインウェイブ曲がれた。ちょっと内寄り気味
#k_vs_47700 =     [1.0, 0.96, 0.92, 0.91 , 0.91] #47700用減少補正。
#k_vs_47700 =     [1.0, 0.96, 0.92, 0.87 , 0.85] #47700用減少補正。参考、ツインウェイブ曲がれない。後半オーバーする
#k_vs_org_47700 = [0  , 0.01, 0.02, 0.035, 0.05]
k_vs_47700 =     [1.0, 0.96, 0.92, 0.91 , 0.90, 0.85 ] #47700用減少補正。
k_vs_org_47700 = [0  , 0.01, 0.02, 0.035, 0.05, 0.075] #ツインウェイブ終盤オーバーステア対策
with open('/tmp/curvature_info.txt','w') as fp:
  fp.write('%.9f/%.3f' % (0 , 1.0))

skip_curvature_info = False

MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def clip_curvature(v_ego, prev_curvature, new_curvature , CP):

  global CT_get_lag_adjusted_curvature,dc_get_lag_adjusted_curvature
  flag_eps_TSS2 = CP.flags & ToyotaFlags.POWER_STEERING_TSS2.value
  if flag_eps_TSS2 and CT_get_lag_adjusted_curvature % 100 == 51:
    try:
      with open('/tmp/knight_scanner_bit3.txt','r') as fp: #ナイトスキャナーボタン ⚫︎⚪︎⚪︎ で有効
        knight_scanner_bit3_str = fp.read()
        if knight_scanner_bit3_str:
          knight_scanner_bit3 = int(knight_scanner_bit3_str)
          if (knight_scanner_bit3 & 0x01) != 0: #1ビット（"⚫︎⚪︎⚪︎"）が点灯で舵力抑制発動
            dc_get_lag_adjusted_curvature = True
          else:
            dc_get_lag_adjusted_curvature = False
    except Exception as e:
      dc_get_lag_adjusted_curvature = True #デフォルト
  CT_get_lag_adjusted_curvature += 1

  k_v = 1.0
  org_desired_curvature = new_curvature
  if flag_eps_TSS2 and dc_get_lag_adjusted_curvature == True:
    #自分だけのスペシャル処理
    k_v = interp(abs(new_curvature) , k_vs_org_47700 , k_vs_47700)
    new_curvature *= k_v

  if CT_get_lag_adjusted_curvature % 10 == 7 and skip_curvature_info == False: #書き出し頻度を1/10に
    try:
      with open('/tmp/curvature_info.txt','w') as fp:
        fp.write('%.9f/%.3f' % (org_desired_curvature if v_ego > 0.1/3.6 else 0 , k_v))
    except Exception as e:
      pass

  v_ego = max(MIN_SPEED, v_ego)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = clip(new_curvature,
                                prev_curvature - max_curvature_rate * DT_CTRL,
                                prev_curvature + max_curvature_rate * DT_CTRL)

  return safe_desired_curvature


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0

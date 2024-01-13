import math

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.toyota.values import ToyotaFlags

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = 1.6  # should be CV.MPH_TO_KPH, but this causes rounding errors

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

#新処理をTSS2で使用
# EU guidelines
tss_type = 0
dc_get_lag_adjusted_curvature = False
CT_get_lag_adjusted_curvature = 0
MAX_LATERAL_JERK = 5.0
# this corresponds to 80deg/s and 20deg/s steering angle in a toyota corolla
#MAX_CURVATURE_RATES = [0.03762194918267951, 0.003441203371932992]
#MAX_CURVATURE_RATES = [0.03762194918267951 * 2.7, 0.03762194918267951 * 1.0] #藤沢警察署前Y字路カーブ、キコーナ前上りカーブ、養命寺横カーブ、吹上下り走行車線成功,どこまで上がる？,低速域の限界を上げてみる。
# MAX_CURVATURE_RATES_0 = [0.03762194918267951, 0.03762194918267951 * 0.8] #最初の係数を機械推論反映値として計算する（1〜2.7）
# MAX_CURVATURE_RATE_SPEEDS = [0, 35]

# k_vs =     [1.02, 1.029, 1.06 , 1.10  , 1.14  , 1.19 , 1.24 ] #desired_curvatureでinterpする。1.15定数倍で保土ヶ谷出口32度回ってる。
# k_vs_org = [0   , 0.004, 0.006, 0.0085, 0.0095, 0.014, 0.021]
# k_vs =     [1.00, 1.00, 1.00] #TSS2と同じ、公式の値に介入しない(ロングノイズモデルから1.0で。こんどこそ要らない？)
# k_vs_org = [0   , 0.01, 0.05]
# k2_vs =     [1.0, 1.0  , 1.0] #TSS2用減少補正。 舵力減少させないでテスト。
# k2_vs_org = [0  , 0.033, 0.05]
k_vs_47700 =     [1.0, 0.95, 0.85, 0.80 ] #47700用減少補正。
k_vs_org_47700 = [0  , 0.01, 0.02, 0.035]
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


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired
  desired_curvature_rate = curvature_rates[0]

  global tss_type,CT_get_lag_adjusted_curvature,dc_get_lag_adjusted_curvature
  if tss_type == 0:
    try:
      with open('../../../tss_type_info.txt','r') as fp:
        tss_type_str = fp.read()
        if tss_type_str:
          if int(tss_type_str) == 2: #TSS2
            tss_type = 2
          elif int(tss_type_str) == 1: #TSSP
            tss_type = 1
    except Exception as e:
      pass

  flag_47700 = CP.flags & ToyotaFlags.POWER_STEERING_47700.value

  if flag_47700 and CT_get_lag_adjusted_curvature % 100 == 51:
    try:
      with open('/tmp/knight_scanner_bit3.txt','r') as fp: #ひとまずナイトスキャナーボタンに連動
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
  org_desired_curvature = desired_curvature
  if flag_47700 and dc_get_lag_adjusted_curvature == True:
    #自分だけのスペシャル処理
    k_v = interp(abs(desired_curvature) , k_vs_org_47700 , k_vs_47700)
    desired_curvature_rate *= k_v
    desired_curvature *= k_v

  if CT_get_lag_adjusted_curvature % 10 == 7 and skip_curvature_info == False: #書き出し頻度を1/10に
    try:
      with open('/tmp/curvature_info.txt','w') as fp:
        fp.write('%.9f/%.3f' % (org_desired_curvature , k_v))
    except Exception as e:
      pass

  # This is the "desired rate of the setpoint" not an actual desired rate
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                     -max_curvature_rate,
                                     max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


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

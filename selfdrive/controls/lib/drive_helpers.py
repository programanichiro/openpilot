import math
import os
from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
V_CRUISE_MAX = 145  # kph
V_CRUISE_MIN = 8  # kph
V_CRUISE_ENABLE_MIN = 40  # kph
V_CRUISE_INITIAL = 255  # kph

LAT_MPC_N = 16
LON_MPC_N = 32
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
MAX_CURVATURE_RATES_0 = [0.03762194918267951, 0.03762194918267951 * 0.8] #最初の係数を機械推論反映値として計算する（1〜2.7）
MAX_CURVATURE_RATE_SPEEDS = [0, 35]

k_vs =     [1.02, 1.029, 1.06 , 1.10  , 1.14  , 1.19 , 1.24 ] #desired_curvatureでinterpする。1.15定数倍で保土ヶ谷出口32度回ってる。
k_vs_org = [0   , 0.004, 0.006, 0.0085, 0.0095, 0.014, 0.021]
k2_vs =     [1.0, 1.0  , 0.92] #TSS2用減少補正。
k2_vs_org = [0  , 0.033, 0.05]
with open('/tmp/curvature_info.txt','w') as fp:
  fp.write('%.9f/%.3f' % (0 , 1.0))

skip_curvature_info = False
# if os.environ['DONGLE_ID'] in ('cdcb457f7528673b'):
#   skip_curvature_info = True #curvature_infoを送るとエラーが起きる人対策。

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


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def update_v_cruise(v_cruise_kph, v_ego, gas_pressed, buttonEvents, button_timers, enabled, metric):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  if not enabled:
    return v_cruise_kph

  long_press = False
  button_type = None

  # should be CV.MPH_TO_KPH, but this causes rounding errors
  v_cruise_delta = 1. if metric else 1.6

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph  # end long press
      button_type = b.type.raw
      break
  else:
    for k in button_timers.keys():
      if button_timers[k] and button_timers[k] % CRUISE_LONG_PRESS == 0:
        button_type = k
        long_press = True
        break

  if button_type:
    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and v_cruise_kph % v_cruise_delta != 0:  # partial interval
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if gas_pressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      v_cruise_kph = max(v_cruise_kph, v_ego * CV.MS_TO_KPH)

    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(v_ego, 0.1)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired
  desired_curvature_rate = curvature_rates[0]

  global tss_type,CT_get_lag_adjusted_curvature,dc_get_lag_adjusted_curvature
  if tss_type == 0:
    try:
      with open('./tss_type_info.txt','r') as fp:
        tss_type_str = fp.read()
        if tss_type_str:
          if int(tss_type_str) == 2: #TSS2
            tss_type = 2
            dc_get_lag_adjusted_curvature = True
          elif int(tss_type_str) == 1: #TSSP
            tss_type = 1
    except Exception as e:
      pass

  if CT_get_lag_adjusted_curvature % 100 == 51:
    try:
      with open('/tmp/handle_ctrl_disable.txt','r') as fp:
        dcm_handle_ctrl_disable_str = fp.read()
        if dcm_handle_ctrl_disable_str:
          dcm_handle_ctrl_disable = int(dcm_handle_ctrl_disable_str)
          if dcm_handle_ctrl_disable == 0:
            dc_get_lag_adjusted_curvature = False
          else:
            dc_get_lag_adjusted_curvature = True
    except Exception as e:
      dc_get_lag_adjusted_curvature = False if tss_type < 2 else True #tss2ならデフォがTrue
  CT_get_lag_adjusted_curvature += 1
    
  if dc_get_lag_adjusted_curvature == True:
    #新処理をTSS2で使用。公式状態。
    # This is the "desired rate of the setpoint" not an actual desired rate
    if CT_get_lag_adjusted_curvature % 10 == 3 and skip_curvature_info == False: #書き出し頻度を1/10に
      try:
        with open('/tmp/curvature_info.txt','w') as fp:
          fp.write('%.9f/%.3f' % (desired_curvature , 1.0))
      except Exception as e:
        pass
    max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
    safe_desired_curvature_rate = clip(desired_curvature_rate,
                                            -max_curvature_rate,
                                            max_curvature_rate)
    safe_desired_curvature = clip(desired_curvature,
                                      current_curvature_desired - max_curvature_rate * DT_MDL,
                                      current_curvature_desired + max_curvature_rate * DT_MDL)
  else:
    # k_v_tss = interp(abs(desired_curvature) , k_vs_org , k_vs)
    # k_v_tss2 = interp(abs(desired_curvature) , k2_vs_org , k2_vs)
    k_v = interp(abs(desired_curvature) , k_vs_org , k_vs) if tss_type < 2 else interp(abs(desired_curvature) , k2_vs_org , k2_vs)
    #k_v = 1.15 if tss_type < 2 else interp(abs(desired_curvature) , k2_vs_org , k2_vs)
    # with open('/tmp/debug_out_y','w') as fp:
    #   fp.write('kv:%.2f , dc:%.5f' % (k_v , desired_curvature*100))
    if CT_get_lag_adjusted_curvature % 10 == 7 and skip_curvature_info == False: #書き出し頻度を1/10に
      try:
        with open('/tmp/curvature_info.txt','w') as fp:
          fp.write('%.9f/%.3f' % (desired_curvature , k_v))
      except Exception as e:
        pass
    k_v2 = 4 if tss_type < 2 else 2 #tss2なら2,tsspでは試行中。
    max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) *k_v2 # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
    safe_desired_curvature_rate = clip(desired_curvature_rate*k_v,
                                            -max_curvature_rate,
                                            max_curvature_rate)
    safe_desired_curvature = clip(desired_curvature*k_v,
                                      current_curvature_desired - max_curvature_rate * DT_MDL,
                                      current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature, safe_desired_curvature_rate

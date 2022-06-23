import math
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from selfdrive.config import Conversions as CV
from selfdrive.modeld.constants import T_IDXS

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
V_CRUISE_MAX = 145  # kph
V_CRUISE_MIN = 8  # kph
V_CRUISE_ENABLE_MIN = 40  # kph

LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# this corresponds to 80deg/s and 20deg/s steering angle in a toyota corolla
#MAX_CURVATURE_RATES = [0.03762194918267951, 0.003441203371932992]
#MAX_CURVATURE_RATES = [0.03762194918267951 * 2.7, 0.03762194918267951 * 1.0] #藤沢警察署前Y字路カーブ、キコーナ前上りカーブ、養命寺横カーブ、吹上下り走行車線成功,どこまで上がる？,低速域の限界を上げてみる。
MAX_CURVATURE_RATES_0 = [0.03762194918267951, 0.03762194918267951 * 0.8] #最初の係数を機械推論反映値として計算する（1〜2.7）
MAX_CURVATURE_RATE_SPEEDS = [0, 35]

CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  car.CarState.ButtonEvent.Type.accelCruise: math.ceil,
  car.CarState.ButtonEvent.Type.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  car.CarState.ButtonEvent.Type.accelCruise: +1,
  car.CarState.ButtonEvent.Type.decelCruise: -1,
}


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


class MPC_COST_LONG:
  TTC = 5.0
  DISTANCE = 0.1
  ACCELERATION = 10.0
  JERK = 20.0


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def update_v_cruise(v_cruise_kph, buttonEvents, button_timers, enabled, metric):
  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  if not enabled:
    return v_cruise_kph

  long_press = False
  button_type = None

  v_cruise_delta = 1 if metric else 1.6

  for b in buttonEvents:
    if b.type.raw in button_timers and not b.pressed:
      if button_timers[b.type.raw] > CRUISE_LONG_PRESS:
        return v_cruise_kph # end long press
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
    if long_press and v_cruise_kph % v_cruise_delta != 0: # partial interval
      v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]
    v_cruise_kph = clip(round(v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type == car.CarState.ButtonEvent.Type.accelCruise and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))


def get_lag_adjusted_curvature(CP, v_ego, steerAng , psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0 for i in range(CONTROL_N)]
    curvatures = [0.0 for i in range(CONTROL_N)]
    curvature_rates = [0.0 for i in range(CONTROL_N)]

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2
  current_curvature = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  desired_curvature_rate = curvature_rates[0]

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  curvature_diff_from_psi = psi / (max(v_ego, 1e-1) * delay) - current_curvature
  desired_curvature = current_curvature + 2 * curvature_diff_from_psi

  abs_sta = abs(steerAng) / 10
  if abs_sta > 1:
    abs_sta = 1 # abs_sta:0〜1
  MAX_CURVATURE_RATES_k = 1.1 + (2.7 - 1) * abs_sta #1.1〜2.7+0.1
  MAX_CURVATURE_RATES = [MAX_CURVATURE_RATES_0[0]*MAX_CURVATURE_RATES_k , MAX_CURVATURE_RATES_0[1]]
  max_curvature_rate = interp(v_ego, MAX_CURVATURE_RATE_SPEEDS, MAX_CURVATURE_RATES)
  vv2 = v_ego if v_ego >= 31/3.6 else 31/3.6 #この速度(31km/h)以下はk_vが上がらないようにする
  #abs(steerAng):0〜10→1〜1.9
  max_k_v = 2.0
  max_k_v = 1.0 + (max_k_v - 1) * abs_sta #max_k_v = 1.0〜max_k_v , ひとまずハンドル（前方カーブ予測含む）が10度で最大値になる。
  k_v = 1.0 if vv2 >= 75/3.6 else 1+ (1 - vv2 / (75/3.6))*(max_k_v-1) # 1〜0 -> 1〜max_k_v(75km/h以上はk_v=1)
  #with open('./debug_out_k','w') as fp:
  #  fp.write('k_v:%.2f , steerAng:%.2f' % (k_v , steerAng))
  safe_desired_curvature_rate = clip(desired_curvature_rate *k_v,
                                          -max_curvature_rate,
                                          max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature *k_v,
                                     current_curvature - max_curvature_rate / DT_MDL, #0.8.13で掛けられていたが、ハンドルが甘くなった気がするので悪に戻す。
                                     current_curvature + max_curvature_rate / DT_MDL)
  return safe_desired_curvature, safe_desired_curvature_rate

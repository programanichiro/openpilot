from cereal import log
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

#新処理をTSS2で使用
# EU guidelines
# dc_get_lag_adjusted_curvature = False
# CT_get_lag_adjusted_curvature = 0
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

def clip_curvature(v_ego, prev_curvature, new_curvature):
  v_ego = max(MIN_SPEED, v_ego)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = clip(new_curvature,
                                prev_curvature - max_curvature_rate * DT_CTRL,
                                prev_curvature + max_curvature_rate * DT_CTRL)

  return safe_desired_curvature


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0


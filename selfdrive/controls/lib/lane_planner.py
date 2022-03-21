import os
import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog

STEER_SAME_DIRECTION_CT = 0
STEER_OLD_ANGLE = 0
STEERING_CENTER = -4.3
DCM_FRAME = 0
dcm_handle_ctrl = True
if os.path.isfile('./handle_center_info.txt'):
  with open('./handle_center_info.txt','r') as fp:
    handle_center_info_str = fp.read()
    if handle_center_info_str:
      STEERING_CENTER = float(handle_center_info_str)

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of EON's camera. TICI is 0.1 m away,
# however the average measured path difference is 0.04 m
if EON:
  CAMERA_OFFSET = -0.05 #0.06
  PATH_OFFSET = 0.0
elif TICI:
  CAMERA_OFFSET = 0.04
  PATH_OFFSET = 0.04
else:
  CAMERA_OFFSET = 0.0
  PATH_OFFSET = 0.0


class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

  def parse_model(self, md):
    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, st_angle, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(3.5, self.lane_width) #4.0
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    prob_limit_angle = 6 #これよりハンドル角が大きい時で、カーブのアウト側がインより薄い認識だと、アウト側を無視してみる
    if st_angle < -prob_limit_angle:
      if r_prob > 0.5 and r_prob*0.8 > l_prob:
        l_prob = 0
    elif st_angle > prob_limit_angle:
      if l_prob > 0.5 and l_prob*0.8 > r_prob:
        r_prob = 0
    dcm = self.calc_dcm(st_angle, v_ego,clipped_lane_width,l_prob,r_prob)
    path_from_left_lane -= dcm
    path_from_right_lane -= dcm
    path_xyz[:,1] -= dcm

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz

#関数を最後に追加,dcm(ダイナミックカメラマージン？)名前がおかしいが、コーナーのイン側に寄せるオフセットである。
  def calc_dcm(self, st_angle, v_ego,clipped_lane_width,l_prob,r_prob):
    #数値を実際に取得して、調整してみる。UIスイッチで車体寄せをやめるなら、ここでゼロを返せばいい。
    global DCM_FRAME , dcm_handle_ctrl
    if DCM_FRAME % 30 == 1:
      if os.path.isfile('./handle_ctrl_disable.txt'):
        with open('./handle_ctrl_disable.txt','r') as fp:
          dcm_handle_ctrl_disable_str = fp.read()
          if dcm_handle_ctrl_disable_str:
            dcm_handle_ctrl_disable = int(dcm_handle_ctrl_disable_str)
            if dcm_handle_ctrl_disable == 0:
              dcm_handle_ctrl = True
            else:
              dcm_handle_ctrl = False
      else:
        dcm_handle_ctrl = True
    DCM_FRAME += 1
    if dcm_handle_ctrl == False:
      return 0 #車体寄せを行わない

    handle_margin = 1 #1.5
    handle_over = 10
    camera_margin = 0.1 #0.05 -> 0.1
    dcm = 0
    mdcm = 1.2
    w_add = 0
    global STEER_SAME_DIRECTION_CT
    global STEER_OLD_ANGLE
    if (STEER_OLD_ANGLE) * (st_angle) > 0:
      STEER_SAME_DIRECTION_CT += 1
    else:
      STEER_SAME_DIRECTION_CT = 0
    STEER_OLD_ANGLE = st_angle
    if v_ego > 60/3.6: # 60 or 70km/h over
      handle_margin = 1.5
      if STEER_SAME_DIRECTION_CT > 70 and clipped_lane_width - 2.5 >= 0:  #2.5 <- 1.9=prius width
        w_add = (clipped_lane_width - 2.5)  * 0.8 / 2.0
    if st_angle > handle_margin:
      dcm = 0.01 - CAMERA_OFFSET + camera_margin
      dcm += w_add * 1.1 / 1.2
      dcm *= min((st_angle -(handle_margin)) / handle_over,1.2)
    if st_angle < -handle_margin:
      dcm = -0.11 - CAMERA_OFFSET - camera_margin
      dcm -= w_add * 0.8 / 1.2 #減速と合わせると相当寄りすぎなので小さく
      dcm *= min(-(st_angle +(handle_margin)) / handle_over,1.2)
#🟥🟥🟥🟥🟥🟥🟥
    if False: #デバッグ表示なし。
      ms = "O:%+.2f" % (dcm)
      if dcm >= 0.01:
        ms+= "<"
        for vml in range(int(min(dcm*100-1,30))):
          ms+= "-"
      if clipped_lane_width >= 2.5:
        for vml in range(int(min((clipped_lane_width-2.5)*50,50))):
          ms+= "="
      if dcm <= -0.01:
        for vml in range(int(min(-dcm*100-1,30))):
          ms+= "-"
        ms+= ">"
      ms += "W:%.2f" % (clipped_lane_width)
      ms += ",ct:%d;%.2f,%.2f" % (min(STEER_SAME_DIRECTION_CT,99),l_prob,r_prob)
      #with open('./debug_out_2','w') as fp:
      #  #fp.write('l:{0}\n'.format(['%0.2f' % i for i in path_from_left_lane]))
      #  #fp.write('r:{0}\n'.format(['%0.2f' % i for i in path_from_right_lane]))
      #  #fp.write('ofst:%0.2f[m] , lane_w:%0.2f[m], ct:%d' % (dcm , clipped_lane_width,STEER_SAME_DIRECTION_CT))
      #  #fp.write('OFS:%+.2f,w:%.2f[m],ct:%d' % (dcm , clipped_lane_width,min(STEER_SAME_DIRECTION_CT,99)))
      #  fp.write(ms)
    #if self.camera_offset * CAMERA_OFFSET < 0: #Consider wide_cameraこれ不要。ワイドカメラがメインカメラの反対についているだけで、方向が反対になるわけではない。
    #  dcm = -dcm
#    if r_prob == -1 and l_prob == -1: #ない方がいいかもしれん。取ると車体が右による？。想定と逆
#      dcm -= self.camera_offset #レーンレスモデル用のカメラオフセット反映値
    return dcm

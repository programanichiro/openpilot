import os
import numpy as np
from openpilot.common.params import Params
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_MDL
# from openpilot.system.swaglog import cloudlog

#このファイルは廃止です。削除予定。-> chillモード時に復活してみる。昔の小細工は働かないようにしている。
params = Params()

STEER_SAME_DIRECTION_CT = 0
STEER_OLD_ANGLE = 0
STEERING_CENTER = -4.3
DCM_FRAME = 0
dcm_handle_ctrl = False

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera
PATH_OFFSET = 0.00
CAMERA_OFFSET = 0.04


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

    # self.l_lane_change_prob = 0.
    # self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    #self.camera_offset += 0.10 # 車体を10cm右に寄せる
    self.lane_collision = 0 #bit0:left , bit1:right
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

    self.frame_ct = 0
    self.lta_mode = False

  def parse_model(self, md, v_ego_car):
    #ここでlta_mode判定を行う。
    if self.frame_ct % 20 == 0:
      chill_enable = False #(sm['controlsState'].experimentalMode == False) #ここにsmはないので、experimentalMode判定を復活するなら一手間かかる。
      lta_enable_sw = False
      try:
        with open('/tmp/lta_enable_sw.txt','r') as fp:
          lta_enable_sw_str = fp.read()
          if lta_enable_sw_str:
            if int(lta_enable_sw_str) == 1: #LTA有効。
              lta_enable_sw = True
      except Exception as e:
        pass
      self.lta_mode = (v_ego_car > 16/3.6 or chill_enable) and lta_enable_sw

    self.frame_ct += 1
    if self.lta_mode == False:
      return

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

    # desire_state = md.meta.desireState
    # if len(desire_state):
    #   self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
    #   self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, st_angle, pred_angle , org_angle , v_ego, path_t, path_xyz):
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

    if False:
      # Reduce reliance on uncertain lanelines
      l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
      r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
      l_prob *= l_std_mod
      r_prob *= r_std_mod

      # Find current lanewidth
      self.lane_width_certainty.update(l_prob * r_prob)
      current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
      self.lane_width_estimate.update(current_lane_width)
      speed_lane_width = interp(v_ego, [0., 31.], [2.4, 3.1]) #, [2.8, 3.5]
      self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                        (1 - self.lane_width_certainty.x) * speed_lane_width

    # clipped_lane_width = min(4.0, self.lane_width)
    # path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    # path_from_right_lane = self.rll_y - clipped_lane_width / 2.0
    #速度によってマージンを増したほうがいい？
    lane_speed_margin = interp(v_ego*3.6 , [30,100] , [1,2]) #時速60キロで1.5倍弱になるよう調整。
    path_from_left_lane = self.lll_y + 1.8 / 2.0 + 0.3*lane_speed_margin #プリウスの車幅だけ補正して、左端〜右端の間はe2eの推論選択に任せる。
    path_from_right_lane = self.rll_y - 1.8 / 2.0 - 0.2*lane_speed_margin

    # self.d_prob = l_prob + r_prob - l_prob * r_prob # (*1)でここが0.25減で最大94%未満(0.75+0.75-0.75*0.75)になるよう調整される。
    safe_idxs = np.isfinite(self.ll_t)
    new_lane_collision = 0 #bit0:left , bit1:right
    if safe_idxs[0]:
      # lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
      # lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      # path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
      # # 以上従来処理
      lane_path_y_interp_left = np.interp(path_t, self.ll_t[safe_idxs], path_from_left_lane[safe_idxs])
      lane_path_y_interp_right = np.interp(path_t, self.ll_t[safe_idxs], path_from_right_lane[safe_idxs])
      # with open('/tmp/debug_out_o','w') as fp:
      #   fp.write('L:%.2f , e:%.2f ,w:%.1f , R:%.2f' % (path_from_left_lane[0] , path_xyz[:,1][0] , clipped_lane_width , path_from_right_lane[0]))
      #以下、各要素がレーンの左右をはみ出さないように。はみ出てなければe2eLatに従う。
      diff_mul = 1.1 #押し戻すための倍率
      diff_add = 0.05 * lane_speed_margin #さらに押し戻す距離[m]
      prob_max = 0.5 #レーン確率がこれ以上だと全て信用する。
      prob_min = 0.3 #レーン確率がこれ以上だと若干信用する。
      if pred_angle > 0:
        #左に曲がる時は右->左の順番で検査する。カーブの内側に切り込まないように。
        if r_prob > prob_min: #レーン右からはみ出さないように。
          # path_xyz[:,1] = [min(a, b) for a, b in zip(lane_path_y_interp_right, path_xyz[:,1])]
          diff_r = lane_path_y_interp_right[0] - path_xyz[:,1][0]
          if r_prob < prob_max:
            diff_r *= r_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_r < 0:
            path_xyz[:,1] += diff_r * diff_mul -diff_add #lane_path_y_interp_rightのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            new_lane_collision |= 2
        if l_prob > prob_min: #レーン左からはみ出さないように。
          # path_xyz[:,1] = [max(a, b) for a, b in zip(lane_path_y_interp_left, path_xyz[:,1])]
          diff_l = lane_path_y_interp_left[0] - path_xyz[:,1][0]
          if l_prob < prob_max:
            diff_l *= l_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_l > 0:
            path_xyz[:,1] += diff_l * diff_mul +diff_add #lane_path_y_interp_leftのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            new_lane_collision |= 1
      else:
        #右に曲がる時は左->右の順番で検査する。カーブの内側に切り込まないように。
        if l_prob > prob_min: #レーン左からはみ出さないように。
          # path_xyz[:,1] = [max(a, b) for a, b in zip(lane_path_y_interp_left, path_xyz[:,1])]
          diff_l = lane_path_y_interp_left[0] - path_xyz[:,1][0]
          if l_prob < prob_max:
            diff_l *= l_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_l > 0:
            path_xyz[:,1] += diff_l * diff_mul +diff_add #lane_path_y_interp_leftのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            new_lane_collision |= 1
        if r_prob > prob_min: #レーン右からはみ出さないように。
          # path_xyz[:,1] = [min(a, b) for a, b in zip(lane_path_y_interp_right, path_xyz[:,1])]
          diff_r = lane_path_y_interp_right[0] - path_xyz[:,1][0]
          if r_prob < prob_max:
            diff_r *= r_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_r < 0:
            path_xyz[:,1] += diff_r * diff_mul -diff_add #lane_path_y_interp_rightのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            new_lane_collision |= 2
    else:
      # cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
      pass
    if self.lane_collision != new_lane_collision:
      # if new_lane_collision == 1 or new_lane_collision == 2:
      #   with open('/tmp/signal_start_prompt_info.txt','w') as fp:
      #     fp.write('%d' % (1)) #prompt.wav音を鳴らしてみる。
      with open('/tmp/lane_collision.txt','w') as fp:
        fp.write('%d' % (new_lane_collision))
        self.lane_collision = new_lane_collision
    return path_xyz

#関数を最後に追加,dcm(ダイナミックカメラマージン？)名前がおかしいが、コーナーのイン側に寄せるオフセットである。早晩、こちらはlateral_planner.pyへ引っ越し予定。
  def calc_dcm(self, st_angle, pred_angle , org_angle , v_ego,clipped_lane_width,l_prob,r_prob):
    #数値を実際に取得して、調整してみる。UIスイッチで車体寄せをやめるなら、ここでゼロを返せばいい。
    return 0 #車体寄せを行わない。
    # handle_ctrl_sw.txt無効による実質的な廃止。
    global DCM_FRAME , dcm_handle_ctrl
    if DCM_FRAME % 30 == 1 and (st_angle != 0 or pred_angle != 0):
      try:
        with open('/tmp/handle_ctrl_sw.txt','r') as fp:
          dcm_handle_ctrl_sw_str = fp.read()
          if dcm_handle_ctrl_sw_str:
            dcm_handle_ctrl_sw = int(dcm_handle_ctrl_sw_str)
            if dcm_handle_ctrl_sw >= 2:
              dcm_handle_ctrl = True
            else:
              dcm_handle_ctrl = False
      except Exception as e:
        dcm_handle_ctrl = False
    DCM_FRAME += 1
    if dcm_handle_ctrl == False:
      return 0 #車体寄せを行わない

    handle_margin = 1 #1.5
    handle_over = 5
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
    if False: #0.8.14に合わせ、以下のブロックを無効に。v_ego > 60/3.6: # 60 or 70km/h over
      handle_margin = 1.5
      if STEER_SAME_DIRECTION_CT > 70 and clipped_lane_width - 2.5 >= 0:  #2.5 <- 1.9=prius width
        w_add = (clipped_lane_width - 2.5)  * 0.8 / 2.0
    if st_angle > handle_margin:
      #dcm = 0.01 - self.camera_offset + camera_margin
      dcm = 0.05 + camera_margin
      #dcm += w_add * 1.1 / 1.2
      dcm *= min((st_angle -(handle_margin)) / handle_over,1.0)
    if st_angle < -handle_margin:
      #dcm = -0.11 - self.camera_offset - camera_margin
      dcm = -0.05 - camera_margin
      #dcm -= w_add * 0.8 / 1.2 #減速と合わせると相当寄りすぎなので小さく
      dcm *= min(-(st_angle +(handle_margin)) / handle_over,1.0)
#🟥🟥🟥🟥🟥🟥🟥
    dcm_k = abs(pred_angle - org_angle) #前方推論角度と現角度の差がたくさんある時にオフセットする。
    if dcm_k > 10:
      dcm_k = 10
    dcm_k /= 10
    if False: #デバッグ表示なし。
      ms = "O:%+.2fx%.2f" % (dcm,dcm_k)
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
      with open('/tmp/debug_out_2','w') as fp:
        #fp.write('l:{0}\n'.format(['%0.2f' % i for i in path_from_left_lane]))
        #fp.write('r:{0}\n'.format(['%0.2f' % i for i in path_from_right_lane]))
        #fp.write('ofst:%0.2f[m] , lane_w:%0.2f[m], ct:%d' % (dcm , clipped_lane_width,STEER_SAME_DIRECTION_CT))
        #fp.write('OFS:%+.2f,w:%.2f[m],ct:%d' % (dcm , clipped_lane_width,min(STEER_SAME_DIRECTION_CT,99)))
        fp.write(ms)
    #if self.camera_offset * CAMERA_OFFSET < 0: #Consider wide_cameraこれ不要。ワイドカメラがメインカメラの反対についているだけで、方向が反対になるわけではない。
    #  dcm = -dcm
#    if r_prob == -1 and l_prob == -1: #ない方がいいかもしれん。取ると車体が右による？。想定と逆
#      dcm -= (-0.10) #この数字は結構いい感じ。
#      dcm -= self.camera_offset #レーンレスモデル用のカメラオフセット反映値
    return dcm * dcm_k #前方推論舵角だけ最大10度でdcmの大きさをリミットつける

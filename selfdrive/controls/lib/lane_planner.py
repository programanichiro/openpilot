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
    # self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    # self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    # self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    # self.d_prob = 0.

    # self.lll_std = 0.
    # self.rll_std = 0.

    # self.l_lane_change_prob = 0.
    # self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    #self.camera_offset += 0.10 # 車体を10cm右に寄せる
    self.lane_collision = 0 #bit0:left , bit1:right
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

    self.frame_ct = 0
    self.lta_mode = False

  def parse_model(self, md, v_ego_car, experimentalMode):
    #ここでlta_mode判定を行う。
    if self.frame_ct % 20 == 0:
      # chill_enable = experimentalMode #果たしてこうするべき？ , 速度条件は常に↓でいいのでは。
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

    self.camera_offset = -CAMERA_OFFSET if experimentalMode else CAMERA_OFFSET #experimentalMode == wide_camでいいの？

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
      # self.lll_std = md.laneLineStds[1]
      # self.rll_std = md.laneLineStds[2]

    # desire_state = md.meta.desireState
    # if len(desire_state):
    #   self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
    #   self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, pred_angle , v_ego, path_t, path_xyz):
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

    lane_speed_margin = interp(v_ego*3.6 , [30,100] , [1,0]) #時速60キロで1.5倍弱になるよう調整。走行モデル向上によってオーバーしにくくなったのか、効果を弱める。
    path_from_left_lane = self.lll_y + 1.8 / 2.0 + 0.2*lane_speed_margin #プリウスの車幅だけ補正して、左端〜右端の間はe2eの推論選択に任せる。
    path_from_right_lane = self.rll_y - 1.8 / 2.0 - 0.2*lane_speed_margin

    # self.d_prob = l_prob + r_prob - l_prob * r_prob # (*1)でここが0.25減で最大94%未満(0.75+0.75-0.75*0.75)になるよう調整される。
    safe_idxs = np.isfinite(self.ll_t)
    new_lane_collision = 0 #bit0:left , bit1:right
    lane_d = 0
    if safe_idxs[0]:
      lane_path_y_interp_left = np.interp(path_t, self.ll_t[safe_idxs], path_from_left_lane[safe_idxs])
      lane_path_y_interp_right = np.interp(path_t, self.ll_t[safe_idxs], path_from_right_lane[safe_idxs])
      # with open('/tmp/debug_out_o','w') as fp:
      #   fp.write('L:%.2f , e:%.2f , R:%.2f' % (path_from_left_lane[0] , path_xyz[:,1][0] , path_from_right_lane[0]))
      #以下、各要素がレーンの左右をはみ出さないように。はみ出てなければe2eLatに従う。
      diff_mul = 1.02 #押し戻すための倍率
      diff_add = 0.05 * lane_speed_margin #さらに押し戻す距離[m]
      prob_max = 0.5 #レーン確率がこれ以上だと全て信用する。
      prob_min = 0.3 #レーン確率がこれ以上だと若干信用する。
      org_path_y_0 = path_xyz[:,1][0]
      if pred_angle > 0:
        #左に曲がる時は右->左の順番で検査する。カーブの内側に切り込まないように。
        if r_prob > prob_min: #レーン右からはみ出さないように。
          # path_xyz[:,1] = [min(a, b) for a, b in zip(lane_path_y_interp_right, path_xyz[:,1])]
          diff_r = lane_path_y_interp_right[0] - org_path_y_0
          if r_prob < prob_max:
            diff_r *= r_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_r < 0:
            #不要path_xyz[:,1] += diff_r * diff_mul -diff_add #lane_path_y_interp_rightのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            lane_d = diff_r * diff_mul -diff_add
            new_lane_collision |= 2
        if l_prob > prob_min: #レーン左からはみ出さないように。
          # path_xyz[:,1] = [max(a, b) for a, b in zip(lane_path_y_interp_left, path_xyz[:,1])]
          diff_l = lane_path_y_interp_left[0] - org_path_y_0
          if l_prob < prob_max:
            diff_l *= l_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_l > 0:
            #不要path_xyz[:,1] += diff_l * diff_mul +diff_add #lane_path_y_interp_leftのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            lane_d = diff_l * diff_mul +diff_add
            new_lane_collision |= 1
      else:
        #右に曲がる時は左->右の順番で検査する。カーブの内側に切り込まないように。
        if l_prob > prob_min: #レーン左からはみ出さないように。
          # path_xyz[:,1] = [max(a, b) for a, b in zip(lane_path_y_interp_left, path_xyz[:,1])]
          diff_l = lane_path_y_interp_left[0] - org_path_y_0
          if l_prob < prob_max:
            diff_l *= l_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_l > 0:
            #不要path_xyz[:,1] += diff_l * diff_mul +diff_add #lane_path_y_interp_leftのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            lane_d = diff_l * diff_mul +diff_add
            new_lane_collision |= 1
        if r_prob > prob_min: #レーン右からはみ出さないように。
          # path_xyz[:,1] = [min(a, b) for a, b in zip(lane_path_y_interp_right, path_xyz[:,1])]
          diff_r = lane_path_y_interp_right[0] - org_path_y_0
          if r_prob < prob_max:
            diff_r *= r_prob/prob_max #prob_max以下の場合は押し戻す距離を減らす
          if diff_r < 0:
            #不要path_xyz[:,1] += diff_r * diff_mul -diff_add #lane_path_y_interp_rightのカーブ形状が使えないとなると、path_xyzを活かさなければならない。
            lane_d = diff_r * diff_mul -diff_add
            new_lane_collision |= 2

      if new_lane_collision == 3: #両脇に接触した例外処理
        #中央値を取る
        center_y = (lane_path_y_interp_right[0] + lane_path_y_interp_left[0]) * 0.5
        # center_y = (r_prob * lane_path_y_interp_right[0] + l_prob * lane_path_y_interp_left[0]) / (l_prob + r_prob + 0.0001) #probを考慮
        # lane_d = center_y - org_path_y_0
        lane_d = 0 #もしくはlane_d=0にするのも手か。
        new_lane_collision |= 4 #無視状態をUIに表示

      lane_w = -99 #右がプラスの数字
      if r_prob > prob_min and l_prob > prob_min:
        lane_l = self.lll_y[0]
        if l_prob < prob_max:
          lane_l *= l_prob/prob_max
        lane_r = self.rll_y[0]
        if r_prob < prob_max:
          lane_r *= r_prob/prob_max
        lane_w = lane_r - lane_l #これでメートル的なイメージになる？
        if lane_w <= 1.9:
          lane_d = 0 #操舵しない
          new_lane_collision |= 4 #無視状態をUIに表示
      # with open('/tmp/debug_out_o','w') as fp:
      #   fp.write('%.1fm' % (lane_w))

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
    #return path_xyz , lane_d #パスは戻り値に要らない。
    return lane_d


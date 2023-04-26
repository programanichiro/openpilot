#!/usr/bin/env python3
import os
import random
import math
import numpy as np
from common.numpy_fast import clip, interp

import cereal.messaging as messaging
from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, MIN_ACCEL, MAX_ACCEL
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from system.swaglog import cloudlog

from selfdrive.car.toyota.values import TSS2_CAR
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from common.params import Params
params = Params()
CVS_FRAME = 0
handle_center = 0 #STEERING_CENTER
accel_lead_ctrl = True
decel_lead_ctrl = True
v_cruise = 0
v_cruise_old = 0
signal_scan_ct = 0
red_signal_scan_ct = 0
red_signal_scan_ct_2 = 0 #red_signal_scan_flagが2になった瞬間から加算し始める。
red_signal_scan_span = 0 #red_signal_scan_flagが3になった瞬間のred_signal_scan_ct_2を保持する。
red_signal_speed_down_before = 0
red_signal_scan_flag = 0 #0:何もしない, 1:赤信号センシング, 2:赤信号検出, 3:赤信号停止動作中
with open('/tmp/red_signal_scan_flag.txt','w') as fp:
  fp.write('%d' % (0))
path_x_old_signal = 0
path_x_old_signal_check = 0
desired_path_x_speeds    = [0,5 ,10  ,15  ,20  ,30  ,40   ,50   ,60   ,70   ,80   ,90   ,100  ]
#desired_path_x_by_speeds = [0,15,60-5,65-5,75-5,90-5,125-5,158-5,178-5,190-5,220-5,240-5,255-5] #toro_555
desired_path_x_by_speeds = [0,15,55  ,60  ,73-5,88-5,125-5,160-5,178-5,193-5,220-5,240-5,255-5] #toro_555
#desired_path_x_by_speeds = [0,15,60-5,65-5,75-5,95-5,125-5,150-5,170-5,190-5,220-5,240-5,255-5]
long_speeddown_flag = False
before_v_cruise_kph_max_1 = 0

def calc_limit_vc(X1,X2,X3 , Y1,Y2,Y3):
  Z1 = (X2-X1)/(Y1-Y2) - (X3-X2)/(Y2-Y3)
  Z2 = (X3-X2)/(Y2-Y3) - (X1-X3)/(Y3-Y1)
  A = (X2-X1)*(X1*X2 - X2*X3) - (X1-X3)*(X2*X3 - X3*X1)
  A /= Z1*(X2-X1) - Z2*(X1-X3)
  B = ((X1*X2 - X2*X3) - A*Z1) / (X1-X3)
  C = Y1 - A / (X1 - B)
  return (A,B,C)

#LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,11.6,27.0 , 86-4      ,60-4      ,47-4      )
#LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,11.6,27.0 , 91-4      ,65-4      ,49-4      )
LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,13.6,29.0 , 92-4      ,65.5-4      ,45.5-4      )
#LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH = calc_limit_vc(8.7,13.0,25.0 , 112,93,81)
LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH = calc_limit_vc(9.5,13.0,25.0 , 120,108,99)

OP_ENABLE_ACCEL_RELEASE = False
OP_ENABLE_PREV = False
OP_ENABLE_v_cruise_kph = 0
OP_ENABLE_gas_speed = 0
OP_ACCEL_PUSH = False
on_onepedal_ct = -1
cruise_info_power_up = False
one_pedal_chenge_restrict_time = 0

START_DASH_CUT    = [0, 17/3.6, 26/3.6, 36/3.6, 45/3.6, 55/3.6, 64/3.6, 74/3.6, 83/3.6,  93/3.6]
START_DASH_SPEEDS = [0, 31/3.6, 41/3.6, 51/3.6, 61/3.6, 70/3.6, 80/3.6, 90/3.6, 100/3.6, 110/3.6]

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.red_signals = np.zeros(10)
    self.red_signal_path_xs = np.zeros(5)
    self.old_red_signal_path_xs = 0
    self.night_time = 100 #変数名がわかりにくいが環境光の強さが0〜100で取得できる。
    self.night_time_refresh_ct = 0
    self.desired_path_x_rates = np.zeros(5)
    self.ac_vc_time = 0.0
    self.limitspeed_point = 0.0

    try:
      with open('/tmp/lane_sw_mode.txt','r') as fp:
        lane_sw_mode_str = fp.read()
        if lane_sw_mode_str:
          lane_sw_mode = int(lane_sw_mode_str)
          if lane_sw_mode >= 1: #dynamic experimental mode
            with open('/tmp/long_speeddown_disable.txt','w') as fp:
              fp.write('%d' % (1)) #初期はイチロウロング無効
    except Exception as e:
      pass

    if os.environ['DONGLE_ID'] in ('cdcb457f7528673b'):
      self.dexp_mode_min = 40/3.6
      self.dexp_mode_max = 41/3.6
    else:
      self.dexp_mode_min = 20/3.6
      self.dexp_mode_max = 23/3.6

    if self.CP.carFingerprint not in TSS2_CAR:
      LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,13.6,57.0 , 92-4      ,65.5-4      ,31.0      ) #ハンドル60度で時速30km/h程度まで下げる設定。

  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm):
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    dexp_mode = False
    try:
      with open('/tmp/lane_sw_mode.txt','r') as fp:
        lane_sw_mode_str = fp.read()
        if lane_sw_mode_str:
          lane_sw_mode = int(lane_sw_mode_str)
          if lane_sw_mode >= 1: #dynamic experimental mode
            dexp_mode = True
    except Exception as e:
      pass
    if dexp_mode:
      if self.mpc.mode == 'acc':
        if v_ego <= self.dexp_mode_min and sm['carState'].gasPressed == False:
          params.put_bool("ExperimentalMode", True) # blended
          with open('/tmp/long_speeddown_disable.txt','w') as fp:
            fp.write('%d' % (1)) #イチロウロング無効
      else:
        if v_ego > self.dexp_mode_max or sm['carState'].gasPressed == True:
          params.put_bool("ExperimentalMode", False) # acc
          with open('/tmp/long_speeddown_disable.txt','w') as fp:
            fp.write('%d' % (0)) #イチロウロング有効
    global CVS_FRAME , handle_center , OP_ENABLE_PREV , OP_ENABLE_v_cruise_kph , OP_ENABLE_gas_speed , OP_ENABLE_ACCEL_RELEASE , OP_ACCEL_PUSH , on_onepedal_ct , cruise_info_power_up , one_pedal_chenge_restrict_time
    #with open('/tmp/debug_out_v','w') as fp:
    #  fp.write("%d push:%d , gas:%.2f" % (CVS_FRAME,sm['carState'].gasPressed,sm['carState'].gas))
    min_acc_speed = 31
    v_cruise_kph = sm['controlsState'].vCruise
    if self.CP.carFingerprint not in TSS2_CAR:
      v_cruise_kph = (55 - (55 - (v_cruise_kph+4)) * 2 - 4) if v_cruise_kph < (55 - 4) else v_cruise_kph
      v_cruise_kph = (110 + ((v_cruise_kph+6) - 110) * 3 - 6) if v_cruise_kph > (110 - 6) else v_cruise_kph
      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('../../../tss_type_info.txt','w') as fp:
          fp.write('%d' % (1))
    else:
      min_acc_speed = 30
      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('../../../tss_type_info.txt','w') as fp:
          fp.write('%d' % (2))
    if v_cruise_kph < min_acc_speed:
      v_cruise_kph = min_acc_speed #念のため

    one_pedal = False
    on_accel0 = False #押した瞬間
    if v_ego <= 3/3.6 or (OP_ACCEL_PUSH == False and sm['carState'].gasPressed):
      try:
        with open('/tmp/accel_engaged.txt','r') as fp:
          accel_engaged_str = fp.read()
          if accel_engaged_str:
            if int(accel_engaged_str) == 3: #ワンペダルモード
              one_pedal = True
              if OP_ACCEL_PUSH == False and sm['carState'].gasPressed:
                if on_onepedal_ct < 0:
                  on_onepedal_ct = 0 #ワンペダルかアクセル判定開始
      except Exception as e:
        pass
    if on_onepedal_ct >= 0:
      on_onepedal_ct += 1
      if on_onepedal_ct > 5:# 1秒後に。フレームレートを実測すると、30カウントくらいで1秒？
        if sm['carState'].gas < 0.32: #アクセルが弱いかチョン押しなら
          on_accel0 = True #ワンペダルに変更
        on_onepedal_ct = -1 #アクセル判定消去
    if on_accel0 and v_ego > 1/3.6 : #オートパイロット中にアクセルを弱めに操作したらワンペダルモード有効。ただし先頭スタートは除く。
      if sm['controlsState'].enabled and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > 1.0 / 3.6):
        with open('/tmp/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (1)) #prompt.wav音を鳴らしてみる。
          #しばらくやってもなかなか出ない？fp.write('%d' % (3)) #デバッグでpo.wav音を鳴らす。
      OP_ENABLE_v_cruise_kph = v_cruise_kph
      OP_ENABLE_gas_speed = 1.0 / 3.6
      one_pedal_chenge_restrict_time = 20
    if one_pedal_chenge_restrict_time > 0:
      one_pedal_chenge_restrict_time -= 1
    if one_pedal == True and v_ego < 0.1/3.6 and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > 1.0 / 3.6) and sm['controlsState'].enabled and sm['carState'].gasPressed == False:
      force_one_pedal_set = False
      try:
        with open('/tmp/force_one_pedal.txt','r') as fp:
          force_one_pedal_str = fp.read()
          if force_one_pedal_str:
            if int(force_one_pedal_str) == 1:
              force_one_pedal_set = True
              OP_ENABLE_v_cruise_kph = v_cruise_kph
              OP_ENABLE_gas_speed = 1.0 / 3.6
      except Exception as e:
        pass
      if force_one_pedal_set == True:
        with open('/tmp/force_one_pedal.txt','w') as fp:
          fp.write('%d' % (0))
        with open('/tmp/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (1)) #MAXを1に戻すのでprompt.wavを鳴らす。

    sm_longControlState = sm['controlsState'].longControlState
    if sm_longControlState == LongCtrlState.off:
      # if sm['carState'].gasPressed and sm['controlsState'].enabled: #アクセル踏んでエンゲージ中なら
      if sm['controlsState'].enabled: #アクセル踏む条件を無視してみる。
        sm_longControlState = LongCtrlState.pid #0.8.14からアクセルONでLongCtrlState.offとなるため、従来動作をシミュレート
    if OP_ENABLE_PREV == False and sm_longControlState != LongCtrlState.off and (((one_pedal or v_ego > 3/3.6) and v_ego < min_acc_speed/3.6 and int(v_cruise_kph) == min_acc_speed) or sm['carState'].gasPressed):
       #速度が時速３km以上かつ31km未満かつsm['controlsState'].vCruiseが最低速度なら、アクセル踏んでなくても無条件にエクストラエンゲージする
    #if tss2_flag == False and OP_ENABLE_PREV == False and sm['controlsState'].longControlState != LongCtrlState.off and sm['carState'].gasPressed:
      #アクセル踏みながらのOP有効化の瞬間
      OP_ENABLE_v_cruise_kph = v_cruise_kph
      if one_pedal_chenge_restrict_time == 0:
        OP_ENABLE_gas_speed = v_ego
      try:
        with open('/tmp/accel_engaged.txt','r') as fp:
          accel_engaged_str = fp.read()
          if accel_engaged_str:
            if int(accel_engaged_str) == 3 and sm['carState'].gasPressed == False: #ワンペダルモード(開始時にアクセル操作していたら低速エンゲージとする)
              OP_ENABLE_gas_speed = 1.0 / 3.6
      except Exception as e:
        pass
      OP_ENABLE_ACCEL_RELEASE = False
    if sm_longControlState != LongCtrlState.off:
      OP_ENABLE_PREV = True
      if sm['carState'].gasPressed and OP_ENABLE_ACCEL_RELEASE == False:
        if one_pedal_chenge_restrict_time == 0:
          OP_ENABLE_gas_speed = v_ego
        # try:
        #   with open('/tmp/accel_engaged.txt','r') as fp:
        #     accel_engaged_str = fp.read()
        #     if accel_engaged_str:
        #       if int(accel_engaged_str) == 3: #ワンペダルモード
        #         OP_ENABLE_gas_speed = 1.0 / 3.6
        # except Exception as e:
        #   pass
    else:
      OP_ENABLE_PREV = False
      OP_ENABLE_v_cruise_kph = 0
    if sm['carState'].gasPressed == False: #一旦アクセルを離したら、クルーズ速度は変更しない。変更を許すと、ACC速度とMAX速度の乖離が大きくなり過ぎる可能性があるから。
      OP_ENABLE_ACCEL_RELEASE = True
      OP_ACCEL_PUSH = False #アクセル離した
    else:
      OP_ACCEL_PUSH = True #アクセル押した

    md = sm['modelV2']
    hasLead = sm['radarState'].leadOne.status
    distLead_near = sm['radarState'].leadOne.dRel < interp(v_ego*3.6 , [30,80] , [50,120]) #前走車が近ければTrue
    global signal_scan_ct,path_x_old_signal,path_x_old_signal_check , red_signal_scan_flag
    if v_ego <= 0.01/3.6 and (OP_ENABLE_v_cruise_kph > 0 or one_pedal == False or (OP_ENABLE_v_cruise_kph == 0 and (hasLead == False or distLead_near == False))) and sm['controlsState'].enabled and sm['carState'].gasPressed == False: #and (hasLead == False or (sm['radarState'].leadOne.dRel > 40 and sm['radarState'].leadOne.modelProb > 0.5)):
      #速度ゼロでエンゲージ中、前走車なしでアクセル踏んでない。
      steer_ang = sm['carState'].steeringAngleDeg - handle_center
      # 停止時の青信号発進抑制、一時的に緩和、15->50度
      if (abs(steer_ang) < 50 or one_pedal == False) and len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE: #ワンペダルならある程度ハンドルが正面を向いていること。
        #path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        path_x = md.position.x #path_xyz[:,0]
        # with open('/tmp/debug_out_k','w') as fp: #path_xの中を解析して、ビュンと伸びる瞬間を判断したい。
        #   #fp.write('{0}\n'.format(['%0.2f' % i for i in path_x]))
        #   fp.write('l:%d(%.2f),%.2f[m],x:%.2f' % (hasLead ,sm['radarState'].leadOne.modelProb , sm['radarState'].leadOne.dRel , path_x[TRAJECTORY_SIZE -1]))
        if (path_x_old_signal < 2) and path_x[TRAJECTORY_SIZE -1] > 40:
          path_x_old_signal_check = path_x[TRAJECTORY_SIZE -1] #ゆっくり立ち上がったらこれはTrueにならない。
        path_x_base_limit = 64.0 #70.0 , この座標値超で青信号スタート発火。
        if path_x[TRAJECTORY_SIZE -1] > path_x_base_limit or path_x_old_signal_check > 40: #青信号判定の瞬間
          path_x_old_signal_check += path_x[TRAJECTORY_SIZE -1] #最初の立ち上がりは2倍される
          signal_scan_ct += 1 #横道からの進入車でパスが伸びたのを勘違いするので、バッファを設ける。
          limit_8 = 8 if path_x[TRAJECTORY_SIZE -1] > path_x_base_limit else 16
          if signal_scan_ct > limit_8 and signal_scan_ct < 100 and (path_x[TRAJECTORY_SIZE -1] > path_x_base_limit or (path_x_old_signal_check-40) > 50*limit_8):
            with open('/tmp/signal_start_prompt_info.txt','w') as fp:
              if sm['driverMonitoringState'].isActiveMode == True: #よそ見をしていたら発進しない。
                if OP_ENABLE_v_cruise_kph > 0: #MAX==1の状態。
                  fp.write('%d' % (2)) #engage.wavを鳴らす。
                else:
                  fp.write('%d' % (1)) #prompt.wavを鳴らす。
                OP_ENABLE_v_cruise_kph = 0 #エクストラエンゲージ解除
                # if red_signal_scan_flag == 3:
                #   red_signal_scan_flag = 0 #赤ブレーキキャンセルならゼロに戻す。
              else:
                fp.write('%d' % (1)) #よそ見してたらprompt.wavを鳴らす。
            signal_scan_ct = 200 #2回鳴るのを防止
            path_x_old_signal_check = 0
        else:
          signal_scan_ct = 0 if signal_scan_ct < 4 else signal_scan_ct - 4
          path_x_old_signal_check = 0
        path_x_old_signal = path_x[TRAJECTORY_SIZE -1]
    else:
      signal_scan_ct = 0 if signal_scan_ct < 4 else signal_scan_ct - 4
    if path_x_old_signal < 30:
      path_x_old_signal_check = 0

    if a_ego > 0 and v_ego >= min_acc_speed/3.6 and OP_ENABLE_v_cruise_kph > 0 and sm['controlsState'].enabled and sm['carState'].gas > 0.32: #アクセル強押しでワンペダルからオートパイロットへ
      OP_ENABLE_v_cruise_kph = 0 #エクストラエンゲージ解除
      signal_scan_ct = 200 #このあと信号スタート判定されてprompt.wavが鳴るのを防止する。
      with open('/tmp/signal_start_prompt_info.txt','w') as fp:
        fp.write('%d' % (2)) #engage.wavを鳴らす。

    global red_signal_scan_ct , red_signal_scan_ct_2 , red_signal_speed_down_before , red_signal_scan_span , long_speeddown_flag , before_v_cruise_kph_max_1
    red_signal_scan_flag_1 = red_signal_scan_flag
    red_signal_speed_down = 1.0
    desired_path_x_rate = 1.0 #一般的な減速制御
    set_red_signal_scan_flag_3 = False
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE: #これがFalseのケースは想定外
      path_x = md.position.x #path_xyz[:,0]
      red_signal_v_ego = 4/3.6 #この速度超で赤信号認識。
      if (hasLead == False or distLead_near == False) and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > red_signal_v_ego):
        if red_signal_scan_flag_1 != 3 and v_ego > red_signal_v_ego:
          red_signal_scan_flag_1 = 1 #赤信号センシング

      # path_x[TRAJECTORY_SIZE -1]が増加方向の時は弾きたい。
      self.red_signal_path_xs = np.append(self.red_signal_path_xs,path_x[TRAJECTORY_SIZE -1])
      self.red_signal_path_xs = np.delete(self.red_signal_path_xs , [0])
      sum_red_signal_path_xs = np.sum(self.red_signal_path_xs)

      if (hasLead == False or distLead_near == False) and path_x[TRAJECTORY_SIZE -1] < interp(v_ego*3.6 , [0,10,20,30,40,50,55,60] , [20,30,50,70,80,90,105,120]): #60
        red_signal = "●"
        self.red_signals = np.append(self.red_signals,1)
      else:
        red_signal = "◯"
        self.red_signals = np.append(self.red_signals,0)
      self.red_signals = np.delete(self.red_signals , [0])
      red_signals_sum = np.sum(self.red_signals)
      if red_signals_sum > self.red_signals.size * 0.7:
        red_signals_mark = "■"
        if red_signal_scan_flag_1 != 3 and v_ego > red_signal_v_ego:
          if red_signal_scan_flag < 2:
            red_signal_scan_ct_2 = 0
          red_signal_scan_ct_2 += 1 #red_signal_scan_flagが2になった瞬間から加算し始める。
          red_signal_scan_flag_1 = 2 #赤信号検出
          #この信号認識状態 and sum_red_signal_path_xs < self.old_red_signal_path_xsなら速度を落とし始めてもいい？ 473行のv_cruiseを1割落とすとか
          if v_ego > 20/3.6 and sum_red_signal_path_xs < self.old_red_signal_path_xs:
            red_signal_speed_down = interp(v_ego*3.6 , [10,20,30,40,50,55,60] , [0.97,0.96,0.95,0.94,0.93,0.92,0.91])
            red_signal_scan_ct_2_rate = 200 if red_signal_scan_ct_2 > 200 else red_signal_scan_ct_2 #最大200
            red_signal_speed_down -= red_signal_scan_ct_2_rate * 0.3 / 200 #徐々にブレーキが強くなる
            red_signal_speed_down_before = red_signal_speed_down
          elif red_signal_speed_down_before > 0:
            red_signal_speed_down = red_signal_speed_down_before #条件に外れたら、一度だけ過去を参照する。
            red_signal_speed_down_before = 0
      else:
        red_signals_mark = "□"

      desired_path_x_by_speed = interp(v_ego*3.6,desired_path_x_speeds,desired_path_x_by_speeds)
      desired_path_x_rate = 0 if desired_path_x_by_speed <= 0.01 else path_x[TRAJECTORY_SIZE -1]/desired_path_x_by_speed
      self.desired_path_x_rates = np.append(self.desired_path_x_rates,desired_path_x_rate)
      self.desired_path_x_rates = np.delete(self.desired_path_x_rates , [0])
      desired_path_x_rate = np.sum(self.desired_path_x_rates) / self.desired_path_x_rates.size
      if True: #CVS_FRAME % 2 == 0:
        with open('/tmp/desired_path_x_rate.txt','w') as fp:
          fp.write('%0.2f' % (desired_path_x_rate))
      # with open('/tmp/debug_out_k','w') as fp:
      # #   #fp.write('{0}\n'.format(['%0.2f' % i for i in path_x]))
      #   lead_mark = "▲"
      #   if hasLead == False or distLead_near == False:
      #     lead_mark = "△"
      # #   #fp.write('{0}\n'.format(['%0.2f' % i for i in self.desired_path_x_rates]))
      # #   #fp.write('@@@%f,%f,%f' % (v_ego,desired_path_x_by_speed,path_x[TRAJECTORY_SIZE -1]))
      # #   #fp.write('***%.2f,[%.2f],%d' % (np.sum(self.desired_path_x_rates),desired_path_x_rate,self.desired_path_x_rates.size))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s%dm,[%d%%]%.2f' % (v_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[TRAJECTORY_SIZE -1] ,lead_mark , sm['radarState'].leadOne.dRel,desired_path_x_rate*100,a_ego))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s%dm,↓%.2f,%d' % (v_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[TRAJECTORY_SIZE -1] ,lead_mark , sm['radarState'].leadOne.dRel,red_signal_speed_down,red_signal_scan_span))
      #   fp.write('%02dk<%d>%s%s(%.1f)%s%dm,%d' % (v_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[TRAJECTORY_SIZE -1] ,lead_mark , sm['radarState'].leadOne.dRel,self.night_time))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s(%.2f,%.2f)' % (v_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[TRAJECTORY_SIZE -1] ,lead_mark ,sm['radarState'].leadOne.modelProb,sm['radarState'].leadTwo.modelProb))
      red_signal_scan_ct += 1 #音を鳴らした後の緩衝処理になっているだけで、信号検出のあと徐々に加算されるロジックではないようだ。

      self.night_time_refresh_ct += 1
      if (self.night_time_refresh_ct % 11 == 6 and red_signal == "●") or self.night_time_refresh_ct % 200 == 100:
        try:
          with open('/tmp/night_time_info.txt','r') as fp:
            night_time_info_str = fp.read()
            if night_time_info_str:
              self.night_time = int(night_time_info_str)
        except Exception as e:
          pass
      red_stop_immediately = False
      if long_speeddown_flag == False and self.mpc.mode == 'acc': #公式ロングではelseへ強制遷移する追加条件
        if self.night_time >= 90: #昼,90以下だと夕方で信号がかなり見やすくなる。
          stop_threshold = interp(v_ego*3.6 , [0,10,20,30,40,50,55,60] , [15,25,35,43,59,77,92,103]) #昼の方が認識があまくなるようだ。
        else: #夜
          stop_threshold = interp(v_ego*3.6 , [0,10,20,30,40,50,55,60] , [10,19,28,39,53,75,85,99]) #まあまあ,60km/hでも止まれる！？
        if path_x[TRAJECTORY_SIZE -1] < stop_threshold:
          red_stop_immediately = True #停止せよ。
      else:
        if True: #self.night_time >= 90: #昼,90以下だと夕方で信号がかなり見やすくなる。
          stop_threshold = interp(v_ego*3.6 , [0,10,20,30,40,50,60] , [15,20,23,28,43,57,66]) #事前減速で40km/h以下になることを期待している。昼
        # else: #夜
        #   stop_threshold = interp(v_ego*3.6 , [0,10,20,30,40,50] , [15,20,23,27,38,52]) #事前減速で40km/h以下になることを期待している。夜
        if path_x[TRAJECTORY_SIZE -1] < stop_threshold or desired_path_x_rate < 0.11:
          red_stop_immediately = True #停止せよ。
        # stop_threshold_r = interp(v_ego*3.6 , [0   ,10  ,20  ,25  ,30  ,40  ,50  ]
        #                                     , [0.25,0.30,0.33,0.35,0.38,0.41,0.43]) #さらに減速度の強さa_egoを加味。弱ければより小さくできる？
        # if desired_path_x_rate < stop_threshold_r: #0.4:
        #   red_stop_immediately = True #停止せよ。
      if sum_red_signal_path_xs < self.old_red_signal_path_xs and v_ego > red_signal_v_ego and red_signals_mark == "■" and sm['controlsState'].enabled and sm['carState'].gasPressed == False and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > red_signal_v_ego) and red_stop_immediately == True:
        #赤信号検出でワンペダル発動
        if red_signal_scan_ct < 10000:
          red_signal_scan_ct = 10000
          #まずは音を鳴らす。
          try:
            with open('/tmp/accel_engaged.txt','r') as fp:
              accel_engaged_str = fp.read()
              if accel_engaged_str:
                if int(accel_engaged_str) == 3: #ワンペダルモード
                    # fp.write('%d' % (3)) #デバッグ用にpo.wavを鳴らしてみる。
                  lock_off = False
                  if os.path.isfile('/tmp/lockon_disp_disable.txt'):
                    with open('/tmp/lockon_disp_disable.txt','r') as fp: #臨時でロックオンボタンに連動
                      lockon_disp_disable_str = fp.read()
                      if lockon_disp_disable_str:
                        lockon_disp_disable = int(lockon_disp_disable_str)
                        if lockon_disp_disable != 0:
                          lock_off = True #ロックオンOFFで停車コードOFF
                  if lock_off == False:
                    with open('/tmp/signal_start_prompt_info.txt','w') as fp:
                      fp.write('%d' % (1)) #prompt.wav音を鳴らしてみる。
                    OP_ENABLE_v_cruise_kph = v_cruise_kph
                    OP_ENABLE_gas_speed = 1.0 / 3.6
                    #one_pedal_chenge_restrict_time = 10 , ここは意味的に要らないか。
                    red_signal_scan_flag_1 = 3 #赤信号停止状態
                    set_red_signal_scan_flag_3 = True #セットした瞬間
                    red_signal_scan_span = red_signal_scan_ct_2 #2〜3までのフレーム数
          except Exception as e:
            pass
      else:
        red_signal_scan_ct = 0 if red_signal_scan_ct < 1000 else red_signal_scan_ct - 1000
      self.old_red_signal_path_xs = sum_red_signal_path_xs

    if (hasLead == True and distLead_near == True) or sm['controlsState'].enabled == False or sm['carState'].gasPressed == True or v_ego < 0.1/3.6:
      if set_red_signal_scan_flag_3 == False:
        red_signal_scan_flag_1 = 0

    if red_signal_scan_flag_1 != red_signal_scan_flag:
      red_signal_scan_flag = red_signal_scan_flag_1
      rssf = red_signal_scan_flag
      if red_signal_scan_flag <= 1:
        red_signal_scan_span = 0
      try:
        with open('/tmp/accel_engaged.txt','r') as fp:
          accel_engaged_str = fp.read()
          if accel_engaged_str:
            if int(accel_engaged_str) != 3: #ワンペダルモード以外
              rssf = 0
      except Exception as e:
        pass
      with open('/tmp/red_signal_scan_flag.txt','w') as fp:
        fp.write('%d' % (rssf))

    if hasLead == False and one_pedal == True and v_ego < 0.1/3.6: #速度ゼロでIPモード時にレバー下に入れたら
      if v_cruise_kph < before_v_cruise_kph_max_1 and before_v_cruise_kph_max_1 < 200: #200km/h以下の場合のみ。初回の誤設定を弾く。
        if OP_ENABLE_v_cruise_kph == 0:
          with open('/tmp/signal_start_prompt_info.txt','w') as fp:
            fp.write('%d' % (1)) #MAXを1に戻すのでprompt.wavを鳴らす。
        OP_ENABLE_v_cruise_kph = v_cruise_kph
        OP_ENABLE_gas_speed = 1.0 / 3.6
    if v_ego > 3/3.6 and v_ego <= 30/3.6:
      force_low_engage_set = False #MAX!=1でタッチすると低速(スピードが3〜30km/h)でエンゲージ。
      try:
        with open('/tmp/force_low_engage.txt','r') as fp:
          force_low_engage_str = fp.read()
          if force_low_engage_str:
            if int(force_low_engage_str) == 1:
              OP_ENABLE_v_cruise_kph = v_cruise_kph
              OP_ENABLE_gas_speed = v_ego
              force_low_engage_set = True
              if sm['carState'].gasPressed:
                OP_ENABLE_ACCEL_RELEASE = False #このあとのアクセルコントロールを許可する
      except Exception as e:
        pass
      if force_low_engage_set:
        with open('/tmp/force_low_engage.txt','w') as fp:
          fp.write('%d' % (0))
        with open('/tmp/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (2)) #engage.wavを鳴らす。
    if v_ego > 3/3.6 and v_ego <= 30/3.6 and sm['carState'].gasPressed and sm['controlsState'].enabled: #oneペダル操作中にアクセル踏みながら30km/h以下の走行時にレバーを上に入れたら、一旦車体速度にエクストラエンゲージし直す。
      if before_v_cruise_kph_max_1 <= 37 and OP_ENABLE_gas_speed == 1.0 / 3.6 and v_cruise_kph > before_v_cruise_kph_max_1: # これを繰り返すとACC設定速度がどんどん上がっていく。ACC最低速度近辺(37程度)に限定
        OP_ENABLE_v_cruise_kph = v_cruise_kph
        OP_ENABLE_gas_speed = v_ego
        OP_ENABLE_ACCEL_RELEASE = False #このあとのアクセルコントロールを許可する
        with open('/tmp/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (2)) #engage.wavを鳴らす。
    before_v_cruise_kph_max_1 = v_cruise_kph

    if OP_ENABLE_v_cruise_kph != v_cruise_kph: #レバー操作したらエンゲージ初期クルーズ速度解除
      OP_ENABLE_v_cruise_kph = 0
      if red_signal_scan_flag == 3:
        red_signal_scan_flag = 2
        with open('/tmp/red_signal_scan_flag.txt','w') as fp:
          fp.write('%d' % (red_signal_scan_flag))
    if OP_ENABLE_v_cruise_kph != 0:
      v_cruise_kph = OP_ENABLE_gas_speed*3.6 #エンゲージ初期クルーズ速度を優先して使う
    if CVS_FRAME % 5 == 4:
      try:
        with open('/tmp/handle_center_info.txt','r') as fp:
          handle_center_info_str = fp.read()
          if handle_center_info_str:
            handle_center = float(handle_center_info_str)
      except Exception as e:
        pass

    limitspeed_set = False
    try:
      with open('/tmp/limitspeed_sw.txt','r') as fp:
        limitspeed_sw_str = fp.read()
        if limitspeed_sw_str:
          if int(limitspeed_sw_str) == 1 and OP_ENABLE_v_cruise_kph == 0 and sm['controlsState'].enabled: #自動設定モード
            with open('/tmp/limitspeed_data.txt','r') as fp2:
              limitspeed_data_str = fp2.read()
              # fp.write('%d,%.2f,999,%d,%.1fm,+%d' % (int(get_limitspeed/10) * 10 , get_limitspeed , self.velo_ave_ct_old , (self.min_distance_old**0.5) * 100 / 0.0009 , self.db_add))
              limitspeed_data = limitspeed_data_str.split(",")
              limitspeed_flag = int(limitspeed_data[2])
              if limitspeed_flag == 999:
                target = float(limitspeed_data[1]) #実際にセットするのは平均速度の方
                if target > self.limitspeed_point+10:
                  self.limitspeed_point = target -10
                elif target < self.limitspeed_point-10:
                  self.limitspeed_point = target +10
                elif target > self.limitspeed_point+5:
                  self.limitspeed_point += 1
                elif target < self.limitspeed_point-5:
                  self.limitspeed_point -= 1
                elif target > self.limitspeed_point:
                  self.limitspeed_point += 0.1
                elif target < self.limitspeed_point:
                  self.limitspeed_point -= 0.1
                v_cruise_kph = self.limitspeed_point
                limitspeed_set = True
    except Exception as e:
      pass

#  struct LeadData {
#    dRel @0 :Float32;
#    yRel @1 :Float32;
#    vRel @2 :Float32;
#    aRel @3 :Float32;
#    vLead @4 :Float32;
#    dPath @6 :Float32;
#    vLat @7 :Float32;
#    vLeadK @8 :Float32;
#    aLeadK @9 :Float32;
#    fcw @10 :Bool;
#    status @11 :Bool;
#    aLeadTau @12 :Float32;
#    modelProb @13 :Float32;
#    radar @14 :Bool;
#
#    aLeadDEPRECATED @5 :Float32;
#  }
    add_v_by_lead = False #前走車に追いつくための増速処理

    global accel_lead_ctrl
    if CVS_FRAME % 30 == 13:
      try:
        with open('/tmp/accel_ctrl_disable.txt','r') as fp:
          accel_lead_ctrl_disable_str = fp.read()
          if accel_lead_ctrl_disable_str:
            accel_lead_ctrl_disable = int(accel_lead_ctrl_disable_str)
            if accel_lead_ctrl_disable == 0:
              accel_lead_ctrl = True
            else:
              accel_lead_ctrl = False
      except Exception as e:
        accel_lead_ctrl = True

    v_cruise_kph_1_15 = v_cruise_kph * 1.15
    if accel_lead_ctrl == True and hasLead == True and sm['radarState'].leadOne.modelProb > 0.5: #前走者がいる,信頼度が高い
      leadOne = sm['radarState'].leadOne
      d_rel = leadOne.dRel #前走者までの距離
      #a_rel = leadOne.aRel #前走者の加速？　離れていっている時はプラス,常にゼロ？UIで使ってるgetAEgoと違うようだ。
      v_abs = leadOne.vRel + v_ego #前走者の速度。vRelは相対速度のもよう。
      # with open('/tmp/debug_out_x','w') as fp:
      #   fp.write('%.0f[m],%.1f[k],%.2f[a]' % (leadOne.dRel , v_abs*3.6 , leadOne.aRel))
      if v_ego * 3.6 * 0.8 < d_rel / 0.98 and v_abs >= v_ego * 0.9: #例、時速50kmの時前走車までの距離が50m以上離れている。×0.8はd_relの値と実際の距離感との調整。&&相手が自分より速い。
        if v_ego * 3.6 >= v_cruise_kph * 0.98: #ACC設定速度がすでに出ている。
          add_v_by_lead = True #前走車に追いつくための増速処理が有効
          org_v_cruise_kph = v_cruise_kph
          if self.ac_vc_time < 1.0:
            self.ac_vc_time += 0.02
          self.ac_vc_time = clip(self.ac_vc_time,0.0,1.0)
          # v_cruise_kph *= 1.15 #ACC設定速度を1.5割増速
          v_cruise_kph = v_cruise_kph_1_15 * self.ac_vc_time + v_cruise_kph * (1-self.ac_vc_time)
          if v_cruise_kph > 105:
            v_cruise_kph = 105 #危ないのでひとまず時速105kmまで。
            if v_cruise_kph < org_v_cruise_kph:
              v_cruise_kph = org_v_cruise_kph #計算前の速度より遅くなったら、追従加速をやめる。
              self.ac_vc_time = 0
              add_v_by_lead = False
    if add_v_by_lead == False:
      if self.ac_vc_time > 0:
        self.ac_vc_time -= 0.02
      self.ac_vc_time = clip(self.ac_vc_time,0.0,1.0)
      v_cruise_kph = v_cruise_kph_1_15 * self.ac_vc_time + v_cruise_kph * (1-self.ac_vc_time)

    steerAng = sm['carState'].steeringAngleDeg - handle_center
    # orgSteerAng = steerAng , 使わなくても良くなった？
    limit_vc = V_CRUISE_MAX
    limit_vc_h = V_CRUISE_MAX
    #ml_csv = ""

    global decel_lead_ctrl
    if CVS_FRAME % 30 == 29:
      try:
        with open('/tmp/decel_ctrl_disable.txt','r') as fp:
          decel_lead_ctrl_disable_str = fp.read()
          if decel_lead_ctrl_disable_str:
            decel_lead_ctrl_disable = int(decel_lead_ctrl_disable_str)
            if decel_lead_ctrl_disable == 0:
              decel_lead_ctrl = True
            else:
              decel_lead_ctrl = False
      except Exception as e:
        decel_lead_ctrl = True

    if decel_lead_ctrl == True and len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      #path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      max_yp = 0
      for yp in md.position.y: #path_y
        max_yp = yp if abs(yp) > abs(max_yp) else max_yp
        if abs(steerAng) < abs(max_yp) / 2.5:
          steerAng = (-max_yp / 2.5)
      limit_vc = V_CRUISE_MAX if abs(steerAng) <= LIMIT_VC_B else LIMIT_VC_A / (abs(steerAng) - LIMIT_VC_B) + LIMIT_VC_C
      limit_vc_h = V_CRUISE_MAX if abs(steerAng) <= LIMIT_VC_BH else LIMIT_VC_AH / (abs(steerAng) - LIMIT_VC_BH) + LIMIT_VC_CH
      #前方カーブ機械学習用ファイルデータ生成処理。ひとまず保留
      #if CVS_FRAME % 10 == 0 and v_ego * 3.6 > 20: # over 20km/h
      #  ml_csv = '%0.2f,' % v_cruise_kph
      #  for i in path_y:
      #    ml_csv += '%0.2f,' % i
    v_cruise_kph_org = v_cruise_kph
    limit_vc_th = 95-5 #85-5 #80-4
    limit_vc_tl = 60-4 #50-4 #65-4 #70-4
    if v_cruise_kph_org > limit_vc_th:
      limit_vc = limit_vc_h
    elif v_cruise_kph_org >= limit_vc_tl:
      limit_vc = (limit_vc * ((limit_vc_th)-v_cruise_kph_org) + limit_vc_h * (v_cruise_kph_org - (limit_vc_tl))) / (limit_vc_th - limit_vc_tl)
    v_cruise_kph = limit_vc if limit_vc < v_cruise_kph else v_cruise_kph
    if CVS_FRAME % 5 == 2:
      with open('/tmp/limit_vc_info.txt','w') as fp:
        fp.write('%d' % (limit_vc))
    if True: #CVS_FRAME % 5 == 1:
      #os.environ['steer_ang_info'] = '%f' % (steerAng)
      with open('/tmp/steer_ang_info.txt','w') as fp:
       fp.write('%f' % (steerAng))
       #fp.write('%f' % (-max_yp / 2.5))
    if CVS_FRAME % 5 == 0:
      with open('/tmp/cruise_info.txt','w') as fp:
        #fp.write('%d/%d' % (v_cruise_kph_org , (limit_vc if limit_vc < V_CRUISE_MAX else V_CRUISE_MAX)))
        if v_cruise_kph == limit_vc:
          if cruise_info_power_up:
            fp.write('%d;' % (v_cruise_kph))
          else:
            fp.write('%d.' % (v_cruise_kph))
        else:
          if add_v_by_lead == True:
            if cruise_info_power_up:
              fp.write('%d;' % (v_cruise_kph_org))
            else:
              fp.write(',%d' % (v_cruise_kph_org))
          else:
            vo = v_cruise_kph_org
            if int(vo) == 59 or int(vo) == 61:
              vo += 0.5 #メーター表示誤差補正
            if cruise_info_power_up:
              fp.write('%d;' % (vo))
            elif limitspeed_set == True:
              fp.write(';%d' % (vo))
            else:
              fp.write('%d' % (vo))
    #if CVS_FRAME % 10 == 0 and limit_vc < V_CRUISE_MAX and v_ego * 3.6 > 20: # over 20km/h
    #  with open('./ml_data.csv','a') as fp:
    #    fp.write('%s%0.2f\n' % (ml_csv , limit_vc))
    CVS_FRAME += 1
    global v_cruise , v_cruise_old
    v_cruise_old = v_cruise

    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS # * red_signal_speed_down
    long_speeddown_flag = False
    if desired_path_x_rate > 0.1 and desired_path_x_rate < 1.0:
      long_speeddown_disable = 0
      try:
        with open('/tmp/long_speeddown_disable.txt','r') as fp:
          long_speeddown_disable_str = fp.read()
          if long_speeddown_disable_str:
            long_speeddown_disable = int(long_speeddown_disable_str) #0で有効。
      except Exception as e:
        pass
      if long_speeddown_disable == 0:
        long_speeddown_flag = True #このフラグで信号ストップ条件の切り替えを行っている。
        # v_cruise *= desired_path_x_rate #赤信号やAボタンモード関係なく、path_xの情報を加味してみる。
        v_cruise_org = min(v_cruise_kph_org, V_CRUISE_MAX) * CV.KPH_TO_MS #ACC設定速度
        # v_cruise:カーブ減速含むACC設定速度,もしカーブ減速(v_cruise)がイチロウロング減速(v_cruise_org * desired_path_x_rate)より強い時は、カーブ減速をそのまま採用する。
        if v_cruise > v_cruise_org * (desired_path_x_rate**0.85):
          v_cruise = v_cruise_org * (desired_path_x_rate**0.85) #カーブ減速がなければv_cruise==v_cruise_orgなので、従来のアルゴリズムは保たれる。
    if long_speeddown_flag == False:
      v_cruise *= red_signal_speed_down
    
    if OP_ENABLE_v_cruise_kph != 0 and v_cruise_kph <= 1.2: #km/h
      #v_cruise = 0 #ワンペダル停止処理
      v_cruise = interp(v_ego*3.6,[0,5,8,15,60],[0,0,3,5,20]) / 3.6 #速度が大きい時は1/3を目指す
      # if red_signal_scan_span > 0: これでブレーキングの強さが変わったら制御しづらいのでやめる。
      #   v_cruise *= interp(red_signal_scan_span , [0,25,100] , [1,1,1.5]) #2〜3のスパンが長いと、速度を落とすのに距離が伸びるように。

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [MIN_ACCEL, MAX_ACCEL]
      accel_limits_turns = [MIN_ACCEL, MAX_ACCEL]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    if len(sm['modelV2'].temporalPose.trans):
      self.v_model_error = sm['modelV2'].temporalPose.trans[0] - v_ego

    if False:
      msv_desired = max(0,self.v_desired_filter.x * 3.6)
      msc = "A:%5.1fkm/h" % (v_cruise_kph_org)
      if int(min(v_cruise_kph_org,V_CRUISE_MAX) / 2) - len(msc) > 0:
        for vm in range(int(min(v_cruise_kph_org,V_CRUISE_MAX) / 2) - len(msc)):
          msc += "#"
      msl = "L:%5.1fkm/h" % (limit_vc)
      if int(min(limit_vc,V_CRUISE_MAX) / 2) - len(msl) > 0:
        for vml in range(int(min(limit_vc,V_CRUISE_MAX) / 2) - len(msl)):
          msl += "<"
      v_ego_2 = max(0,v_ego * 3.6)
      msv = "V:%5.1fkm/h" % (v_ego_2)
      if msv_desired <= v_ego_2:
        if int(min(msv_desired,V_CRUISE_MAX) / 2) - len(msv) > 0:
          for vml in range(int(min(msv_desired,V_CRUISE_MAX) / 2) - len(msv)):
            msv += "|"
        if int(min(v_ego_2,V_CRUISE_MAX) / 2) - len(msv) > 0:
          for vml in range(int(min(v_ego_2,V_CRUISE_MAX) / 2) - len(msv)):
            msv += "<"
      else:
        if int(min(v_ego_2,V_CRUISE_MAX) / 2) - len(msv) > 0:
          for vml in range(int(min(v_ego_2,V_CRUISE_MAX) / 2) - len(msv)):
            msv += "|"
        if int(min(msv_desired,V_CRUISE_MAX) / 2) - len(msv) > 0:
          for vml in range(int(min(msv_desired,V_CRUISE_MAX) / 2) - len(msv)):
            msv += ">"
      msv += "%+.1fkm/h" % (msv_desired-v_ego_2)
      with open('/tmp/debug_out_v','w') as fp:
        #fp.write('[%i],vc:%.1f(%.1f) , v:%.2f , vd:%.2f[km/h] ; ah:%.2f bh:%.2f ch:%.2f' % (prev_accel_constraint , v_cruise_kph_org , limit_vc , v_ego * 3.6 , self.v_desired_filter.x* 3.6 , LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH) )
        #fp.write('[%i],vc:%.1f(%.1f) , v:%.2f , vd:%.2f[km/h] ; a:%.2f , ad:%.2f[m/ss]' % (prev_accel_constraint , v_cruise_kph , limit_vc , v_ego * 3.6 , self.v_desired_filter.x* 3.6 , a_ego , self.a_desired) )
        fp.write('ah:%.2f bh:%.2f ch:%.2f\n' % (LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH) )
        #fp.write('op:[%d] vk:%.2f gs:%.2fkm/h\n' % (OP_ENABLE_PREV,OP_ENABLE_v_cruise_kph,OP_ENABLE_gas_speed*3.6) )
        fp.write("%s\n%s\n%s" % (msc ,msl ,msv))

    #with open('/tmp/debug_out_vd','w') as fp:
    #  fp.write('vc:%.2f[km/h] , vd:%.2f[km/h] ; ang:%.2f[deg] ; v:%.2f[km/h]' % (v_cruise * 3.6 , self.v_desired_filter.x* 3.6,steerAng , v_ego * 3.6) )
    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds

    a_desired_mul = 1.0
    vl = 0
    vd = 0
    lcd = 0
    if hasLead == True and sm['radarState'].leadOne.modelProb > 0.5: #前走者がいる,信頼度が高い
      leadOne = sm['radarState'].leadOne
      to_lead_distance = 35 #35m以上空いている
      add_lead_distance = v_ego * 3.6 #速度km/hを車間距離(m)と見做す
      add_lead_distance = 0 if add_lead_distance < 50 else add_lead_distance - 50
      to_lead_distance += add_lead_distance #時速50km/h以上ならto_lead_distanceをのばす。時速100km/hでは85mになる。
      if leadOne.dRel > to_lead_distance:
        lcd = leadOne.dRel #前走者までの距離
        lcd -= to_lead_distance #0〜
        lcd /= ((70 + add_lead_distance) -to_lead_distance) #70m離れていたら1.0(時速50km以下の時、時速100kmでは130mとなる)
        if lcd > 1:
          lcd = 1
    if (hasLead == False or lcd > 0) and self.a_desired > 0 and v_ego >= 1/3.6 and sm['carState'].gasPressed == False: #前走者がいない。加速中
      if hasLead == False:
        lcd = 1.0 #前走車がいなければlcd=1扱い。
      vl = v_cruise
      if vl > 100/3.6:
        vl = 100/3.6
      #vl *= 0.60 #加速は目標速度の半分程度でおしまい。そうしないと増速しすぎる
      vl = interp(vl, START_DASH_SPEEDS, START_DASH_CUT) #定数倍ではなく、表で考えてみる。
      vd = v_ego
      if vd > vl:
        vd = vl #vdの最大値はvl
      if vl > 0:
        vd /= vl #0〜1
        vd = 1 - vd #1〜0
        add_k = interp(v_ego,[0,10/3.6],[0.1,0.2]) #0.2固定だと雨の日ホイールスピンする
        a_desired_mul = 1 + add_k*vd*lcd #1.2〜1倍で、(最大100km/hかv_cruise)*0.60に達すると1になる。→新方法は折れ線グラフの表から決定。速度が大きくなると大体目標値-20くらいにしている。これから検証。
        try:
          with open('/tmp/start_accel_power_up_disp_enable.txt','r') as fp:
            start_accel_power_up_disp_enable_str = fp.read()
            if start_accel_power_up_disp_enable_str:
              start_accel_power_up_disp_enable = int(start_accel_power_up_disp_enable_str)
              if start_accel_power_up_disp_enable == 0:
                a_desired_mul = 1 #スタート加速増なし
        except Exception as e:
          a_desired_mul = 1 #ファイルがなくてもスタート加速増なし

    if a_desired_mul == 1.0 or v_ego < 1/3.6:
      cruise_info_power_up = False
    else:
      cruise_info_power_up = True

    if OP_ENABLE_v_cruise_kph != 0 and v_cruise_kph <= 1.2: #km/h
      if sm['carState'].gasPressed == False and self.a_desired > 0:
        self.a_desired = 0 #アクセル離して加速ならゼロに。
      if self.a_desired < 0:
        #ワンペダル停止の減速を強めてみる。
        a_desired_mul = interp(v_ego,[0.0,10/3.6,20/3.6,40/3.6],[1.0,1.02,1.06,1.17]) #30km/hあたりから減速が強くなり始める->低速でもある程度強くしてみる。

    #with open('/tmp/debug_out_v','w') as fp:
    #  fp.write("lead:%d(lcd:%.2f) a:%.2f , m:%.2f(%d) , vl:%dkm/h , vd:%.2f" % (hasLead,lcd,self.a_desired,a_desired_mul,cruise_info_power_up,vl*3.6,vd))
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired*a_desired_mul + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired*a_desired_mul - 0.05)
    self.mpc.set_weights(prev_accel_constraint)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired*a_desired_mul)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j)

    self.v_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    pm.send('longitudinalPlan', plan_send)

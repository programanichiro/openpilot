#!/usr/bin/env python3
import os
import math
import numpy as np
from openpilot.common.params import Params
from cereal import log

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from opendbc.car.toyota.values import TSS2_CAR,ToyotaFlags
params = Params()
#g_tss_type = 0
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
with open('/dev/shm/red_signal_scan_flag.txt','w') as fp:
  fp.write('%d' % (0))
path_x_old_signal = 0
path_x_old_signal_check = 0
desired_path_x_speeds    = [0,5 ,10  ,15  ,20  ,30  ,40   ,50   ,60   ,70    ,80    ,90    ,100   ]
desired_path_x_by_speeds = [0,15,55  ,60  ,70-5,85-5,125-8,150-9,170-9,190-10,220-10,240-10,255-10] #MLSIM以降用に見直し。スピードが上がらないので、これをちょっと下げる。
#desired_path_x_speeds    = [0,5 ,10  ,15  ,20  ,30  ,40   ,50   ,60   ,70   ,80   ,90   ,100  ]
#desired_path_x_by_speeds = [0,15,55  ,60  ,70-5,85-5,125-5,150-5,170-5,190-5,220-5,240-5,255-5] #piQ用に50,60,70の係数見直し
#desired_path_x_by_speeds = [0,15,55  ,60  ,70-5,85-5,125-5,155-5,175-5,193-5,220-5,240-5,255-5] #50,60の係数見直し
#desired_path_x_by_speeds =[0,15,55  ,60  ,70-5,85-5,125-5,160-5,178-5,193-5,220-5,240-5,255-5] #toro_555,2023/4/27
#desired_path_x_by_speeds =[0,15,60-5,65-5,75-5,95-5,125-5,150-5,170-5,190-5,220-5,240-5,255-5] #オリジナル
long_speeddown_flag = False
before_v_cruise_kph_max_1 = 0
OnePedal_Low_speed_auto_engage = True # Falseでこれまで通り低速自動制御なしに戻る

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

#START_DASH_CUT   = [0, 17/3.6, 26/3.6, 36/3.6, 45/3.6, 55/3.6, 64/3.6, 74/3.6, 83/3.6,  93/3.6]
START_DASH_CUT    = [0, 27/3.6, 35/3.6, 43/3.6, 51/3.6, 60/3.6, 69/3.6, 78/3.6, 86/3.6,  95/3.6]
START_DASH_SPEEDS = [0, 31/3.6, 41/3.6, 51/3.6, 61/3.6, 70/3.6, 80/3.6, 90/3.6, 100/3.6, 110/3.6]

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

phv_2019 = params.get_bool("DisableMaxSpeedModify")

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    self.red_signals = np.zeros(10)
    self.red_signal_path_xs = np.zeros(5)
    self.old_red_signal_path_xs = 0
    self.night_time = 100 #変数名がわかりにくいが環境光の強さが0〜100で取得できる。
    self.night_time_refresh_ct = 0
    self.desired_path_x_rates = np.zeros(5)
    self.ac_vc_time = 0.0
    self.limitspeed_point = 0.0
    self.limitspeed_point_avg = 0.0
    self.limitspeed_point_dim = []
    self.v_cruise_kph_1_15 = 0 #前走車まで追従する速度
    self.lead_v_abs = []
    self.a_desired_mul = 1.0
    self.v_cruise_onep_k = 1.0
    self.red_signal_eP_iP_flag = 0

    try:
      with open('/dev/shm/dexp_sw_mode.txt','r') as fp:
        dexp_sw_mode_str = fp.read()
        if dexp_sw_mode_str:
          if int(dexp_sw_mode_str) >= 1: #dynamic experimental mode
            with open('/dev/shm/long_speeddown_disable.txt','w') as fp:
              fp.write('%d' % (1)) #初期はイチロウロング無効
    except Exception as e:
      pass

    self.dexp_mode_min = 20/3.6
    self.dexp_mode_max = 23/3.6
    self.hasLead_1s = False
    self.hasLead_1s_frame = 0
    self.weak_one_pedal = False #True:チョン押し後の16オーバー判定無効
    self.max_one_pedal = False #True:低速から16オーバーエンゲージした

    if self.CP.carFingerprint in TSS2_CAR or (self.CP.flags & ToyotaFlags.POWER_STEERING_TSS2.value): #47700はTSS2相当の操舵範囲
      LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,13.6,57.0 , 92-4      ,65.5-4      ,31.0      ) #ハンドル60度で時速30km/h程度まで下げる設定。

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    # v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    vk_ego = v_ego # sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    dexp_mode = False
    try:
      with open('/dev/shm/dexp_sw_mode.txt','r') as fp:
        dexp_sw_mode_str = fp.read()
        if dexp_sw_mode_str:
          if int(dexp_sw_mode_str) >= 1: #dynamic experimental mode
            dexp_mode = True
    except Exception as e:
      pass

    hasLead = sm['radarState'].leadOne.status
    # hasLeadの短時間切り替えによるカメラのバタつきを抑える。
    if self.hasLead_1s != hasLead:
      if self.hasLead_1s_frame >= 15: #30カウントくらいで1秒
        self.hasLead_1s = hasLead
      else:
        self.hasLead_1s_frame += 1
    else:
      self.hasLead_1s_frame = 0

    if dexp_mode:
      if not sm['selfdriveState'].experimentalMode:
        with open('/dev/shm/long_speeddown_disable.txt','w') as fp:
          if self.hasLead_1s:
            fp.write('%d' % (1)) #前走車がいるからイチロウロング無効
          else:
            fp.write('%d' % (0)) #前走車がいないからイチロウロング有効

        if (self.hasLead_1s == False and vk_ego <= self.dexp_mode_min and sm['carState'].gasPressed == False) or (vk_ego > 0.1/3.6 and vk_ego <= 45/3.6 and (sm['carState'].leftBlinker or sm['carState'].rightBlinker)):
          params.put_bool("ExperimentalMode", True) # blended
          with open('/dev/shm/long_speeddown_disable.txt','w') as fp:
            fp.write('%d' % (1)) #イチロウロング無効
      else:
        if (self.hasLead_1s == True or vk_ego > self.dexp_mode_max or sm['carState'].gasPressed == True) and (vk_ego <= 0.1/3.6 or vk_ego > 45/3.6 or (sm['carState'].leftBlinker == False and sm['carState'].rightBlinker == False)):
          params.put_bool("ExperimentalMode", False) # acc

    global CVS_FRAME , handle_center , OP_ENABLE_PREV , OP_ENABLE_v_cruise_kph , OP_ENABLE_gas_speed , OP_ENABLE_ACCEL_RELEASE , OP_ACCEL_PUSH , on_onepedal_ct , cruise_info_power_up , one_pedal_chenge_restrict_time #, g_tss_type
    min_acc_speed = 31
    v_cruise_kph = sm['carState'].vCruise
    if self.CP.carFingerprint not in TSS2_CAR:
      tss_type = 1
      v_cruise_kph = (55 - (55 - (v_cruise_kph+4)) * 2 - 4) if v_cruise_kph < (55 - 4) else v_cruise_kph
      if phv_2019 == False:
      # v_cruise_kph = (110 + ((v_cruise_kph+6) - 110) * 3 - 6) if v_cruise_kph > (110 - 6) else v_cruise_kph #最大119
      # v_cruise_kph = (107 + ((v_cruise_kph+6) - 107) * 2 - 6) if v_cruise_kph > (107 - 6) else v_cruise_kph #最大119 -> 114 -> 117に。
      # v_cruise_kph = (106 + ((v_cruise_kph+6) - 106) * 2 - 6) if v_cruise_kph > (106 - 6) else v_cruise_kph #最大118に。
        v_cruise_kph = (109 + ((v_cruise_kph+6) - 109) * 2 - 6) if v_cruise_kph > (109 - 6) else v_cruise_kph #最大115に。

#100,101,102,103,104,105,106,107,108,109
#100,101,102,103,105,107,109,111,113,115 ;407 *今これ
#100,101,102,104,106,108,110,112,114,116
#100,101,103,105,107,109,111,113,115,117 ;409
#100,102,104,106,108,110,112,114,116,118 ;410

      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('../../../tss_type_info.txt','w') as fp:
          fp.write('%d' % (1))
    else:
      tss_type = 2
      min_acc_speed = 27
      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('../../../tss_type_info.txt','w') as fp:
          fp.write('%d' % (2))
    #g_tss_type = tss_type
    if v_cruise_kph < min_acc_speed:
      v_cruise_kph = min_acc_speed #念のため

    accel_engaged_str = None
    try:
      with open('/dev/shm/accel_engaged.txt','r') as fp:
        accel_engaged_str = fp.read()
    except Exception as e:
      pass
    one_pedal = False
    on_accel0 = False #押した瞬間
    if vk_ego <= 3/3.6 or (OP_ACCEL_PUSH == False and sm['carState'].gasPressed):
      if accel_engaged_str:
        if int(accel_engaged_str) >= 3: #ワンペダルモード
          one_pedal = True
          if OP_ACCEL_PUSH == False and sm['carState'].gasPressed:
            if on_onepedal_ct < 0 and self.weak_one_pedal == False:
              on_onepedal_ct = 0 #ワンペダルかアクセル判定開始
    if on_onepedal_ct >= 0:
      on_onepedal_ct += 1
      if on_onepedal_ct > 5:# 1秒後に。フレームレートを実測すると、30カウントくらいで1秒？
        if sm['carState'].gas < 0.32: #アクセルが弱いかチョン押しなら
          on_accel0 = True #ワンペダルに変更
        on_onepedal_ct = -1 #アクセル判定消去
    if on_accel0 and vk_ego > 1/3.6 and self.max_one_pedal == False: #オートパイロット中にアクセルを弱めに操作したらワンペダルモード有効。ただし先頭スタートは除く。
      if sm['selfdriveState'].enabled and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > 1.0 / 3.6):
        self.weak_one_pedal = True
        OP_ENABLE_ACCEL_RELEASE = True #アクセルコントロールを許可しない
        with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (1)) #prompt.wav音を鳴らしてみる。
          #しばらくやってもなかなか出ない？fp.write('%d' % (3)) #デバッグでpo.wav音を鳴らす。
      OP_ENABLE_v_cruise_kph = v_cruise_kph
      OP_ENABLE_gas_speed = 1.0 / 3.6
      one_pedal_chenge_restrict_time = 20
    if one_pedal_chenge_restrict_time > 0:
      one_pedal_chenge_restrict_time -= 1
    if one_pedal == True and vk_ego < 0.1/3.6 and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > 1.0 / 3.6) and sm['selfdriveState'].enabled and sm['carState'].gasPressed == False:
      force_one_pedal_set = False
      try:
        with open('/dev/shm/force_one_pedal.txt','r') as fp:
          force_one_pedal_str = fp.read()
          if force_one_pedal_str:
            if int(force_one_pedal_str) == 1:
              force_one_pedal_set = True
              OP_ENABLE_v_cruise_kph = v_cruise_kph
              OP_ENABLE_gas_speed = 1.0 / 3.6
      except Exception as e:
        pass
      if force_one_pedal_set == True:
        with open('/dev/shm/force_one_pedal.txt','w') as fp:
          fp.write('%d' % (0))
        with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (1)) #MAXを1に戻すのでprompt.wavを鳴らす。

    sm_longControlState = sm['controlsState'].longControlState
    if sm_longControlState == LongCtrlState.off:
      # if sm['carState'].gasPressed and sm['selfdriveState'].enabled: #アクセル踏んでエンゲージ中なら
      if sm['selfdriveState'].enabled: #アクセル踏む条件を無視してみる。
        sm_longControlState = LongCtrlState.pid #0.8.14からアクセルONでLongCtrlState.offとなるため、従来動作をシミュレート
    if OP_ENABLE_PREV == False and sm_longControlState != LongCtrlState.off and (((one_pedal or vk_ego > 3/3.6) and vk_ego < min_acc_speed/3.6 and int(v_cruise_kph) == min_acc_speed) or sm['carState'].gasPressed):
       #速度が時速３km以上かつ31km未満かつsm['carState'].vCruiseが最低速度なら、アクセル踏んでなくても無条件にエクストラエンゲージする
    #if tss2_flag == False and OP_ENABLE_PREV == False and sm['controlsState'].longControlState != LongCtrlState.off and sm['carState'].gasPressed:
      #アクセル踏みながらのOP有効化の瞬間
      OP_ENABLE_v_cruise_kph = v_cruise_kph
      if one_pedal_chenge_restrict_time == 0:
        OP_ENABLE_gas_speed = vk_ego
      if accel_engaged_str:
        if int(accel_engaged_str) >= 3 and sm['carState'].gasPressed == False: #ワンペダルモード(開始時にアクセル操作していたら低速エンゲージとする)
          OP_ENABLE_gas_speed = 1.0 / 3.6
      OP_ENABLE_ACCEL_RELEASE = False

    if OnePedal_Low_speed_auto_engage and self.weak_one_pedal == False and OP_ENABLE_v_cruise_kph != 0 and one_pedal_chenge_restrict_time == 0 and sm['carState'].gasPressed and vk_ego >= 16/3.6 and vk_ego < min_acc_speed/3.6 and OP_ENABLE_gas_speed == 1.0/3.6 and a_ego > 0:
      with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
        fp.write('%d' % (2)) #MAXが上昇するのでengage.wavを鳴らす。
      self.max_one_pedal = True
      OP_ENABLE_ACCEL_RELEASE = False #ワンペダル中の低速操作で常にアクセル操作をMAXに伝える。アクセルを放しても減速しなくなる。

    if sm_longControlState != LongCtrlState.off:
      OP_ENABLE_PREV = True
      if sm['carState'].gasPressed and OP_ENABLE_ACCEL_RELEASE == False:
        if one_pedal_chenge_restrict_time == 0:
          OP_ENABLE_gas_speed = vk_ego
          if OnePedal_Low_speed_auto_engage and vk_ego >= min_acc_speed/3.6 and a_ego > 0 and self.weak_one_pedal == False:
            OP_ENABLE_v_cruise_kph = 0 #通常クルーズへ
      elif OnePedal_Low_speed_auto_engage:
        if sm['carState'].gasPressed and vk_ego >= min_acc_speed/3.6 and a_ego > 0 and OP_ENABLE_v_cruise_kph != 0 and self.weak_one_pedal == False: # and OP_ENABLE_gas_speed == 1.0/3.6
          with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
            fp.write('%d' % (2)) #MAXが上昇するのでengage.wavを鳴らす。
          OP_ENABLE_v_cruise_kph = 0 #通常クルーズへ
          self.max_one_pedal = True #再度ワンペダルに落ちるのを抑制。
    else:
      OP_ENABLE_PREV = False
      OP_ENABLE_v_cruise_kph = 0
    if sm['carState'].gasPressed == False: #一旦アクセルを離したら、クルーズ速度は変更しない。変更を許すと、ACC速度とMAX速度の乖離が大きくなり過ぎる可能性があるから。
      OP_ENABLE_ACCEL_RELEASE = True
      OP_ACCEL_PUSH = False #アクセル離した
      self.weak_one_pedal = False
      self.max_one_pedal = False
    else:
      OP_ACCEL_PUSH = True #アクセル押した

    md = sm['modelV2']
    # hasLead = sm['radarState'].leadOne.status
    #distLead_near = sm['radarState'].leadOne.dRel < np.interp(vk_ego*3.6 , [30,80] , [50,120]) #前走車が近ければTrue
    distLead_near = hasLead #and sm['radarState'].leadOne.dRel < np.interp(vk_ego*3.6 , [30,80] , [60,130]) #前走車が近ければTrue,最近前走者が遠くてもワンペダル遷移してしまうので、ちょっと調整。
    global signal_scan_ct,path_x_old_signal,path_x_old_signal_check , red_signal_scan_flag
    if vk_ego <= 0.1/3.6 and (OP_ENABLE_v_cruise_kph > 0 or one_pedal == False or (OP_ENABLE_v_cruise_kph == 0 and (hasLead == False or distLead_near == False))) and sm['selfdriveState'].enabled and sm['carState'].gasPressed == False: #and (hasLead == False or (sm['radarState'].leadOne.dRel > 40 and sm['radarState'].leadOne.modelProb > 0.5)):
      #速度ゼロでエンゲージ中、前走車なしでアクセル踏んでない。
      steer_ang = sm['carState'].steeringAngleDeg - handle_center
      # 停止時の青信号発進抑制、一時的に緩和、15->50度
      if (abs(steer_ang) < 50 or one_pedal == False) and len(md.position.x) == ModelConstants.IDX_N and len(md.orientation.x) == ModelConstants.IDX_N: #ワンペダルならある程度ハンドルが正面を向いていること。
        #path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
        path_x = md.position.x #path_xyz[:,0]
        # with open('/tmp/debug_out_k','w') as fp: #path_xの中を解析して、ビュンと伸びる瞬間を判断したい。
        #   fp.write('x:%.2f,ct:%d,px:%.1f,v:%.1f' % (path_x[ModelConstants.IDX_N -1],signal_scan_ct,path_x_old_signal_check,vk_ego))
        #   #fp.write('{0}\n'.format(['%0.2f' % i for i in path_x]))
        #   fp.write('l:%d(%.2f),%.2f[m],x:%.2f' % (hasLead ,sm['radarState'].leadOne.modelProb , sm['radarState'].leadOne.dRel , path_x[ModelConstants.IDX_N -1]))
        with open('/dev/shm/blue_signal_chk.txt','w') as fp: #path_xの中を解析して、ビュンと伸びる瞬間を判断したい。
          fp.write('%d' % (int(path_x[ModelConstants.IDX_N -1])))
        half_limit = 25 #40
        if (path_x_old_signal < 2) and path_x[ModelConstants.IDX_N -1] > half_limit:
          path_x_old_signal_check = path_x[ModelConstants.IDX_N -1] #ゆっくり立ち上がったらこれはTrueにならない。
        path_x_base_limit = 30 #64.0 #70.0 , この座標値超で青信号スタート発火。
        if path_x[ModelConstants.IDX_N -1] > path_x_base_limit or path_x_old_signal_check > half_limit: #青信号判定の瞬間
          path_x_old_signal_check += path_x[ModelConstants.IDX_N -1] #最初の立ち上がりは2倍される
          signal_scan_ct += 1 #横道からの進入車でパスが伸びたのを勘違いするので、バッファを設ける。
          limit_8 = 8 if path_x[ModelConstants.IDX_N -1] > path_x_base_limit else 16
          if signal_scan_ct > limit_8 and signal_scan_ct < 100 and (path_x[ModelConstants.IDX_N -1] > path_x_base_limit or (path_x_old_signal_check-half_limit) > 1.25*limit_8 * half_limit): #path_x_old_signal_check-half_limitの20倍程度
            with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
              if sm['driverMonitoringState'].activePolicy == log.DriverMonitoringState.MonitoringPolicy.vision and sm['driverMonitoringState'].alertLevel == log.DriverMonitoringState.AlertLevel.none: #よそ見をしていたら発進しない。
                if OP_ENABLE_v_cruise_kph > 0: #MAX==1の状態。
                  fp.write('%d' % (2)) #engage.wavを鳴らす。
                else:
                  fp.write('%d' % (1)) #prompt.wavを鳴らす。
                OP_ENABLE_v_cruise_kph = 0 #エクストラエンゲージ解除
                # if red_signal_scan_flag == 3:
                #   red_signal_scan_flag = 0 #赤ブレーキキャンセルならゼロに戻す。
                if self.red_signal_eP_iP_flag != 0:
                  self.red_signal_eP_iP_flag = 0
                  with open('/dev/shm/red_signal_eP_iP_set.txt','w') as fp:
                    fp.write('%d' % (0))
              else:
                fp.write('%d' % (1)) #よそ見してたらprompt.wavを鳴らす。
            signal_scan_ct = 200 #2回鳴るのを防止
            path_x_old_signal_check = 0
        else:
          signal_scan_ct = 0 if signal_scan_ct < 4 else signal_scan_ct - 4
          path_x_old_signal_check = 0
        path_x_old_signal = path_x[ModelConstants.IDX_N -1]
    else:
      signal_scan_ct = 0 if signal_scan_ct < 4 else signal_scan_ct - 4
    if path_x_old_signal < 20:
      path_x_old_signal_check = 0

    if a_ego > 0 and vk_ego > 24/3.6 and OP_ENABLE_v_cruise_kph > 0 and sm['selfdriveState'].enabled and sm['carState'].gas > 0.32: #アクセル強押しでワンペダルからオートパイロットへ。30キロ(min_acc_speed)以上から24キロ以上へ変更
      OP_ENABLE_v_cruise_kph = 0 #エクストラエンゲージ解除
      signal_scan_ct = 200 #このあと信号スタート判定されてprompt.wavが鳴るのを防止する。
      with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
        fp.write('%d' % (2)) #engage.wavを鳴らす。

    # if CVS_FRAME % 10 == 0 and vk_ego >= 1/3.6 and OP_ENABLE_v_cruise_kph > 0 and sm['selfdriveState'].enabled:
    #   try: #首ジェスチャーでエクストラエンゲージ解除をやろうと思ったが、誤作動懸念で保留。アクセル踏み込みとレバーアップがあるし
    #     with open('/dev/shm/gesture_onpe2AP.txt','r') as fp:
    #       gesture_onpe2AP_str = fp.read()
    #       if gesture_onpe2AP_str and int(gesture_onpe2AP_str) == 1:
    #         OP_ENABLE_v_cruise_kph = 0 #エクストラエンゲージ解除
    #         signal_scan_ct = 200 #このあと信号スタート判定されてprompt.wavが鳴るのを防止する。
    #         with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
    #           fp.write('%d' % (2)) #engage.wavを鳴らす。
    #         with open('/dev/shm/gesture_onpe2AP.txt','w') as fp:
    #           fp.write('%d' % (0))
    #   except Exception as e:
    #     pass

    global red_signal_scan_ct , red_signal_scan_ct_2 , red_signal_speed_down_before , red_signal_scan_span , long_speeddown_flag , before_v_cruise_kph_max_1
    red_signal_scan_flag_1 = red_signal_scan_flag
    red_signal_speed_down = 1.0
    desired_path_x_rate = 1.0 #一般的な減速制御
    set_red_signal_scan_flag_3 = False
    if len(md.position.x) == ModelConstants.IDX_N and len(md.orientation.x) == ModelConstants.IDX_N: #これがFalseのケースは想定外
      path_x = md.position.x #path_xyz[:,0]
      red_signal_v_ego = 4/3.6 #この速度超で赤信号認識。
      if (hasLead == False or distLead_near == False) and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > red_signal_v_ego):
        if red_signal_scan_flag_1 != 3 and vk_ego > red_signal_v_ego:
          red_signal_scan_flag_1 = 1 #赤信号センシング

      # path_x[ModelConstants.IDX_N -1]が増加方向の時は弾きたい。
      self.red_signal_path_xs = np.append(self.red_signal_path_xs,path_x[ModelConstants.IDX_N -1])
      self.red_signal_path_xs = np.delete(self.red_signal_path_xs , [0])
      sum_red_signal_path_xs = np.sum(self.red_signal_path_xs)

      if (hasLead == False or distLead_near == False) and path_x[ModelConstants.IDX_N -1] < np.interp(vk_ego*3.6 , [0,10,20,30,40,50,55,60] , [20,30,50,70,80,90,105,120]): #60
        red_signal = "●"
        self.red_signals = np.append(self.red_signals,1)
      else:
        red_signal = "◯"
        self.red_signals = np.append(self.red_signals,0)
      self.red_signals = np.delete(self.red_signals , [0])
      red_signals_sum = np.sum(self.red_signals)
      if red_signals_sum > self.red_signals.size * 0.7:
        red_signals_mark = "■"
        if red_signal_scan_flag_1 != 3 and vk_ego > red_signal_v_ego:
          if red_signal_scan_flag < 2:
            red_signal_scan_ct_2 = 0
          red_signal_scan_ct_2 += 1 #red_signal_scan_flagが2になった瞬間から加算し始める。
          red_signal_scan_flag_1 = 2 #赤信号検出
          #この信号認識状態 and sum_red_signal_path_xs < self.old_red_signal_path_xsなら速度を落とし始めてもいい？ 473行のv_cruiseを1割落とすとか
          if vk_ego > 20/3.6 and sum_red_signal_path_xs < self.old_red_signal_path_xs:
            red_signal_speed_down = np.interp(vk_ego*3.6 , [10,20,30,40,50,55,60] , [0.97,0.96,0.95,0.94,0.93,0.92,0.91])
            red_signal_scan_ct_2_rate = 200 if red_signal_scan_ct_2 > 200 else red_signal_scan_ct_2 #最大200
            red_signal_speed_down -= red_signal_scan_ct_2_rate * 0.3 / 200 #徐々にブレーキが強くなる
            red_signal_speed_down_before = red_signal_speed_down
          elif red_signal_speed_down_before > 0:
            red_signal_speed_down = red_signal_speed_down_before #条件に外れたら、一度だけ過去を参照する。
            red_signal_speed_down_before = 0
      else:
        red_signals_mark = "□"

      desired_path_x_by_speed = np.interp(vk_ego*3.6,desired_path_x_speeds,desired_path_x_by_speeds)
      desired_path_x_rate = 0 if desired_path_x_by_speed <= 0.01 else path_x[ModelConstants.IDX_N -1]/desired_path_x_by_speed
      self.desired_path_x_rates = np.append(self.desired_path_x_rates,desired_path_x_rate)
      self.desired_path_x_rates = np.delete(self.desired_path_x_rates , [0])
      desired_path_x_rate = np.sum(self.desired_path_x_rates) / self.desired_path_x_rates.size
      if True: #CVS_FRAME % 2 == 0:
        with open('/dev/shm/desired_path_x_rate.txt','w') as fp:
          fp.write('%0.2f' % (desired_path_x_rate))
      # with open('/tmp/debug_out_k','w') as fp:
      # #   #fp.write('{0}\n'.format(['%0.2f' % i for i in path_x]))
      #   lead_mark = "▲"
      #   if hasLead == False or distLead_near == False:
      #     lead_mark = "△"
      # #   #fp.write('{0}\n'.format(['%0.2f' % i for i in self.desired_path_x_rates]))
      # #   #fp.write('@@@%f,%f,%f' % (vk_ego,desired_path_x_by_speed,path_x[ModelConstants.IDX_N -1]))
      # #   #fp.write('***%.2f,[%.2f],%d' % (np.sum(self.desired_path_x_rates),desired_path_x_rate,self.desired_path_x_rates.size))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s%dm,[%d%%]%.2f' % (vk_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[ModelConstants.IDX_N -1] ,lead_mark , sm['radarState'].leadOne.dRel,desired_path_x_rate*100,a_ego))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s%dm,↓%.2f,%d' % (vk_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[ModelConstants.IDX_N -1] ,lead_mark , sm['radarState'].leadOne.dRel,red_signal_speed_down,red_signal_scan_span))
      #   fp.write('%02dk<%d>%s%s(%.1f)%s%dm,%d' % (vk_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[ModelConstants.IDX_N -1] ,lead_mark , sm['radarState'].leadOne.dRel,self.night_time))
      # #   #fp.write('%02dk<%d>%s%s(%.1f)%s(%.2f,%.2f)' % (vk_ego*3.6,red_signal_scan_flag,red_signals_mark , red_signal , path_x[ModelConstants.IDX_N -1] ,lead_mark ,sm['radarState'].leadOne.modelProb,sm['radarState'].leadTwo.modelProb))
      red_signal_scan_ct += 1 #音を鳴らした後の緩衝処理になっているだけで、信号検出のあと徐々に加算されるロジックではないようだ。

      self.night_time_refresh_ct += 1
      if (self.night_time_refresh_ct % 11 == 6 and red_signal == "●") or self.night_time_refresh_ct % 200 == 100:
        try:
          with open('/dev/shm/night_time_info.txt','r') as fp:
            night_time_info_str = fp.read()
            if night_time_info_str:
              self.night_time = int(night_time_info_str)
        except Exception as e:
          pass
      red_stop_immediately = False
      if long_speeddown_flag == False and not sm['selfdriveState'].experimentalMode: #公式ロングではelseへ強制遷移する追加条件
        if self.night_time >= 90: #昼,90以下だと夕方で信号がかなり見やすくなる。
          stop_threshold = np.interp(vk_ego*3.6 , [0,10,20,30,40,50,55,60] , [15,25,35,43,59,77,92,103]) #昼の方が認識があまくなるようだ。
        else: #夜
          stop_threshold = np.interp(vk_ego*3.6 , [0,10,20,30,40,50,55,60] , [10,19,28,39,53,75,85,99]) #まあまあ,60km/hでも止まれる！？
        if path_x[ModelConstants.IDX_N -1] < stop_threshold:
          red_stop_immediately = True #停止せよ。
      else:
        if True: #self.night_time >= 90: #昼,90以下だと夕方で信号がかなり見やすくなる。
          stop_threshold = np.interp(vk_ego*3.6 , [0,10,20,30,40,50,60] , [15,20,23,28,43,57,66]) #事前減速で40km/h以下になることを期待している。昼
        # else: #夜
        #   stop_threshold = np.interp(vk_ego*3.6 , [0,10,20,30,40,50] , [15,20,23,27,38,52]) #事前減速で40km/h以下になることを期待している。夜
        if path_x[ModelConstants.IDX_N -1] < stop_threshold or desired_path_x_rate < 0.11:
          red_stop_immediately = True #停止せよ。
        # stop_threshold_r = np.interp(vk_ego*3.6 , [0   ,10  ,20  ,25  ,30  ,40  ,50  ]
        #                                     , [0.25,0.30,0.33,0.35,0.38,0.41,0.43]) #さらに減速度の強さa_egoを加味。弱ければより小さくできる？
        # if desired_path_x_rate < stop_threshold_r: #0.4:
        #   red_stop_immediately = True #停止せよ。
      if sum_red_signal_path_xs < self.old_red_signal_path_xs and vk_ego > red_signal_v_ego and red_signals_mark == "■" and sm['selfdriveState'].enabled and sm['carState'].gasPressed == False and (OP_ENABLE_v_cruise_kph == 0 or OP_ENABLE_gas_speed > red_signal_v_ego) and red_stop_immediately == True:
        #赤信号検出でワンペダル発動
        if red_signal_scan_ct < 10000:
          red_signal_scan_ct = 10000
          #まずは音を鳴らす。
          try:
            if accel_engaged_str:
              if int(accel_engaged_str) >= 3: #ワンペダルモード
                  # fp.write('%d' % (3)) #デバッグ用にpo.wavを鳴らしてみる。
                lock_off = False
                if os.path.isfile('/dev/shm/lockon_disp_disable.txt'):
                  with open('/dev/shm/lockon_disp_disable.txt','r') as fp: #臨時でロックオンボタンに連動
                    lockon_disp_disable_str = fp.read()
                    if lockon_disp_disable_str:
                      lockon_disp_disable = int(lockon_disp_disable_str)
                      if lockon_disp_disable != 0:
                        lock_off = True #ロックオンOFFで停車コードOFF
                if lock_off == False:
                  with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
                    fp.write('%d' % (1)) #prompt.wav音を鳴らしてみる。
                  OP_ENABLE_v_cruise_kph = v_cruise_kph
                  OP_ENABLE_gas_speed = 1.0 / 3.6
                  #one_pedal_chenge_restrict_time = 10 , ここは意味的に要らないか。
                  red_signal_scan_flag_1 = 3 #赤信号停止状態
                  set_red_signal_scan_flag_3 = True #セットした瞬間
                  red_signal_scan_span = red_signal_scan_ct_2 #2〜3までのフレーム数
                  if self.red_signal_eP_iP_flag != 1:
                    self.red_signal_eP_iP_flag = 1
                    with open('/dev/shm/red_signal_eP_iP_set.txt','w') as fp:
                      fp.write('%d' % (1))
          except Exception as e:
            pass
      else:
        red_signal_scan_ct = 0 if red_signal_scan_ct < 1000 else red_signal_scan_ct - 1000
      self.old_red_signal_path_xs = sum_red_signal_path_xs

    lever_up_down = 0
    # if (hasLead == True and distLead_near == True) or vk_ego > 24/3.6: #ここではlimitspeed_setは判定できないand limitspeed_set == True:
    if hasLead == True:
      if before_v_cruise_kph_max_1 > 0 and before_v_cruise_kph_max_1 < 200:
        if v_cruise_kph < before_v_cruise_kph_max_1:
          lever_up_down = -1
        elif v_cruise_kph > before_v_cruise_kph_max_1:
          lever_up_down = 1

    if (hasLead == True and sm['radarState'].leadOne.dRel < 10) or sm['selfdriveState'].enabled == False or sm['carState'].gasPressed == True or vk_ego < 0.1/3.6:
      if set_red_signal_scan_flag_3 == False:
        if self.red_signal_eP_iP_flag != 0:
          self.red_signal_eP_iP_flag = 0
          with open('/dev/shm/red_signal_eP_iP_set.txt','w') as fp:
            fp.write('%d' % (0))

    if (hasLead == True and distLead_near == True) or sm['selfdriveState'].enabled == False or sm['carState'].gasPressed == True or vk_ego < 0.1/3.6:
      if set_red_signal_scan_flag_3 == False:
        red_signal_scan_flag_1 = 0

    if red_signal_scan_flag_1 != red_signal_scan_flag:
      red_signal_scan_flag = red_signal_scan_flag_1
      rssf = red_signal_scan_flag
      if red_signal_scan_flag <= 1:
        red_signal_scan_span = 0
      if accel_engaged_str:
        if int(accel_engaged_str) < 3: #ワンペダルモード以外
          rssf = 0
      with open('/dev/shm/red_signal_scan_flag.txt','w') as fp:
        fp.write('%d' % (rssf))

    if hasLead == False and one_pedal == True and vk_ego < 0.1/3.6: #速度ゼロでIPモード時にレバー下に入れたら
      if v_cruise_kph < before_v_cruise_kph_max_1 and before_v_cruise_kph_max_1 < 200: #200km/h以下の場合のみ。初回の誤設定を弾く。
        if OP_ENABLE_v_cruise_kph == 0:
          with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
            fp.write('%d' % (1)) #MAXを1に戻すのでprompt.wavを鳴らす。
        OP_ENABLE_v_cruise_kph = v_cruise_kph
        OP_ENABLE_gas_speed = 1.0 / 3.6
    if vk_ego > 3/3.6 and vk_ego < min_acc_speed/3.6:
      force_low_engage_set = False #MAX!=1でタッチすると低速(スピードが3〜31(min_acc_speed)km/h未満)でエンゲージ。
      try:
        with open('/dev/shm/force_low_engage.txt','r') as fp:
          force_low_engage_str = fp.read()
          if force_low_engage_str:
            if int(force_low_engage_str) == 1:
              OP_ENABLE_v_cruise_kph = v_cruise_kph
              OP_ENABLE_gas_speed = vk_ego
              force_low_engage_set = True
              if sm['carState'].gasPressed:
                OP_ENABLE_ACCEL_RELEASE = False #このあとのアクセルコントロールを許可する
      except Exception as e:
        pass
      if force_low_engage_set:
        with open('/dev/shm/force_low_engage.txt','w') as fp:
          fp.write('%d' % (0))
        with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (2)) #engage.wavを鳴らす。
    if vk_ego > 3/3.6 and vk_ego < min_acc_speed/3.6 and sm['carState'].gasPressed and sm['selfdriveState'].enabled: #oneペダル操作中にアクセル踏みながら31(min_acc_speed)km/h未満の走行時にレバーを上に入れたら、一旦車体速度にエクストラエンゲージし直す。
      if before_v_cruise_kph_max_1 <= (37 if tss_type < 2 else 57) and OP_ENABLE_gas_speed == 1.0 / 3.6 and v_cruise_kph > before_v_cruise_kph_max_1: # これを繰り返すとACC設定速度がどんどん上がっていく。ACC最低速度近辺(37程度)に限定
        OP_ENABLE_v_cruise_kph = v_cruise_kph
        OP_ENABLE_gas_speed = vk_ego
        OP_ENABLE_ACCEL_RELEASE = False #このあとのアクセルコントロールを許可する
        with open('/dev/shm/signal_start_prompt_info.txt','w') as fp:
          fp.write('%d' % (2)) #engage.wavを鳴らす。
    before_v_cruise_kph_max_1 = v_cruise_kph

    # if OP_ENABLE_v_cruise_kph != 0 and OP_ENABLE_v_cruise_kph != v_cruise_kph: #レバー操作したらエンゲージ初期クルーズ速度解除
    # 単にレバーアップダウンで通常クルーズに移行するのをやめる。基本は上なら通常クルーズ。下の時は現車速より上がってしまうなら何もしない。
    if OP_ENABLE_v_cruise_kph != 0 and ((OP_ENABLE_v_cruise_kph < v_cruise_kph and vk_ego*3.6 < v_cruise_kph) or (OP_ENABLE_v_cruise_kph > v_cruise_kph and vk_ego*3.6 > v_cruise_kph)):
      OP_ENABLE_v_cruise_kph = 0
      if red_signal_scan_flag == 3:
        red_signal_scan_flag = 2
        with open('/dev/shm/red_signal_scan_flag.txt','w') as fp:
          fp.write('%d' % (red_signal_scan_flag))

    # if OP_ENABLE_v_cruise_kph != 0 and (OP_ENABLE_v_cruise_kph > v_cruise_kph and OP_ENABLE_gas_speed  > 1.0/3.6):
    #   ワンペダルの時は抑制したい
    #   OP_ENABLE_v_cruise_kph = v_cruise_kph
    #   OP_ENABLE_gas_speed = 1.0 / 3.6

    if OP_ENABLE_v_cruise_kph != 0:
      v_cruise_kph = OP_ENABLE_gas_speed*3.6 #エンゲージ初期クルーズ速度を優先して使う,MAX=1もここで入ってくる。
    if CVS_FRAME % 5 == 4:
      try:
        with open('/dev/shm/handle_center_info.txt','r') as fp:
          handle_center_info_str = fp.read()
          if handle_center_info_str:
            handle_center = float(handle_center_info_str)
      except Exception as e:
        pass

    limitspeed_set = False
    try:
      with open('/dev/shm/limitspeed_data.txt','r') as fp2:
        limitspeed_data_str = fp2.read()
        if limitspeed_data_str:
          limitspeed_data = limitspeed_data_str.split(",")
          limitspeed_flag = int(limitspeed_data[2])
          #self.limitspeed_pointの計算は常に行う。
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
            if target < self.limitspeed_point:
              self.limitspeed_point = target
          elif target < self.limitspeed_point:
            self.limitspeed_point -= 0.1
            if target > self.limitspeed_point:
              self.limitspeed_point = target

          if limitspeed_flag != 999:
            self.limitspeed_point = vk_ego * 3.6

          self.limitspeed_point_dim.append(self.limitspeed_point)
          if len(self.limitspeed_point_dim) > 50:
            self.limitspeed_point_dim.pop(0)
          self.limitspeed_point_avg = sum(self.limitspeed_point_dim) / len(self.limitspeed_point_dim) #直近50個の平均。

          with open('/dev/shm/limitspeed_sw.txt','r') as fp:
            limitspeed_sw_str = fp.read()
            if limitspeed_sw_str and limitspeed_data_str:
              if int(limitspeed_sw_str) == 1 and OP_ENABLE_v_cruise_kph == 0 and sm['selfdriveState'].enabled: #自動設定モード
                if limitspeed_flag == 999:
                  v_cruise_kph = self.limitspeed_point_avg
                  # v_cruise_kph = self.limitspeed_point
                  limitspeed_set = True
    except Exception as e:
      self.limitspeed_point = vk_ego * 3.6
      pass

    if lever_up_down != 0 and limitspeed_set == True:
      with open('/dev/shm/accel_ctrl_disable.txt','w') as fp:
        fp.write('%d' % (0 if lever_up_down > 0 else 1))

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
        with open('/dev/shm/accel_ctrl_disable.txt','r') as fp:
          accel_lead_ctrl_disable_str = fp.read()
          if accel_lead_ctrl_disable_str:
            accel_lead_ctrl_disable = int(accel_lead_ctrl_disable_str)
            if accel_lead_ctrl_disable == 0:
              accel_lead_ctrl = True
            else:
              accel_lead_ctrl = False
      except Exception as e:
        accel_lead_ctrl = True

    if accel_lead_ctrl == True and hasLead == True and sm['radarState'].leadOne.modelProb > 0.5 and OP_ENABLE_v_cruise_kph == 0: #前走者がいる,信頼度が高い,MAX!=1の状態
      leadOne = sm['radarState'].leadOne
      d_rel = leadOne.dRel #前走者までの距離
      #a_rel = leadOne.aRel #前走者の加速？　離れていっている時はプラス,常にゼロ？UIで使ってるgetAEgoと違うようだ。
      v_abs0 = leadOne.vRel + vk_ego #前走者の速度。vRelは相対速度のもよう。

      self.lead_v_abs.append(v_abs0)
      if len(self.lead_v_abs) > 20:
        self.lead_v_abs.pop(0)
      v_abs = sum(self.lead_v_abs) / len(self.lead_v_abs) #直近20個の平均。ガタつきを抑える

      # with open('/tmp/debug_out_x','w') as fp:
      #   fp.write('%.0f[m],%.1f[k],%.2f[a]' % (leadOne.dRel , v_abs*3.6 , leadOne.aRel))
      if vk_ego * 3.6 * 0.6 < d_rel and v_cruise_kph < v_abs * 3.6 + 7: #例、時速50kmの時前走車までの距離が30m(50x0.6)以上離れている。&&MAX(v_cruise_kph)より相手+7が速い。
        self.v_cruise_kph_1_15 = v_abs * 3.6 + 7
        car_v_cruise_kph = sm['carState'].vCruise #車のACCレバー速度の内部値（メーター表示45なら41）
        if self.v_cruise_kph_1_15 > car_v_cruise_kph + 8:
          self.v_cruise_kph_1_15 = car_v_cruise_kph + 8 #MAXを最大8は超えない
        if self.v_cruise_kph_1_15 > 115: #念の為
          self.v_cruise_kph_1_15 = 115 #危ないのでひとまず時速115kmまで。
        if vk_ego * 3.6 >= v_cruise_kph * 0.90: #ACC設定速度がすでに出ている。
          add_v_by_lead = True #前走車に追いつくための増速処理が有効
          org_v_cruise_kph = v_cruise_kph
          if self.ac_vc_time < 1.0:
            self.ac_vc_time += 0.02
          self.ac_vc_time = np.clip(self.ac_vc_time,0.0,1.0)
          # v_cruise_kph *= 1.15 #ACC設定速度を1.5割増速
          v_cruise_kph = self.v_cruise_kph_1_15 * self.ac_vc_time + v_cruise_kph * (1-self.ac_vc_time)
          if v_cruise_kph > 115:
            v_cruise_kph = 115 #危ないのでひとまず時速115kmまで。
            if v_cruise_kph < org_v_cruise_kph:
              v_cruise_kph = org_v_cruise_kph #計算前の速度より遅くなったら、追従加速をやめる。
              self.ac_vc_time = 0
              add_v_by_lead = False
    if add_v_by_lead == False and self.v_cruise_kph_1_15 > 0:
      if self.ac_vc_time > 0:
        self.ac_vc_time -= 0.003 #解除はセット(0.02)の何倍も時間をかける
        test_v_cruise_kph = self.v_cruise_kph_1_15 * self.ac_vc_time + v_cruise_kph * (1-self.ac_vc_time)
        if vk_ego <= 1*3.6 or int(test_v_cruise_kph) <= int(v_cruise_kph):
          self.ac_vc_time -= 0.02 #停車時では早く終わらせる。数字が元の速度と同じ時も同様。
        if OP_ENABLE_v_cruise_kph != 0:
          self.ac_vc_time = 0 #ワンペダル操作では直に終わらせる。
      self.ac_vc_time = np.clip(self.ac_vc_time,0.0,1.0)
      if self.ac_vc_time <= 0:
        self.v_cruise_kph_1_15 = 0
        self.lead_v_abs = []
      v_cruise_kph = self.v_cruise_kph_1_15 * self.ac_vc_time + v_cruise_kph * (1-self.ac_vc_time)

    steerAng = sm['carState'].steeringAngleDeg - handle_center
    # orgSteerAng = steerAng , 使わなくても良くなった？
    limit_vc = V_CRUISE_MAX
    limit_vc_h = V_CRUISE_MAX
    #ml_csv = ""

    global decel_lead_ctrl
    if CVS_FRAME % 30 == 29:
      try:
        with open('/dev/shm/decel_ctrl_disable.txt','r') as fp:
          decel_lead_ctrl_disable_str = fp.read()
          if decel_lead_ctrl_disable_str:
            decel_lead_ctrl_disable = int(decel_lead_ctrl_disable_str)
            if decel_lead_ctrl_disable == 0:
              decel_lead_ctrl = True
            else:
              decel_lead_ctrl = False
      except Exception as e:
        decel_lead_ctrl = True

    steer_ang_predicate = False
    if decel_lead_ctrl == True and len(md.position.x) == ModelConstants.IDX_N and len(md.orientation.x) == ModelConstants.IDX_N:
      #path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      max_yp = 0
      for yp in md.position.y: #path_y
        max_yp = yp if abs(yp) > abs(max_yp) else max_yp
        if abs(steerAng) < abs(max_yp) / 2.5:
          steer_ang_predicate = True
          steerAng = (-max_yp / 2.5)
      limit_vc = V_CRUISE_MAX if abs(steerAng) <= LIMIT_VC_B else LIMIT_VC_A / (abs(steerAng) - LIMIT_VC_B) + LIMIT_VC_C
      limit_vc_h = V_CRUISE_MAX if abs(steerAng) <= LIMIT_VC_BH else LIMIT_VC_AH / (abs(steerAng) - LIMIT_VC_BH) + LIMIT_VC_CH
      #前方カーブ機械学習用ファイルデータ生成処理。ひとまず保留
      #if CVS_FRAME % 10 == 0 and vk_ego * 3.6 > 20: # over 20km/h
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
      with open('/dev/shm/limit_vc_info.txt','w') as fp:
        fp.write('%d' % (limit_vc))
    if CVS_FRAME % 5 == 1:
      with open('/dev/shm/car_vego.txt','w') as fp:
        fp.write('%.1f' % (vk_ego))
    if CVS_FRAME % 5 == 1:
      with open('/dev/shm/steer_ang_predicate.txt','w') as fp: #md.position.yによる前方カーブ予測が急な時にTrue
       fp.write('%d' % int(limit_vc < 145 and steer_ang_predicate == True))
    if CVS_FRAME % 5 == 0:
      with open('/dev/shm/cruise_info.txt','w') as fp:
        #fp.write('%d/%d' % (v_cruise_kph_org , (limit_vc if limit_vc < V_CRUISE_MAX else V_CRUISE_MAX)))
        if v_cruise_kph == limit_vc:
          if cruise_info_power_up:
            fp.write('%d;' % (v_cruise_kph))
          else:
            fp.write('%d.' % (v_cruise_kph))
        else:
          if add_v_by_lead == True or self.ac_vc_time > 0:
            if cruise_info_power_up:
              fp.write('%d;' % (v_cruise_kph_org))
            else:
              fp.write(',%d' % (v_cruise_kph_org))
          else:
            vo = v_cruise_kph_org
            if int(vo) == 59 or int(vo) == 61:
              vo += 0.5 #メーター表示誤差補正
            if tss_type >= 2 and (int(vo) == 29 or int(vo) == 30):
              vo += 0.5 #メーター表示誤差補正
            if cruise_info_power_up:
              fp.write('%d;' % (vo))
            elif limitspeed_set == True:
              #速度自動セットで、前走車がいないときは速度を5キロ刻みで安定させる
              if add_v_by_lead == False and (tss_type >= 2 or phv_2019 or vo < 115.0) and vo > min_acc_speed:
                vo = int(vo / 5) * 5
              fp.write(';%d' % (vo))
            else:
              fp.write('%d' % (vo))
    #if CVS_FRAME % 10 == 0 and limit_vc < V_CRUISE_MAX and vk_ego * 3.6 > 20: # over 20km/h
    #  with open('./ml_data.csv','a') as fp:
    #    fp.write('%s%0.2f\n' % (ml_csv , limit_vc))
    CVS_FRAME += 1
    global v_cruise , v_cruise_old
    v_cruise_old = v_cruise

    v_117 = 116
    if tss_type < 2 and phv_2019 == False and v_cruise_kph >= 105: # TSSPで105km/h以上の設定なら
      personality = sm['selfdriveState'].personality #aggressiveで+1, relaxedで-1
      if personality==log.LongitudinalPersonality.relaxed and v_cruise_kph > 1:
        v_cruise_kph -= 1
        v_117 -= 1
      elif personality==log.LongitudinalPersonality.aggressive:
        v_cruise_kph += 1
        v_117 += 1
      # v_cruise_kph += (1 - sm['selfdriveState'].personality) #これではダメだ。数値じゃない？

    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS # * red_signal_speed_down
    long_speeddown_flag = False
    if desired_path_x_rate > 0.1 and desired_path_x_rate < 1.0:
      long_speeddown_disable = 0
      try:
        with open('/dev/shm/long_speeddown_disable.txt','r') as fp:
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

    creep_a_mul = 1.0
    if OP_ENABLE_v_cruise_kph != 0 and v_cruise_kph <= 1.2: #km/h
      ePedal = False
      if accel_engaged_str and int(accel_engaged_str) == 4: #eペダルモード以外
        ePedal = True

      if ePedal == False or sm['carState'].cruiseState.standstill or self.red_signal_eP_iP_flag == 1:
        #クリープ中にここを通してはいけない。AI判断でやたら停止してしまう。self.red_signal_eP_iP_flag == 1なら一時的iPモード。
        v_cruise = 0 #ワンペダル停止処理,冬タイヤはこれで良い？
        self.v_cruise_onep_k = (np.interp(vk_ego*3.6,[0,5,10,20,40,60],[1.0,0.96,0.93,0.9,0.87,0.85]) #もう少し滑らかに
                                if tss_type < 2 else
                                np.interp(vk_ego*3.6,[0,5,10,20,40,60],[1.0,0.98,0.96,0.94,0.92,0.90])) #TSS2は早めに強く踏む
      else:
        t_v = 9/3.6  #m/s完全停止しない。クリープ速度。
        v_cruise = t_v
        if vk_ego < t_v and self.a_desired > 0: #クリープ発進を滑らかに。
          creep_a_mul = np.interp(vk_ego*3.6
                               ,[0  ,1  ,2  ,3  ,6  ,7  ,8  ,9  ]
                               ,[1.0,1.0,0.7,0.6,0.5,0.7,0.9,1.0])
        self.v_cruise_onep_k = 1.0
      #v_cruise = np.interp(vk_ego*3.6,[0,5,8,15,60],[0,0,3,5,20]) / 3.6 #速度が大きい時は1/3を目指す ->冬タイヤで停止距離が伸び伸びに。
      # self.v_cruise_onep_k = np.interp(vk_ego*3.6,[0,5,8,15,60],[1.0,0.75,0.666,0.333,0.333])
    else:
      self.v_cruise_onep_k = 1.0

      # if red_signal_scan_span > 0: これでブレーキングの強さが変わったら制御しづらいのでやめる。
      #   v_cruise *= np.interp(red_signal_scan_span , [0,25,100] , [1,1,1.5]) #2〜3のスパンが長いと、速度を落とすのに距離が伸びるように。
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    if limitspeed_set == True and self.ac_vc_time == 0 and cruise_info_power_up == False and self.v_desired_filter.x > self.limitspeed_point / 3.6: #増速した時を除く
      self.v_desired_filter.x = self.limitspeed_point / 3.6 #理想速度がACC自動セットより速くならないようにする
    if limitspeed_set == True and (add_v_by_lead == True or self.ac_vc_time > 0) and self.v_desired_filter.x > v_cruise_kph_org / 3.6:
      self.v_desired_filter.x = v_cruise_kph_org / 3.6 #理想速度が増速分より速くならないようにする
    if tss_type < 2 and phv_2019 == False and self.v_desired_filter.x > v_117 / 3.6:
      self.v_desired_filter.x = v_117 / 3.6
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.a_desired_mul = 1.0
    vl = 0
    vd = 0
    lcd = 0
    if hasLead == True and sm['radarState'].leadOne.modelProb > 0.5: #前走者がいる,信頼度が高い
      leadOne = sm['radarState'].leadOne
      to_lead_distance = 35 #35m以上空いている
      add_lead_distance = vk_ego * 3.6 #速度km/hを車間距離(m)と見做す
      add_lead_distance = 0 if add_lead_distance < 50 else add_lead_distance - 50
      to_lead_distance += add_lead_distance #時速50km/h以上ならto_lead_distanceをのばす。時速100km/hでは85mになる。
      if leadOne.dRel > to_lead_distance:
        lcd = leadOne.dRel #前走者までの距離
        lcd -= to_lead_distance #0〜
        lcd /= ((70 + add_lead_distance) -to_lead_distance) #70m離れていたら1.0(時速50km以下の時、時速100kmでは130mとなる)
        if lcd > 1:
          lcd = 1
    if (hasLead == False or lcd > 0) and self.a_desired > 0 and vk_ego >= 1/3.6 and sm['carState'].gasPressed == False: #前走者がいない。加速中
      if hasLead == False:
        lcd = 1.0 #前走車がいなければlcd=1扱い。
      vl = v_cruise
      if vl > 100/3.6:
        vl = 100/3.6
      #vl *= 0.60 #加速は目標速度の半分程度でおしまい。そうしないと増速しすぎる
      vl = np.interp(vl, START_DASH_SPEEDS, START_DASH_CUT) #定数倍ではなく、表で考えてみる。
      vd = vk_ego
      if vd > vl:
        vd = vl #vdの最大値はvl
      if vl > 0 and vd < vl:
        vd /= vl #0〜1
        vd = 1 - vd #1〜0
        vd = math.sqrt(vd) #sqrt(vd)
        add_k = np.interp(vk_ego,[0,10/3.6],[0.12,0.25]) #0.2固定だと雨の日ホイールスピンする
        self.a_desired_mul = 1 + add_k*vd*lcd #1.2〜1倍で、(最大100km/hかv_cruise)*0.60に達すると1になる。→新方法は折れ線グラフの表から決定。速度が大きくなると大体目標値-20くらいにしている。これから検証。
        try:
          with open('/dev/shm/start_accel_power_up_disp_enable.txt','r') as fp:
            start_accel_power_up_disp_enable_str = fp.read()
            if start_accel_power_up_disp_enable_str:
              start_accel_power_up_disp_enable = int(start_accel_power_up_disp_enable_str)
              if start_accel_power_up_disp_enable == 0:
                self.a_desired_mul = 1 #スタート加速増なし
        except Exception as e:
          self.a_desired_mul = 1 #ファイルがなくてもスタート加速増なし

    if self.a_desired_mul == 1.0 or vk_ego < 1/3.6:
      cruise_info_power_up = False
    else:
      cruise_info_power_up = True

    if OP_ENABLE_v_cruise_kph != 0 and v_cruise_kph <= 1.2: #km/h
      ePedal = False
      if accel_engaged_str and int(accel_engaged_str) == 4: #eペダルモード以外
        ePedal = True
      if sm['carState'].gasPressed == False and self.a_desired > 0 and ePedal == False:
        self.a_desired = 0 #アクセル離して加速ならゼロに。
      if self.a_desired < 0 and ePedal == False:
        #ワンペダル停止の減速を強めてみる。
        self.a_desired_mul = np.interp(vk_ego,[0.0,10/3.6,20/3.6,40/3.6],[1.0,1.02,1.06,1.17]) #30km/hあたりから減速が強くなり始める->低速でもある程度強くしてみる。

    self.a_desired_mul *= creep_a_mul #クリープダッシュを緩和してみる。
    if limitspeed_set == True and (add_v_by_lead == False) and (tss_type >= 2 or phv_2019 or v_cruise < 115.0 / 3.6) and v_cruise > min_acc_speed / 3.6:
      #速度自動セットで、前走車がいないときは速度を5キロ刻みで安定させる
      v_cruise = int(v_cruise * 3.6 / 5) * 5 / 3.6
    # v_cruise2 = v_cruise

    v_cruise = v_cruise if (v_cruise < v_117/3.6 or phv_2019 or tss_type >= 2) else v_117/3.6 #TSSPではACC118を超えないようにする。
    v_cruise_car_limit = sm['carState'].vCruise/3.6 #車のACCレバー速度
    v_cruise_car_limit += 9/3.6 if v_cruise_car_limit < 70/3.6 else 8/3.6 #これ以上増速すると車体が速度を引き戻してしまう。
    v_cruise = v_cruise if v_cruise < v_cruise_car_limit else v_cruise_car_limit
    self.v_desired_filter.x = self.v_desired_filter.x if self.v_desired_filter.x < v_cruise_car_limit else v_cruise_car_limit
    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    if tss_type == 2 and not (self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT.value):
      tss2_amul = 1.0
      if self.a_desired < 0:
        tss2_amul = np.interp(vk_ego,[0,10/3.6],[1.1,1.0]) #減速を強める
        if accel_engaged_str:
          if int(accel_engaged_str) >= 3 and v_cruise_kph <= 1.2: #ワンペダルモードで実際にMAX=1のとき
            a2 = 1.04
            if a2 > tss2_amul:
              tss2_amul = a2
      self.a_desired *= tss2_amul
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    #self.v_desired_trajectoryに119とa_desired_mulの制限をかませる。
    if tss_type < 2 and phv_2019 == False:
      v_desired_trajectory_min = np.minimum(v_cruise_car_limit, v_117/3.6) #全要素を119km/h以下にする->118 and v_cruise_car_limit以下
    else:
      v_desired_trajectory_min = v_cruise_car_limit #TSS2でもv_cruise_car_limit以下
    self.v_desired_trajectory = np.minimum(self.v_desired_trajectory * (self.v_cruise_onep_k * self.a_desired_mul), v_desired_trajectory_min)
    # if v_cruise2 > v_desired_trajectory_min: #加速禁止
    #   self.a_desired_trajectory = np.minimum(self.a_desired_trajectory, 0)

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if sm['selfdriveState'].experimentalMode:
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

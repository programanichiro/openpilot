#!/usr/bin/env python3
import os
import random
import math
import numpy as np
from common.numpy_fast import interp

import cereal.messaging as messaging
from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.swaglog import cloudlog

from selfdrive.car.toyota.values import TSS2_CAR
from selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE , STEERING_CENTER
from common.params import Params
PARAMS = Params()
CVS_FRAME = 0
handle_center = STEERING_CENTER
accel_lead_ctrl = True
decel_lead_ctrl = True

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
LIMIT_VC_A ,LIMIT_VC_B ,LIMIT_VC_C  = calc_limit_vc(8.7,11.6,27.0 , 91-4      ,62.5-4      ,47.5-4      )
#LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH = calc_limit_vc(8.7,13.0,25.0 , 112,93,81)
LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH = calc_limit_vc(9.5,13.0,25.0 , 115,96,85)

OP_ENABLE_ACCEL_RELEASE = False
OP_ENABLE_PREV = False
OP_ENABLE_v_cruise_kph = 0
OP_ENABLE_gas_speed = 0
OP_ACCEL_PUSH = False
on_onepedal_ct = -1
cruise_info_power_up = False

START_DASH_CUT    = [0, 17/3.6, 26/3.6, 36/3.6, 45/3.6, 55/3.6, 64/3.6, 74/3.6, 83/3.6,  93/3.6]
START_DASH_SPEEDS = [0, 31/3.6, 41/3.6, 51/3.6, 61/3.6, 70/3.6, 80/3.6, 90/3.6, 100/3.6, 110/3.6]

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2  # car smoothly decel at .2m/s^2 when user is distracted
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.2, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 15., 25., 40.]

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


class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()

    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    global CVS_FRAME , handle_center , OP_ENABLE_PREV , OP_ENABLE_v_cruise_kph , OP_ENABLE_gas_speed , OP_ENABLE_ACCEL_RELEASE , OP_ACCEL_PUSH , on_onepedal_ct , cruise_info_power_up
    #with open('./debug_out_v','w') as fp:
    #  fp.write("%d push:%d , gas:%.2f" % (CVS_FRAME,sm['carState'].gasPressed,sm['carState'].gas))
    min_acc_speed = 31
    v_cruise_kph = sm['controlsState'].vCruise
    if self.CP.carFingerprint not in TSS2_CAR:
      v_cruise_kph = (55 - (55 - (v_cruise_kph+4)) * 2 - 4) if v_cruise_kph < (55 - 4) else v_cruise_kph
      v_cruise_kph = (110 + ((v_cruise_kph+6) - 110) * 3 - 6) if v_cruise_kph > (110 - 6) else v_cruise_kph
      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('./tss_type_info.txt','w') as fp:
          fp.write('%d' % (1))
    else:
      min_acc_speed = 30
      if CVS_FRAME % 5 == 3 and CVS_FRAME < 30:
        with open('./tss_type_info.txt','w') as fp:
          fp.write('%d' % (2))
    if v_cruise_kph < min_acc_speed:
      v_cruise_kph = min_acc_speed #念のため

    hasLead = sm['radarState'].leadOne.status
    add_v_by_lead = False #前走車に追いつくための増速処理

    steerAng = sm['carState'].steeringAngleDeg - handle_center
    orgSteerAng = steerAng

    CVS_FRAME += 1

    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_state == LongCtrlState.off

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

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
      with open('./debug_out_v','w') as fp:
        #fp.write('[%i],vc:%.1f(%.1f) , v:%.2f , vd:%.2f[km/h] ; ah:%.2f bh:%.2f ch:%.2f' % (prev_accel_constraint , v_cruise_kph_org , limit_vc , v_ego * 3.6 , self.v_desired_filter.x* 3.6 , LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH) )
        #fp.write('[%i],vc:%.1f(%.1f) , v:%.2f , vd:%.2f[km/h] ; a:%.2f , ad:%.2f[m/ss]' % (prev_accel_constraint , v_cruise_kph , limit_vc , v_ego * 3.6 , self.v_desired_filter.x* 3.6 , a_ego , self.a_desired) )
        fp.write('ah:%.2f bh:%.2f ch:%.2f\n' % (LIMIT_VC_AH,LIMIT_VC_BH,LIMIT_VC_CH) )
        #fp.write('op:[%d] vk:%.2f gs:%.2fkm/h\n' % (OP_ENABLE_PREV,OP_ENABLE_v_cruise_kph,OP_ENABLE_gas_speed*3.6) )
        fp.write("%s\n%s\n%s" % (msc ,msl ,msv))

    v_desired_rand = 0 #低速の時わざと揺らしてみる。
    #if v_ego < 41 / 3.6 and v_ego > 0:
    #  v_desired_rand = random.random() * 1.0 / 3.6
    #  v_desired_rand *= v_ego / 41/3.6

    #低速急ハンドルで速度を落とす実験->このままでは速度落ちすぎ。v_desired_filter.xより少し落とす感じで、v_cruiseは変更せずv_desired_randに差分を渡して試してみたい。ひとまず保留。
    if False and abs(steerAng) > 10 and v_ego * 3.6 < 41 and self.v_desired_filter.x * 3.6 > 20:
      rate = abs(steerAng) - 10 # 10->30 >> 0->20
      rate /= 20 # 0->1
      rate += 1 # 1->2
      if rate > 1.2:
        rate = 1.2
      new_vd = self.v_desired_filter.x / rate #30度切って最高半分。
      if new_vd < v_cruise and new_vd < self.v_desired_filter.x:
        v_desired_rand = new_vd - self.v_desired_filter.x
        with open('./cruise_info.txt','w') as fp:
          fp.write('%d.' % (int(new_vd * 3.6)))
    #with open('./debug_out_vd','w') as fp:
    #  fp.write('vc:%.2f[km/h] , vd:%.2f[km/h] ; ang:%.2f[deg] ; v:%.2f[km/h]' % (v_cruise * 3.6 , self.v_desired_filter.x* 3.6,steerAng , v_ego * 3.6) )

    accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    #accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    accel_limits_turns = limit_accel_in_turns(v_ego, orgSteerAng, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])
    # clip limits, cannot init MPC outside of bounds
#    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
#    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

#    self.mpc.set_weights(prev_accel_constraint)
#    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
#    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
#    self.mpc.update(sm['carState'], sm['radarState'], v_cruise)

    a_desired_mul = 1.0

    #with open('./debug_out_v','w') as fp:
    #  fp.write("lead:%d(lcd:%.2f) a:%.2f , m:%.2f(%d) , vl:%dkm/h , vd:%.2f" % (hasLead,lcd,self.a_desired,a_desired_mul,cruise_info_power_up,vl*3.6,vd))
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired*a_desired_mul + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired*a_desired_mul - 0.05)
#    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
#    self.mpc.set_cur_state(self.v_desired_filter.x + v_desired_rand, self.a_desired*a_desired_mul)
#    self.mpc.update(sm['carState'], sm['radarState'], v_cruise, prev_accel_constraint=prev_accel_constraint)
    self.mpc.set_weights(prev_accel_constraint)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x + v_desired_rand, self.a_desired*a_desired_mul)
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise)

    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5
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

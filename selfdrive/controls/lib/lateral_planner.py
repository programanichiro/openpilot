import numpy as np
from common.params import Params
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from system.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

tss_type = 0
STEERING_CENTER_calibration = []
STEERING_CENTER_calibration_update_count = 0
params = Params()
try:
  with open('../../../handle_center_info.txt','r') as fp:
    handle_center_info_str = fp.read()
    if handle_center_info_str:
      STEERING_CENTER = float(handle_center_info_str)
      with open('/tmp/handle_center_info.txt','w') as fp: #読み出し用にtmpへ書き込み
        fp.write('%0.2f' % (STEERING_CENTER) )
except Exception as e:
  pass

TRAJECTORY_SIZE = 33
CAMERA_OFFSET = 0.04


PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.05
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 800.0


class LateralPlanner:
  def __init__(self, CP):
    self.use_lanelines = False
    self.LP = LanePlanner(False)
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    # clip speed , lateral planning is not possible at 0 speed
    self.v_ego = max(MIN_SPEED, sm['carState'].vEgo)
    measured_curvature = sm['controlsState'].curvature

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md) #ichiropilot
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)

    STEER_CTRL_Y = sm['carState'].steeringAngleDeg
    path_y = self.path_xyz[:,1]
    max_yp = 0
    for yp in path_y:
      max_yp = yp if abs(yp) > abs(max_yp) else max_yp
    STEERING_CENTER_calibration_max = 300 #3秒
    if abs(max_yp) / 2.5 < 0.1 and self.v_ego > 20/3.6 and abs(STEER_CTRL_Y) < 8:
      STEERING_CENTER_calibration.append(STEER_CTRL_Y)
      if len(STEERING_CENTER_calibration) > STEERING_CENTER_calibration_max:
        STEERING_CENTER_calibration.pop(0)
    if len(STEERING_CENTER_calibration) > 0:
      value_STEERING_CENTER_calibration = sum(STEERING_CENTER_calibration) / len(STEERING_CENTER_calibration)
    else:
      value_STEERING_CENTER_calibration = 0
    handle_center = 0 #STEERING_CENTER
    global STEERING_CENTER_calibration_update_count
    STEERING_CENTER_calibration_update_count += 1
    if len(STEERING_CENTER_calibration) >= STEERING_CENTER_calibration_max:
      handle_center = value_STEERING_CENTER_calibration #動的に求めたハンドルセンターを使う。
      if STEERING_CENTER_calibration_update_count % 100 == 0:
        with open('../../../handle_center_info.txt','w') as fp: #保存用に間引いて書き込み
          fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
      if STEERING_CENTER_calibration_update_count % 10 == 5:
        with open('/tmp/handle_center_info.txt','w') as fp: #読み出し用にtmpへ書き込み
          fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
    else:
      with open('../../../handle_calibct_info.txt','w') as fp:
        fp.write('%d' % ((len(STEERING_CENTER_calibration)+2) / (STEERING_CENTER_calibration_max / 100)) )
    #with open('/tmp/debug_out_y','w') as fp:
    #  path_y_sum = -sum(path_y)
    #  #fp.write('{0}\n'.format(['%0.2f' % i for i in self.path_xyz[:,1]]))
    #  fp.write('calibration:%0.2f/%d ; max:%0.2f ; sum:%0.2f ; avg:%0.2f' % (value_STEERING_CENTER_calibration,len(STEERING_CENTER_calibration),-max_yp , path_y_sum, path_y_sum / len(path_y)) )
    STEER_CTRL_Y -= handle_center #STEER_CTRL_Yにhandle_centerを込みにする。
    ypf = STEER_CTRL_Y
    if abs(STEER_CTRL_Y) < abs(max_yp) / 2.5:
      STEER_CTRL_Y = (-max_yp / 2.5)

    if False:
      ssa = ""
      ssao = ""
      ssas = ""
      if ypf > 0:
        for vml in range(int(min(ypf,30))):
          ssa+= "|"
      if ypf > 0 and int(min(STEER_CTRL_Y - ypf,30 - len(ssa))) > 0:
        for vml in range(int(min(STEER_CTRL_Y - ypf,30 - len(ssa)))):
          ssao+= "<"
      elif ypf < 0 and (STEER_CTRL_Y) > 0 and int(min((STEER_CTRL_Y),30 - len(ssa))) > 0:
        for vml in range(int(min((STEER_CTRL_Y),30 - len(ssa)))):
          ssao+= "<"
      if 30 - len(ssa) - len(ssao) > 0:
        for vml in range(int(30 - len(ssa) - len(ssao))):
          ssas+= " "
      mssa = ""
      mssao = ""
      mssas = ""
      if ypf < 0:
        for vml in range(int(min(-ypf,30))):
          mssa+= "|"
      if ypf < 0 and int(min(-(STEER_CTRL_Y - ypf),30 - len(mssa))) > 0:
        for vml in range(int(min(-(STEER_CTRL_Y - ypf),30 - len(mssa)))):
          mssao+= ">"
      elif ypf > 0 and (STEER_CTRL_Y) < 0 and int(min(-(STEER_CTRL_Y),30 - len(mssa))) > 0:
        for vml in range(int(min(-(STEER_CTRL_Y),30 - len(mssa)))):
          mssao+= ">"
      if 30 - len(mssa) - len(mssao) > 0:
        for vml in range(int(30 - len(mssa) - len(mssao))):
          mssas+= " "
      with open('/tmp/debug_out_1','w') as fp:
        #fp.write('strAng:%0.1f->%0.1f[deg] , speed:%0.1f[km/h]' % (ypf , STEER_CTRL_Y - ypf, self.v_ego * 3.6))
        #fp.write('steerAngY:%0.1f[deg] , speed:%0.1f[km/h]' % (STEER_CTRL_Y, self.v_ego * 3.6))
        #fp.write('steerAng:%0.1f[deg] , speed:%0.1f[km/h]' % (STEER_CTRL_Y + handle_center, self.v_ego * 3.6)) #ハンドルセンターなしの素のSTEER_CTRL_Yを表示
        fp.write('strAng:%5.1f(%+5.1f[deg])%s%s%s^%s%s%s' % (ypf , STEER_CTRL_Y - ypf, ssas,ssao,ssa,mssa,mssao,mssas))
      

    if sm['carState'].leftBlinker == True:
      STEER_CTRL_Y = 90
    if sm['carState'].rightBlinker == True:
      STEER_CTRL_Y = -90

    # Lane change logic
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]
    lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)

    # Turn off lanes during lane change
    # if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
    #   self.LP.lll_prob *= self.DH.lane_change_ll_prob
    #   self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    try:
      # with open('/tmp/debug_out_y','w') as fp:
      #   fp.write('prob:%0.2f , %0.2f' % (self.LP.lll_prob , self.LP.rll_prob))
      with open('/tmp/lane_sw_mode.txt','r') as fp:
        lane_sw_mode_str = fp.read()
        if lane_sw_mode_str:
          lane_sw_mode = int(lane_sw_mode_str)
          if lane_sw_mode == 0:
            if self.use_lanelines == True:
              # params.put_bool('EndToEndToggle',True)
              # self.use_lanelines = not params.get_bool('EndToEndToggle')
              self.use_lanelines = False
          elif lane_sw_mode == 1:
            if self.use_lanelines == False:
              # params.put_bool('EndToEndToggle',False)
              # self.use_lanelines = not params.get_bool('EndToEndToggle')
              self.use_lanelines = True
          else: #lane_sw_mode == 2
            if self.use_lanelines == True and ((self.LP.lll_prob < 0.45 and self.LP.rll_prob < 0.45)
              or (self.LP.lll_prob < 0.60 and self.LP.rll_prob < 0.05)
              or (self.LP.lll_prob < 0.05 and self.LP.rll_prob < 0.60)
              or abs(max_yp) / 2.5 > 7 #前方カーブが強くなったらレーンレス走行へ切り替え。
              ) or self.use_lanelines == False and ((self.LP.lll_prob < 0.55 and self.LP.rll_prob < 0.55)
              or (self.LP.lll_prob < 0.70 and self.LP.rll_prob < 0.15)
              or (self.LP.lll_prob < 0.15 and self.LP.rll_prob < 0.70)
              or abs(max_yp) / 2.5 > 5
              ): #切り替えのバタつき防止
              if self.use_lanelines == True:
                # params.put_bool('EndToEndToggle',True)
                # self.use_lanelines = not params.get_bool('EndToEndToggle')
                self.use_lanelines = False
            else:
              if self.use_lanelines == False:
                # params.put_bool('EndToEndToggle',False)
                # self.use_lanelines = not params.get_bool('EndToEndToggle')
                self.use_lanelines = True
    except Exception as e:
      pass

    global tss_type
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

    # if tss_type >= 2: #↔︎ボタン仕様変更でTSS2スペシャル対応終了。
    #   STEER_CTRL_Y = 0
    #   max_yp = 0

    if self.use_lanelines:
      #d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      d_path_xyz = self.LP.get_d_path(STEER_CTRL_Y , (-max_yp / 2.5) , ypf , self.v_ego, self.t_idxs, self.path_xyz)
      self.lat_mpc.set_weights(PATH_COST, 1.0, LATERAL_ACCEL_COST, LATERAL_JERK_COST,STEERING_RATE_COST) #そろそろ整合性が取れなくなりつつある・・・
    else:
      d_path_xyz = self.path_xyz
      #ToDO,calc_dcmを消える運命のlane_plannerから持ってくる。
      dcm = self.LP.calc_dcm(STEER_CTRL_Y, (-max_yp / 2.5) , ypf , self.v_ego,2.5,-1,-1) #2.5はレーンを消すダミー,-1,-1はカメラオフセット反映に必要
      d_path_xyz[:,1] -= dcm #CAMERA_OFFSETが反映されている。->実はcalc_dcmの中で無視している。無い方が走りが良い？
      self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                               LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                               STEERING_RATE_COST)

    y_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
    heading_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    yaw_rate_pts = np.interp(self.v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw_rate)
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * self.v_ego**2))
    p = np.array([self.v_ego, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    # init state for next iteration
    # mpc.u_sol is the desired second derivative of psi given x0 curv state.
    # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
    # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 20000. or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = False
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlan', plan_send)

import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.drive_helpers import CONTROL_N, MPC_COST_LAT, LAT_MPC_N, CAR_ROTATION_RADIUS
from selfdrive.controls.lib.lane_planner import LanePlanner, TRAJECTORY_SIZE , STEERING_CENTER
from selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

STEERING_CENTER_calibration = []
STEERING_CENTER_calibration_update_count = 0

class LateralPlanner:
  def __init__(self, CP, use_lanelines=True, wide_camera=False):
    self.use_lanelines = use_lanelines
    self.LP = LanePlanner(wide_camera)
    self.DH = DesireHelper()

    self.last_cloudlog_t = 0
    self.steer_rate_cost = CP.steerRateCost
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.path_xyz_stds = np.ones((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    measured_curvature = sm['controlsState'].curvature

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md)
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = list(md.orientation.z)
    if len(md.position.xStd) == TRAJECTORY_SIZE:
      self.path_xyz_stds = np.column_stack([md.position.xStd, md.position.yStd, md.position.zStd])

    STEER_CTRL_Y = sm['carState'].steeringAngleDeg
    path_y = self.path_xyz[:,1]
    max_yp = 0
    for yp in path_y:
      max_yp = yp if abs(yp) > abs(max_yp) else max_yp
    STEERING_CENTER_calibration_max = 300 #3秒
    if abs(max_yp) / 2.5 < 0.1 and v_ego > 20/3.6 and abs(STEER_CTRL_Y) < 8:
      STEERING_CENTER_calibration.append(STEER_CTRL_Y)
      if len(STEERING_CENTER_calibration) > STEERING_CENTER_calibration_max:
        STEERING_CENTER_calibration.pop(0)
    if len(STEERING_CENTER_calibration) > 0:
      value_STEERING_CENTER_calibration = sum(STEERING_CENTER_calibration) / len(STEERING_CENTER_calibration)
    else:
      value_STEERING_CENTER_calibration = 0
    handle_center = STEERING_CENTER
    global STEERING_CENTER_calibration_update_count
    STEERING_CENTER_calibration_update_count += 1
    if len(STEERING_CENTER_calibration) >= STEERING_CENTER_calibration_max:
      handle_center = value_STEERING_CENTER_calibration #動的に求めたハンドルセンターを使う。
      if STEERING_CENTER_calibration_update_count % 10 == 0:
        with open('./handle_center_info.txt','w') as fp:
          fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
    else:
      with open('./handle_calibct_info.txt','w') as fp:
        fp.write('%d' % ((len(STEERING_CENTER_calibration)+2) / (STEERING_CENTER_calibration_max / 100)) )
    #with open('./debug_out_y','w') as fp:
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
      with open('./debug_out_1','w') as fp:
        #fp.write('strAng:%0.1f->%0.1f[deg] , speed:%0.1f[km/h]' % (ypf , STEER_CTRL_Y - ypf, v_ego * 3.6))
        #fp.write('steerAngY:%0.1f[deg] , speed:%0.1f[km/h]' % (STEER_CTRL_Y, v_ego * 3.6))
        #fp.write('steerAng:%0.1f[deg] , speed:%0.1f[km/h]' % (STEER_CTRL_Y + handle_center, v_ego * 3.6)) #ハンドルセンターなしの素のSTEER_CTRL_Yを表示
        fp.write('strAng:%5.1f(%+5.1f[deg])%s%s%s^%s%s%s' % (ypf , STEER_CTRL_Y - ypf, ssas,ssao,ssa,mssa,mssao,mssas))
      

    if sm['carState'].leftBlinker == True:
      STEER_CTRL_Y = 90
    if sm['carState'].rightBlinker == True:
      STEER_CTRL_Y = -90

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['controlsState'].active, lane_change_prob)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    if self.use_lanelines:
      #d_path_xyz = self.LP.get_d_path(v_ego, self.t_idxs, self.path_xyz)
      d_path_xyz = self.LP.get_d_path(STEER_CTRL_Y , v_ego, self.t_idxs, self.path_xyz)
      self.lat_mpc.set_weights(MPC_COST_LAT.PATH, MPC_COST_LAT.HEADING, self.steer_rate_cost)
    else:
      d_path_xyz = self.path_xyz
      dcm = self.LP.calc_dcm(STEER_CTRL_Y, v_ego,2.5,-1,-1) #2.5はレーンを消すダミー,-1,-1はカメラオフセット反映に必要
      d_path_xyz[:,1] -= dcm #CAMERA_OFFSETが反映されている。->実はcalc_dcmの中で無視している。無い方が走りが良い？
      path_cost = np.clip(abs(self.path_xyz[0, 1] / self.path_xyz_stds[0, 1]), 0.5, 1.5) * MPC_COST_LAT.PATH
      # Heading cost is useful at low speed, otherwise end of plan can be off-heading
      heading_cost = interp(v_ego, [5.0, 10.0], [MPC_COST_LAT.HEADING, 0.0])
      self.lat_mpc.set_weights(path_cost, heading_cost, self.steer_rate_cost)

    y_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(d_path_xyz, axis=1), d_path_xyz[:, 1])
    heading_pts = np.interp(v_ego * self.t_idxs[:LAT_MPC_N + 1], np.linalg.norm(self.path_xyz, axis=1), self.plan_yaw)
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    # self.x0[4] = v_ego
    p = np.array([v_ego, CAR_ROTATION_RADIUS])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts)
    # init state for next
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature
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
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = self.lat_mpc.x_sol[0:CONTROL_N, 3].tolist()
    lateralPlan.curvatureRates = [float(x) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlan', plan_send)

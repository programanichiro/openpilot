import time
import numpy as np
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

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

class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.LP = LanePlanner(True) #widw_camera常にONで呼び出す。
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = MIN_SPEED
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.debug_mode = debug

  def update(self, sm):
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md,v_ego_car) #ichiropilot,lta_mode判定をこの中で行う。
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
      self.x_sol = np.column_stack([md.lateralPlannerSolution.x, md.lateralPlannerSolution.y, md.lateralPlannerSolution.yaw, md.lateralPlannerSolution.yawRate])
      #横制御に介入するのはおそらくこのlateralPlannerSolution.yへの修正？

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

    # Lane change logic
    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]
    lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)

    if self.LP.lta_mode and self.DH.lane_change_state == 0: #LTA有効なら。ただしレーンチェンジ中は発動しない。
      ypf = STEER_CTRL_Y
      STEER_CTRL_Y -= handle_center #STEER_CTRL_Yにhandle_centerを込みにする。
      pred_angle = (-max_yp / 2.5)
      lane_d = self.LP.get_d_path(STEER_CTRL_Y , pred_angle , ypf , self.v_ego, self.t_idxs, self.path_xyz) #self.path_xyzは戻り値から外した。
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.velocity.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
        k = np.interp(abs(pred_angle), [0, 7], [1.0, 2]) #旋回中は多めに戻す。
        # self.x_sol[:,1] -= lane_d * 0.15 #変更したパスを維持するため？
        self.x_sol[:,2] += lane_d * 0.015 * k #yaw（ハンドル制御の元値）をレーンの反対へ戻す
        # with open('/tmp/debug_out_v','w') as fp: #左レーンに近づくとlane_dがプラス、右レーン位近づくとlane_dがマイナス。
        #   fp.write("%+.3f/%+.3f , %+.3fAng/%+.3f" % (lane_d ,self.x_sol[CONTROL_N-1,2],pred_angle,k))
        # self.x_sol[:,0] *= 0 #レーン表示の縦が伸びない。しかし表示だけ
        # self.x_sol[:,1] *= 0 #パス表示が直進になる。しかし表示だけ
        # self.x_sol[:,2] *= 0 #yawを無効に。ハンドル制御できる確認。

  def publish(self, sm, pm):
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.path_xyz[:,1].tolist()
    lateralPlan.psis = self.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] # TODO: unused

    lateralPlan.mpcSolutionValid = bool(1)
    lateralPlan.solverExecutionTime = 0.0
    if self.debug_mode:
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.x_sol.tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = False
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    pm.send('lateralPlan', plan_send)

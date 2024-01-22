import os
import capnp
import numpy as np
from typing import Dict
from cereal import log
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_lag_adjusted_curvature, MIN_SPEED
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan, Meta
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
TRAJECTORY_SIZE = 33
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
LP = LanePlanner(True) #widw_camera常にONで呼び出す。

SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

ConfidenceClass = log.ModelDataV2.ConfidenceClass

class PublishState:
  def __init__(self):
    self.disengage_buffer = np.zeros(ModelConstants.CONFIDENCE_BUFFER_LEN*ModelConstants.DISENGAGE_WIDTH, dtype=np.float32)
    self.prev_brake_5ms2_probs = np.zeros(ModelConstants.FCW_5MS2_PROBS_WIDTH, dtype=np.float32)
    self.prev_brake_3ms2_probs = np.zeros(ModelConstants.FCW_3MS2_PROBS_WIDTH, dtype=np.float32)

def fill_xyzt(builder, t, x, y, z, x_std=None, y_std=None, z_std=None):
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.z = z.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if z_std is not None:
    builder.zStd = z_std.tolist()

def fill_xyvat(builder, t, x, y, v, a, x_std=None, y_std=None, v_std=None, a_std=None):
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.v = v.tolist()
  builder.a = a.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if v_std is not None:
    builder.vStd = v_std.tolist()
  if a_std is not None:
    builder.aStd = a_std.tolist()

def fill_model_msg(msg: capnp._DynamicStructBuilder, net_output_data: Dict[str, np.ndarray], publish_state: PublishState,
                   vipc_frame_id: int, vipc_frame_id_extra: int, frame_id: int, frame_drop: float,
                   timestamp_eof: int, timestamp_llk: int, model_execution_time: float,
                   nav_enabled: bool, v_ego: float, steer_delay: float, valid: bool , STEER_CTRL_Y: float , DH , CP) -> None:
  frame_age = frame_id - vipc_frame_id if frame_id > vipc_frame_id else 0
  msg.valid = valid

  modelV2 = msg.modelV2
  modelV2.frameId = vipc_frame_id
  modelV2.frameIdExtra = vipc_frame_id_extra
  modelV2.frameAge = frame_age
  modelV2.frameDropPerc = frame_drop * 100
  modelV2.timestampEof = timestamp_eof
  modelV2.locationMonoTime = timestamp_llk
  modelV2.modelExecutionTime = model_execution_time
  modelV2.navEnabled = nav_enabled

  # plan
  position = modelV2.position
  fill_xyzt(position, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.POSITION].T, *net_output_data['plan_stds'][0,:,Plan.POSITION].T)
  velocity = modelV2.velocity
  fill_xyzt(velocity, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.VELOCITY].T)
  acceleration = modelV2.acceleration
  fill_xyzt(acceleration, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ACCELERATION].T)
  orientation = modelV2.orientation
  fill_xyzt(orientation, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.T_FROM_CURRENT_EULER].T)
  orientation_rate = modelV2.orientationRate
  fill_xyzt(orientation_rate, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ORIENTATION_RATE].T)

  # lateral planning
  x, y, yaw, yawRate = [net_output_data['lat_planner_solution'][0,:,i].tolist() for i in range(4)]
  x_sol = np.column_stack([x, y, yaw, yawRate])
  v_ego = max(MIN_SPEED, v_ego)

  # if len(position.x) == TRAJECTORY_SIZE and len(orientation.x) == TRAJECTORY_SIZE: #ワンペダルならある程度ハンドルが正面を向いていること。
  #   LP.parse_model(modelV2,v_ego) #ichiropilot,lta_mode判定をこの中で行う。
  #   path_xyz = np.column_stack([position.x, position.y, position.z])
  #   t_idxs = np.array(position.t)
  
  #   path_y = path_xyz[:,1]
  #   max_yp = 0
  #   for yp in path_y:
  #     max_yp = yp if abs(yp) > abs(max_yp) else max_yp
  #   STEERING_CENTER_calibration_max = 300 #3秒
  #   if abs(max_yp) / 2.5 < 0.1 and v_ego > 20/3.6 and abs(STEER_CTRL_Y) < 8:
  #     STEERING_CENTER_calibration.append(STEER_CTRL_Y)
  #     if len(STEERING_CENTER_calibration) > STEERING_CENTER_calibration_max:
  #       STEERING_CENTER_calibration.pop(0)
  #   if len(STEERING_CENTER_calibration) > 0:
  #     value_STEERING_CENTER_calibration = sum(STEERING_CENTER_calibration) / len(STEERING_CENTER_calibration)
  #   else:
  #     value_STEERING_CENTER_calibration = 0
  #   handle_center = 0 #STEERING_CENTER
  #   global STEERING_CENTER_calibration_update_count
  #   STEERING_CENTER_calibration_update_count += 1
  #   if len(STEERING_CENTER_calibration) >= STEERING_CENTER_calibration_max:
  #     handle_center = value_STEERING_CENTER_calibration #動的に求めたハンドルセンターを使う。
  #     if STEERING_CENTER_calibration_update_count % 100 == 0:
  #       with open('../../../handle_center_info.txt','w') as fp: #保存用に間引いて書き込み
  #         fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
  #     if STEERING_CENTER_calibration_update_count % 10 == 5:
  #       with open('/tmp/handle_center_info.txt','w') as fp: #読み出し用にtmpへ書き込み
  #         fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
  #   else:
  #     with open('../../../handle_calibct_info.txt','w') as fp:
  #       fp.write('%d' % ((len(STEERING_CENTER_calibration)+2) / (STEERING_CENTER_calibration_max / 100)) )

  #   if LP.lta_mode and DH.lane_change_state == 0: #LTA有効なら。ただしレーンチェンジ中は発動しない。(DHは前回の情報になる)
  #     ypf = STEER_CTRL_Y
  #     STEER_CTRL_Y -= handle_center #STEER_CTRL_Yにhandle_centerを込みにする。
  #     pred_angle = (-max_yp / 2.5)
  #     lane_d = LP.get_d_path(STEER_CTRL_Y , pred_angle , ypf , v_ego, t_idxs, path_xyz) #self.path_xyzは戻り値から外した。
  #     if len(position.x) == TRAJECTORY_SIZE and len(velocity.x) == TRAJECTORY_SIZE:
  #       k = np.interp(abs(pred_angle), [0, 7], [1, 1]) #旋回中は多めに戻す。->やめる
  #       x_sol[:,2] += lane_d * 0.015 * k #yaw（ハンドル制御の元値）をレーンの反対へ戻す
        
  psis = x_sol[0:CONTROL_N, 2].tolist()
  curvatures = (x_sol[0:CONTROL_N, 3]/v_ego).tolist()

  action = modelV2.action
  action.desiredCurvature = get_lag_adjusted_curvature(steer_delay, v_ego, psis, curvatures, CP)

  # times at X_IDXS according to model plan
  PLAN_T_IDXS = [np.nan] * ModelConstants.IDX_N
  PLAN_T_IDXS[0] = 0.0
  plan_x = net_output_data['plan'][0,:,Plan.POSITION][:,0].tolist()
  for xidx in range(1, ModelConstants.IDX_N):
    tidx = 0
    # increment tidx until we find an element that's further away than the current xidx
    while tidx < ModelConstants.IDX_N - 1 and plan_x[tidx+1] < ModelConstants.X_IDXS[xidx]:
      tidx += 1
    if tidx == ModelConstants.IDX_N - 1:
      # if the Plan doesn't extend far enough, set plan_t to the max value (10s), then break
      PLAN_T_IDXS[xidx] = ModelConstants.T_IDXS[ModelConstants.IDX_N - 1]
      break
    # interpolate to find `t` for the current xidx
    current_x_val = plan_x[tidx]
    next_x_val = plan_x[tidx+1]
    p = (ModelConstants.X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val) if abs(next_x_val - current_x_val) > 1e-9 else float('nan')
    PLAN_T_IDXS[xidx] = p * ModelConstants.T_IDXS[tidx+1] + (1 - p) * ModelConstants.T_IDXS[tidx]

  # lane lines
  modelV2.init('laneLines', 4)
  for i in range(4):
    lane_line = modelV2.laneLines[i]
    fill_xyzt(lane_line, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['lane_lines'][0,i,:,0], net_output_data['lane_lines'][0,i,:,1])
  modelV2.laneLineStds = net_output_data['lane_lines_stds'][0,:,0,0].tolist()
  modelV2.laneLineProbs = net_output_data['lane_lines_prob'][0,1::2].tolist()

  # road edges
  modelV2.init('roadEdges', 2)
  for i in range(2):
    road_edge = modelV2.roadEdges[i]
    fill_xyzt(road_edge, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['road_edges'][0,i,:,0], net_output_data['road_edges'][0,i,:,1])
  modelV2.roadEdgeStds = net_output_data['road_edges_stds'][0,:,0,0].tolist()

  # leads
  modelV2.init('leadsV3', 3)
  for i in range(3):
    lead = modelV2.leadsV3[i]
    fill_xyvat(lead, ModelConstants.LEAD_T_IDXS, *net_output_data['lead'][0,i].T, *net_output_data['lead_stds'][0,i].T)
    lead.prob = net_output_data['lead_prob'][0,i].tolist()
    lead.probTime = ModelConstants.LEAD_T_OFFSETS[i]

  # meta
  meta = modelV2.meta
  meta.desireState = net_output_data['desire_state'][0].reshape(-1).tolist()
  meta.desirePrediction = net_output_data['desire_pred'][0].reshape(-1).tolist()
  meta.engagedProb = net_output_data['meta'][0,Meta.ENGAGED].item()
  meta.init('disengagePredictions')
  disengage_predictions = meta.disengagePredictions
  disengage_predictions.t = ModelConstants.META_T_IDXS
  disengage_predictions.brakeDisengageProbs = net_output_data['meta'][0,Meta.BRAKE_DISENGAGE].tolist()
  disengage_predictions.gasDisengageProbs = net_output_data['meta'][0,Meta.GAS_DISENGAGE].tolist()
  disengage_predictions.steerOverrideProbs = net_output_data['meta'][0,Meta.STEER_OVERRIDE].tolist()
  disengage_predictions.brake3MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_3].tolist()
  disengage_predictions.brake4MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_4].tolist()
  disengage_predictions.brake5MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_5].tolist()

  publish_state.prev_brake_5ms2_probs[:-1] = publish_state.prev_brake_5ms2_probs[1:]
  publish_state.prev_brake_5ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_5][0]
  publish_state.prev_brake_3ms2_probs[:-1] = publish_state.prev_brake_3ms2_probs[1:]
  publish_state.prev_brake_3ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_3][0]
  hard_brake_predicted = (publish_state.prev_brake_5ms2_probs > ModelConstants.FCW_THRESHOLDS_5MS2).all() and \
    (publish_state.prev_brake_3ms2_probs > ModelConstants.FCW_THRESHOLDS_3MS2).all()
  meta.hardBrakePredicted = hard_brake_predicted.item()

  # temporal pose
  temporal_pose = modelV2.temporalPose
  temporal_pose.trans = net_output_data['sim_pose'][0,:3].tolist()
  temporal_pose.transStd = net_output_data['sim_pose_stds'][0,:3].tolist()
  temporal_pose.rot = net_output_data['sim_pose'][0,3:].tolist()
  temporal_pose.rotStd = net_output_data['sim_pose_stds'][0,3:].tolist()

  # confidence
  if vipc_frame_id % (2*ModelConstants.MODEL_FREQ) == 0:
    # any disengage prob
    brake_disengage_probs = net_output_data['meta'][0,Meta.BRAKE_DISENGAGE]
    gas_disengage_probs = net_output_data['meta'][0,Meta.GAS_DISENGAGE]
    steer_override_probs = net_output_data['meta'][0,Meta.STEER_OVERRIDE]
    any_disengage_probs = 1-((1-brake_disengage_probs)*(1-gas_disengage_probs)*(1-steer_override_probs))
    # independent disengage prob for each 2s slice
    ind_disengage_probs = np.r_[any_disengage_probs[0], np.diff(any_disengage_probs) / (1 - any_disengage_probs[:-1])]
    # rolling buf for 2, 4, 6, 8, 10s
    publish_state.disengage_buffer[:-ModelConstants.DISENGAGE_WIDTH] = publish_state.disengage_buffer[ModelConstants.DISENGAGE_WIDTH:]
    publish_state.disengage_buffer[-ModelConstants.DISENGAGE_WIDTH:] = ind_disengage_probs

  score = 0.
  for i in range(ModelConstants.DISENGAGE_WIDTH):
    score += publish_state.disengage_buffer[i*ModelConstants.DISENGAGE_WIDTH+ModelConstants.DISENGAGE_WIDTH-1-i].item() / ModelConstants.DISENGAGE_WIDTH
  if score < ModelConstants.RYG_GREEN:
    modelV2.confidence = ConfidenceClass.green
  elif score < ModelConstants.RYG_YELLOW:
    modelV2.confidence = ConfidenceClass.yellow
  else:
    modelV2.confidence = ConfidenceClass.red

  # raw prediction if enabled
  if SEND_RAW_PRED:
    modelV2.rawPredictions = net_output_data['raw_pred'].tobytes()

def fill_pose_msg(msg: capnp._DynamicStructBuilder, net_output_data: Dict[str, np.ndarray],
                  vipc_frame_id: int, vipc_dropped_frames: int, timestamp_eof: int, live_calib_seen: bool) -> None:
  msg.valid = live_calib_seen & (vipc_dropped_frames < 1)
  cameraOdometry = msg.cameraOdometry

  cameraOdometry.frameId = vipc_frame_id
  cameraOdometry.timestampEof = timestamp_eof

  cameraOdometry.trans = net_output_data['pose'][0,:3].tolist()
  cameraOdometry.rot = net_output_data['pose'][0,3:].tolist()
  cameraOdometry.wideFromDeviceEuler = net_output_data['wide_from_device_euler'][0,:].tolist()
  cameraOdometry.roadTransformTrans = net_output_data['road_transform'][0,:3].tolist()
  cameraOdometry.transStd = net_output_data['pose_stds'][0,:3].tolist()
  cameraOdometry.rotStd = net_output_data['pose_stds'][0,3:].tolist()
  cameraOdometry.wideFromDeviceEulerStd = net_output_data['wide_from_device_euler_stds'][0,:].tolist()
  cameraOdometry.roadTransformTransStd = net_output_data['road_transform_stds'][0,:3].tolist()

import os
import capnp
import numpy as np
from cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan, Meta
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner

STEERING_CENTER_calibration = []
STEERING_CENTER_calibration_update_count = 0
DEVICE_OFFSET_update_count = 0
device_y_offset = 0
try:
  with open('/data/handle_center_info.txt','r') as fp:
    handle_center_info_str = fp.read()
    if handle_center_info_str:
      STEERING_CENTER = float(handle_center_info_str)
      with open('/dev/shm/handle_center_info.txt','w') as fp: #読み出し用にtmpへ書き込み
        fp.write('%0.2f' % (STEERING_CENTER) )
except Exception as e:
  pass
LP = LanePlanner(False) #widw_cameraが推論に使われていない模様。常にONではなくOFFとしてみる。2024/5/9
g_lane_d = -999
g_lane_d_dim = []

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

def fill_xyz_poly(builder, degree, x, y, z):
  xyz = np.stack([x, y, z], axis=1)
  coeffs = np.polynomial.polynomial.polyfit(ModelConstants.T_IDXS, xyz, deg=degree)
  builder.xCoefficients = coeffs[:, 0].tolist()
  builder.yCoefficients = coeffs[:, 1].tolist()
  builder.zCoefficients = coeffs[:, 2].tolist()

def fill_lane_line_meta(builder, lane_lines, lane_line_probs):
  builder.leftY = lane_lines[1].y[0]
  builder.leftProb = lane_line_probs[1]
  builder.rightY = lane_lines[2].y[0]
  builder.rightProb = lane_line_probs[2]

def fill_model_msg(base_msg: capnp._DynamicStructBuilder, extended_msg: capnp._DynamicStructBuilder,
                   net_output_data: dict[str, np.ndarray], action: log.ModelDataV2.Action,
                   publish_state: PublishState, vipc_frame_id: int, vipc_frame_id_extra: int,
                   frame_id: int, frame_drop: float, timestamp_eof: int, model_execution_time: float,
                   valid: bool , STEER_CTRL_Y: float, DH, v_ego: float) -> None:
  frame_age = frame_id - vipc_frame_id if frame_id > vipc_frame_id else 0
  frame_drop_perc = frame_drop * 100
  extended_msg.valid = valid
  base_msg.valid = valid

  driving_model_data = base_msg.drivingModelData

  driving_model_data.frameId = vipc_frame_id
  driving_model_data.frameIdExtra = vipc_frame_id_extra
  driving_model_data.frameDropPerc = frame_drop_perc
  driving_model_data.modelExecutionTime = model_execution_time

  driving_model_data.action = action

  modelV2 = extended_msg.modelV2
  modelV2.frameId = vipc_frame_id
  modelV2.frameIdExtra = vipc_frame_id_extra
  modelV2.frameAge = frame_age
  modelV2.frameDropPerc = frame_drop_perc
  modelV2.timestampEof = timestamp_eof
  modelV2.modelExecutionTime = model_execution_time

  # plan
  fill_xyzt(modelV2.position, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.POSITION].T, *net_output_data['plan_stds'][0,:,Plan.POSITION].T)
  fill_xyzt(modelV2.velocity, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.VELOCITY].T)
  fill_xyzt(modelV2.acceleration, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ACCELERATION].T)
  fill_xyzt(modelV2.orientation, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.T_FROM_CURRENT_EULER].T)
  fill_xyzt(modelV2.orientationRate, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ORIENTATION_RATE].T)

  # poly path (apply same lateral offset so controllers use shifted path)
  global DEVICE_OFFSET_update_count,device_y_offset
  if DEVICE_OFFSET_update_count % 50 == 0: #10回に1回、テキストから読み込んで反映する。頻度は多すぎるとファイルI/Oが増えるし、少なすぎると反映が遅れる。10回に1回くらいがちょうどいいかも。
    try:
      with open('/data/device_offset.txt','r') as fp:
        device_offset_str = fp.read() #中央から右にずらす距離をテキストで10みたいに書いておく。ファイルが無いか0でずらし無し。単位はcm。右がプラス。変更後はキャリブレーションリセットが必要みたい。
        if device_offset_str:
          device_y_offset = float(device_offset_str)
          device_y_offset /= 100.0 #cmからmへ変換
    except Exception as e:
      pass
  DEVICE_OFFSET_update_count += 1
  lead_x_offset = 1.0 #前走車の判定を1m近くに寄せる。衝突防止
  y_offset = device_y_offset #デバイスを右にdevice_y_offset cmずらす
  pos_x, pos_y, pos_z = net_output_data['plan'][0,:,Plan.POSITION].T
  pos_x = np.maximum(pos_x - lead_x_offset, 0.0) #Expモードで効果ある？
  fill_xyz_poly(driving_model_data.path, ModelConstants.POLY_PATH_DEGREE, pos_x, pos_y+y_offset, pos_z)

  # action
  modelV2.action = action

  # times at X_IDXS of edges and lines aren't used
  LINE_T_IDXS: list[float] = []

  # lane lines
  modelV2.init('laneLines', 4)
  for i in range(4):
    lane_line = modelV2.laneLines[i]
    fill_xyzt(lane_line, LINE_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['lane_lines'][0,i,:,0], net_output_data['lane_lines'][0,i,:,1])
  modelV2.laneLineStds = net_output_data['lane_lines_stds'][0,:,0,0].tolist()
  modelV2.laneLineProbs = net_output_data['lane_lines_prob'][0,1::2].tolist()

  if len(modelV2.position.x) == ModelConstants.IDX_N and len(modelV2.orientation.x) == ModelConstants.IDX_N: #ワンペダルならある程度ハンドルが正面を向いていること。
    LP.parse_model(modelV2,v_ego) #ichiropilot,lta_mode判定をこの中で行う。
    position = modelV2.position
    path_xyz = np.column_stack([position.x, position.y, position.z])

    path_y = path_xyz[:,1]
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
    #handle_center = 0 #STEERING_CENTER,もうhandle_center_info.txtもいらないか。
    global STEERING_CENTER_calibration_update_count,g_lane_d,g_lane_d_dim
    STEERING_CENTER_calibration_update_count += 1
    if len(STEERING_CENTER_calibration) >= STEERING_CENTER_calibration_max:
      #handle_center = value_STEERING_CENTER_calibration #動的に求めたハンドルセンターを使う。
      if STEERING_CENTER_calibration_update_count % 100 == 0:
        with open('/data/handle_center_info.txt','w') as fp: #保存用に間引いて書き込み
          fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
      if STEERING_CENTER_calibration_update_count % 10 == 5:
        with open('/dev/shm/handle_center_info.txt','w') as fp: #読み出し用にtmpへ書き込み
          fp.write('%0.2f' % (value_STEERING_CENTER_calibration) )
    else:
      with open('/data/handle_calibct_info.txt','w') as fp:
        fp.write('%d' % ((len(STEERING_CENTER_calibration)+2) / (STEERING_CENTER_calibration_max / 100)) )

    lane_d = 0
    if LP.lta_mode and DH.lane_change_state == 0: #LTA有効なら。ただしレーンチェンジ中は発動しない。(DHは前回の情報になる)
      pred_angle = (-max_yp / 2.5)
      lane_d0 = LP.get_d_path(pred_angle , v_ego, path_xyz) #self.path_xyzは戻り値から外した。
      g_lane_d_dim.append(lane_d0)
      if len(g_lane_d_dim) > 20: #20Hz->1秒の平均
        g_lane_d_dim.pop(0)
      lane_d = sum(g_lane_d_dim) / len(g_lane_d_dim)
    if lane_d != g_lane_d:
      g_lane_d = lane_d
      with open('/dev/shm/lane_d_info.txt','w') as fp:
        fp.write('%.5f' % (lane_d))

  fill_lane_line_meta(driving_model_data.laneLineMeta, modelV2.laneLines, modelV2.laneLineProbs)

  # road edges
  modelV2.init('roadEdges', 2)
  for i in range(2):
    road_edge = modelV2.roadEdges[i]
    fill_xyzt(road_edge, LINE_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['road_edges'][0,i,:,0], net_output_data['road_edges'][0,i,:,1])
  modelV2.roadEdgeStds = net_output_data['road_edges_stds'][0,:,0,0].tolist()

  # leads
  modelV2.init('leadsV3', 3)
  for i in range(3):
    lead = modelV2.leadsV3[i]
    x, y, v, a = net_output_data['lead'][0,i].T
    x = np.maximum(x - lead_x_offset, 0.0)
    fill_xyvat(lead, ModelConstants.LEAD_T_IDXS, x, y, v, a, *net_output_data['lead_stds'][0,i].T)
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
  disengage_predictions.gasPressProbs = net_output_data['meta'][0,Meta.GAS_PRESS].tolist()
  disengage_predictions.brakePressProbs = net_output_data['meta'][0,Meta.BRAKE_PRESS].tolist()

  publish_state.prev_brake_5ms2_probs[:-1] = publish_state.prev_brake_5ms2_probs[1:]
  publish_state.prev_brake_5ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_5][0]
  publish_state.prev_brake_3ms2_probs[:-1] = publish_state.prev_brake_3ms2_probs[1:]
  publish_state.prev_brake_3ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_3][0]
  hard_brake_predicted = (publish_state.prev_brake_5ms2_probs > ModelConstants.FCW_THRESHOLDS_5MS2).all() and \
    (publish_state.prev_brake_3ms2_probs > ModelConstants.FCW_THRESHOLDS_3MS2).all()
  meta.hardBrakePredicted = hard_brake_predicted.item()

  # confidence
  if vipc_frame_id % (2*ModelConstants.MODEL_RUN_FREQ) == 0:
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

def fill_pose_msg(msg: capnp._DynamicStructBuilder, net_output_data: dict[str, np.ndarray],
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

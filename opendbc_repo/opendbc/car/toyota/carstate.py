import copy

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, TSS2_CAR, RADAR_ACC_CAR, \
                                                  EPS_SCALE, UNSUPPORTED_DSU_CAR, SECOC_CAR

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.lkas_enabled = False
    self.prev_lkas_enabled = False
    self.prev_hazard = -1

    self.brake_state = False
    # self.params = Params()
    # self.flag_eps_TSS2 = True if CP.flags & ToyotaFlags.POWER_STEERING_TSS2.value else False
    self.before_ang = 0
    self.prob_ang = 0
    self.steeringAngleDegs = []
    self.knight_scanner_bit3_ct = 0

    self.lkas_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    if self.knight_scanner_bit3_ct == 0:
      try:
        with open('/dev/shm/knight_scanner_bit3.txt','r') as fp:
          knight_scanner_bit3_str = fp.read()
          if knight_scanner_bit3_str:
            self.knight_scanner_bit3  = int(knight_scanner_bit3_str)
      except Exception as e:
        # self.knight_scanner_bit3  = 7 #ここでデフォ設定はしない、値を継続させるため。
        # ⚫︎⚪︎⚪︎　空き,2024/7/31
        # ⚪︎⚫︎⚪︎　new_steer平滑化,2024/1/14
        # ⚪︎⚪︎⚫︎　ハンドル高精細化未来予想2024/1/19
        pass
    self.knight_scanner_bit3_ct = (self.knight_scanner_bit3_ct + 1) % 101
    cp_acc = cp_cam if (self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or bool(self.CP.flags & ToyotaFlags.DSU_BYPASS.value)) else cp

    # self.brake_force = cp.vl['BRAKE']['BRAKE_FORCE']

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["VSC1S07"]["GVC"]

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
      can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    else:
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0  # TODO: these also have GAS_PEDAL, come back and unify
      can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
      if not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)
      if self.CP.carFingerprint != CAR.TOYOTA_MIRAI:
        ret.engineRpm = cp.vl["ENGINE_RPM"]["RPM"]

      if self.CP.flags & ToyotaFlags.HYBRID:
        ret.gas = cp.vl["GAS_PEDAL_HYBRID"]["GAS_PEDAL"]
      else: #ichiropilot
        ret.gas = cp.vl["GAS_PEDAL"]["GAS_PEDAL"]

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      # self.accurate_steer_angle_seen = (not self.flag_eps_TSS2) if (self.knight_scanner_bit3 & 0x04) else True #True , 自分だけFalseにする, ただし knight_scanner_bit3.txt ⚪︎⚪︎⚫︎を切ると常にTrue
      self.accurate_steer_angle_seen = True #あれば常にグッドアングルセンサーを使う

    # steeringAngleDeg_ = ret.steeringAngleDeg
    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    self.steeringAngleDegOrg = ret.steeringAngleDeg #回転先予想する前のオリジナル値
    if (self.knight_scanner_bit3_ct & 0x3) == 1:
      with open('/dev/shm/steer_ang_info.txt','w') as fp:
       fp.write('%f' % (self.steeringAngleDegOrg))
    # if self.CP.carFingerprint not in TSS2_CAR:
    if (self.knight_scanner_bit3 & 0x04) and abs(self.steeringAngleDegOrg) < 35: # knight_scanner_bit3.txt ⚪︎⚪︎⚫︎をONで有効, 35度以上急カーブは補正止める
      steeringAngleDeg0 = ret.steeringAngleDeg
      self.steeringAngleDegs.append(float(steeringAngleDeg0))
      if len(self.steeringAngleDegs) > 13:
        self.steeringAngleDegs.pop(0)
        # 過去13フレーム(0.13秒)の角度から、角速度と角加速度の平均を求める。
        angVs = [self.steeringAngleDegs[i + 1] - self.steeringAngleDegs[i] for i in range(len(self.steeringAngleDegs) - 1)] #過去９回の角速度
        angAs = [angVs[i + 1] - angVs[i] for i in range(len(angVs) - 1)]
        angV = sum(angVs) / len(angVs)
        angA = sum(angAs) / len(angAs)
        self.prob_ang += angV
        prob_ct = 10 # 0.1秒先の未来を推定。
        prob_ang2 = prob_ct * angV + (prob_ct-1) * prob_ct / 2 * angA
        if self.before_ang != ret.steeringAngleDeg or self.accurate_steer_angle_seen:
          self.prob_ang = 0
        self.before_ang = ret.steeringAngleDeg
        # with open('/tmp/debug_out_v','w') as fp:
        #   fp.write("%+.2f(%+.2f),%+.2f/%+.2f" % (ret.steeringAngleDeg+self.prob_ang+prob_ang2,self.prob_ang+prob_ang2,angV,angA))
        ret.steeringAngleDeg += self.prob_ang + prob_ang2 #未来推定と現時点高精細処理を同時に行う。

    # with open('/tmp/debug_out_o','w') as fp:
    #   fp.write("%+.3f/%+.4f" % (steeringAngleDeg_ , ret.steeringAngleDeg))
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    hazard = cp.vl["BLINKERS_STATE"]["HAZARD_LIGHT"]
    if self.prev_hazard != hazard:
      self.prev_hazard = hazard
      with open('/tmp/hazard_light.txt','w') as fp:
        fp.write("%d" % (hazard))

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

      # Lane Tracing Assist control is unavailable (EPS_STATUS->LTA_STATE=0) until
      # the more accurate angle sensor signal is initialized
      ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    new_brake_state = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0)
    if self.brake_state != new_brake_state:
      self.brake_state = new_brake_state
      with open('/dev/shm/brake_light_state.txt','w') as fp:
        fp.write('%d' % (new_brake_state))

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.carFaultedNonCritical = cp.vl["PCM_CRUISE_SM"]["TEMP_ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
    if ret.cruiseState.speed != 0:
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if bool(self.CP.flags & ToyotaFlags.DSU_BYPASS.value) or (self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value) and not bool(self.CP.flags & ToyotaFlags.DSU_BYPASS.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      if self.CP.openpilotLongitudinalControl:
        ret.accFaulted = ret.accFaulted or cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    ret.cruiseState.standstill = pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = pcm_acc_status in (1, 2, 3, 4, 5, 6)
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]
    self.pcm_acc_status = pcm_acc_status #TSSPに必要

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])
      self.lkas_enabled = cp_cam.vl["LKAS_HUD"]["LKAS_STATUS"]
      if self.prev_lkas_enabled is None:
        self.prev_lkas_enabled = self.lkas_enabled
      steer_always = 0
      try:
        with open('/dev/shm/steer_always.txt','r') as fp:
          steer_always_str = fp.read()
          if steer_always_str:
            if int(steer_always_str) >= 1:
              steer_always = 2
      except Exception as e:
        pass
      # with open('/tmp/debug_out_v','w') as fp:
      #   fp.write("lkas_enabled:%d,%d,<%d,%d>" % (self.lkas_enabled,self.prev_lkas_enabled,steer_always,ret.cruiseState.available))
      with open('/dev/shm/cruise_available.txt','w') as fp:
        fp.write('%d' % (ret.cruiseState.available and ret.gearShifter != structs.CarState.GearShifter.reverse)) #念の為バック時にはfalse
      if not self.prev_lkas_enabled and self.lkas_enabled and steer_always == 0:# and ret.cruiseState.available:
        with open('/dev/shm/steer_always.txt','w') as fp:
         fp.write('%d' % 1)
        # with open('/data/steer_always.txt','w') as fp:
        #  fp.write('%d' % 1)
      elif (self.prev_lkas_enabled and not self.lkas_enabled and steer_always != 0):# or not ret.cruiseState.available:
        with open('/dev/shm/steer_always.txt','w') as fp:
         fp.write('%d' % 0)
        # with open('/data/steer_always.txt','w') as fp:
        #  fp.write('%d' % 0)
      self.prev_lkas_enabled = self.lkas_enabled

    # if self.pcm_follow_distance != cp.vl["PCM_CRUISE_2"]['PCM_FOLLOW_DISTANCE']:
    #   if self.pcm_follow_distance != 0 and cp.vl["PCM_CRUISE_2"]['PCM_FOLLOW_DISTANCE'] != 0:
    #     #ボタン切り替え
    #     lines = cp.vl["PCM_CRUISE_2"]['PCM_FOLLOW_DISTANCE']
    #     #button(1,2,3) -> LongitudinalPersonality(2,1,0) #大小逆になる
    #     self.params.put("LongitudinalPersonality", str(3-int(lines))) #公式距離ボタン対応で不要に。

    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"] #DISTANCE_LINESと逆1,2,3（遠い、中間、近い）
      # self.pcm_follow_distance = cp.vl["PCM_CRUISE_SM"]["DISTANCE_LINES"] #3,2,1

    buttonEvents = []
    if self.CP.carFingerprint in TSS2_CAR or (self.CP.flags & ToyotaFlags.SMART_DSU) or (self.CP.flags & ToyotaFlags.DSU_BYPASS):
      # lkas button is wired to the camera
      prev_lkas_button = self.lkas_button
      self.lkas_button = cp_cam.vl["LKAS_HUD"]["LDA_ON_MESSAGE"]

      # Cycles between 1 and 2 when pressing the button, then rests back at 0 after ~3s
      if self.lkas_button != 0 and self.lkas_button != prev_lkas_button:
        buttonEvents.extend(create_button_events(1, 0, {1: ButtonType.lkas}) +
                            create_button_events(0, 1, {1: ButtonType.lkas}))

      if self.CP.carFingerprint not in (RADAR_ACC_CAR | SECOC_CAR):
        # distance button is wired to the ACC module (camera or radar)
        prev_distance_button = self.distance_button
        if not self.CP.flags & ToyotaFlags.SMART_DSU:
          self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]
        else:
          self.distance_button = cp.vl["SDSU"]["FD_BUTTON"]


        buttonEvents += create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    ret.buttonEvents = buttonEvents
    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("BLINKERS_STATE", float('nan')),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }

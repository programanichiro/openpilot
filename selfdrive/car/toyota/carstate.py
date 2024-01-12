import copy
import os

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import Params
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR

SteerControlType = car.CarParams.SteerControlType

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
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.brake_state = False
    self.params = Params()
    self.flag_47700 = ('1131d250d405' in os.environ['DONGLE_ID'])
    self.before_ang = 0
    self.before_ang_ct = 0
    self.prob_ang = 0
    self.steeringAngleDegs = []
    self.curvature_hist = []

    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}

  def update(self, cp, cp_cam , actuators):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = ret.gas > 805
    else:
      # TODO: find a common gas pedal percentage signal
      ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0
      #ichiropilot
      msg = "GAS_PEDAL_HYBRID" if (self.CP.flags & ToyotaFlags.HYBRID) else "GAS_PEDAL"
      ret.gas = cp.vl[msg]["GAS_PEDAL"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = ret.vEgoRaw == 0

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    if self.flag_47700:
      #ハンドルは右に回すとマイナス、カーブは右に曲がるとプラス
      # self.curvature_hist.append(actuators.curvature)
      # if len(self.curvature_hist) > 5:
      #   self.curvature_hist.pop(0)
      #   cVs = [self.curvature_hist[i + 1] - self.curvature_hist[i] for i in range(len(self.curvature_hist) - 1)] #過去のカーブ増加率
      #   cV = sum(cVs) / len(cVs)
      #   dAng = cV / 0.00005 #0.00005で1度増す
      #   if dAng > 5:
      #     dAng = 5
      #   elif dAng < -5:
      #     dAng = -5
      #   ret.steeringAngleDeg -= dAng #先読みの角度を与えて、ハンドル回しを打ち消すという考えは合ってそうだ。
      #   with open('/tmp/debug_out_z','w') as fp:
      #     fp.write("%+.2f,d:%+.3f" % (ret.steeringAngleDeg,-dAng))

        # with open('/tmp/debug_out_z','w') as fp:
        #   fp.write("cv:%+.10f,(%+.10f)\n" % (self.curvature_hist[4] , sum(self.curvature_hist)/len(self.curvature_hist)))
        #   fp.write("%+.8f,%+.8f,%+.8f" % (self.curvature_hist[4]-self.curvature_hist[3],self.curvature_hist[3]-self.curvature_hist[2],self.curvature_hist[2]-self.curvature_hist[1]))

      # steeringAngleDeg0 = ret.steeringAngleDeg
      # self.steeringAngleDegs.append(float(steeringAngleDeg0))
      # angV = 0
      # #angA = 0
      # if len(self.steeringAngleDegs) > 17:
      #   self.steeringAngleDegs.pop(0)
      #   # 過去17フレーム(0.17秒)の角度から、角速度と角加速度の平均を求める。
      #   angVs = [self.steeringAngleDegs[i + 1] - self.steeringAngleDegs[i] for i in range(len(self.steeringAngleDegs) - 1)] #過去９回の角速度
      #   #angAs = [angVs[i + 1] - angVs[i] for i in range(len(angVs) - 1)] #過去８回の角加速度
      #   angV = sum(angVs) / len(angVs)
      #   #angA = sum(angAs) / len(angAs)
      #   self.prob_ang += angV

      # if self.before_ang != ret.steeringAngleDeg:
      #   self.before_ang_ct = 0
      #   self.prob_ang = 0
      # else:
      #   self.before_ang_ct += 1
      # # prob_ang = self.before_ang_ct * angV + (self.before_ang_ct-1) * self.before_ang_ct / 2 * angA
      # self.before_ang = ret.steeringAngleDeg
      # with open('/tmp/debug_out_v','w') as fp:
      #   fp.write("ct:%d,%+.2f,%+.2f,%+.2f" % (self.before_ang_ct,ret.steeringAngleDeg,ret.steeringAngleDeg+self.prob_ang,angV))
      # ret.steeringAngleDeg += self.prob_ang
      if abs(self.before_ang - ret.steeringAngleDeg) > 3.0/100: #1秒で3度以上
        # ハンドルが大きく動いたら
        self.before_ang_ct *= 0.9
        self.prob_ang *= 0.9
      else:
        if self.before_ang_ct < 100:
          self.before_ang_ct += 1
      self.before_ang = ret.steeringAngleDeg

      steeringAngleDeg0 = ret.steeringAngleDeg
      self.steeringAngleDegs.append(float(steeringAngleDeg0))
      if len(self.steeringAngleDegs) > 10:
        self.steeringAngleDegs.pop(0)
        #5〜ct〜55 -> 1〜10回の平均
        l = int(self.before_ang_ct) / 5
        l = 1 if l < 1 else (l if l < 10 else 10)
        sum_ang = 0
        # for i in range(l): #i=0..9
        #   sum_ang += self.steeringAngleDegs[9-i]
        ret.steeringAngleDeg = sum_ang / l
        with open('/tmp/debug_out_v','w') as fp:
          fp.write("ct:%d,%+.2f/%+.2f(%+.3f)" % (int(l),ret.steeringAngleDeg,steeringAngleDeg0,ret.steeringAngleDeg-steeringAngleDeg0))
      # ret.steeringAngleDeg += self.prob_ang
      pass
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = not self.flag_47700 #True , 自分だけFalseにする

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

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

    new_brake_state = bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0)
    if self.brake_state != new_brake_state:
      self.brake_state = new_brake_state
      with open('/tmp/brake_light_state.txt','w') as fp:
        fp.write('%d' % (new_brake_state))

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    if ret.cruiseState.speed != 0:
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp

    if self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])
      self.lead_dist_button = cp_cam.vl["ACC_CONTROL"]["DISTANCE"]

    if self.CP.flags & ToyotaFlags.SMART_DSU.value:
      self.lead_dist_button = cp.vl["SDSU"]["FD_BUTTON"]

    if self.lead_dist_lines_init == False or self.lead_dist_lines != cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES']:
      self.lead_dist_lines_init = True #初回は通す。
      if self.lead_dist_lines != 0 and cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES'] != 0:
        #ボタン切り替えの可能性が高い
        self.lead_dist_lines = cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES']
        #button(3,2,1) -> LongitudinalPersonality(2,1,0)
        self.params.put("LongitudinalPersonality", str(int(self.lead_dist_lines)-1))
      else:
        # Ready OFFなどはこちら？
        self.lead_dist_lines = cp.vl["PCM_CRUISE_SM"]['DISTANCE_LINES']
      # with open('/tmp/debug_out_q','w') as fp:
      #   fp.write('lead_dist_lines:%d' % (self.lead_dist_lines))

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # ignore standstill state in certain vehicles, since pcm allows to restart with just an acceleration request
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in (1, 2, 3, 4, 5, 6)
    self.pcm_neutral_force = cp.vl["PCM_CRUISE"]["NEUTRAL_FORCE"]

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if not self.CP.enableDsu and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("GEAR_PACKET", 1),
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    #ichiropilot
    if CP.flags & ToyotaFlags.HYBRID:
      messages.append(("GAS_PEDAL_HYBRID", 33))
    else:
      messages.append(("GAS_PEDAL", 33))

    if CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      messages.append(("DSU_CRUISE", 5))
      messages.append(("PCM_CRUISE_ALT", 1))
    else:
      messages.append(("PCM_CRUISE_2", 33))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      messages.append(("GAS_SENSOR", 50))

    if CP.enableBsm:
      messages.append(("BSM", 1))

    if CP.carFingerprint in RADAR_ACC_CAR and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not CP.flags & ToyotaFlags.SMART_DSU.value:
        messages += [
          ("ACC_CONTROL", 33),
        ]
      messages += [
        ("PCS_HUD", 1),
      ]

    if CP.carFingerprint not in (TSS2_CAR - RADAR_ACC_CAR) and not CP.enableDsu and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      messages += [
        ("PRE_COLLISION", 33),
      ]

    if CP.flags & ToyotaFlags.SMART_DSU.value:
      messages.append(("SDSU", 33))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []

    if CP.carFingerprint != CAR.PRIUS_V:
      messages += [
        ("LKAS_HUD", 1),
      ]

    if CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      messages += [
        ("PRE_COLLISION", 33),
        ("ACC_CONTROL", 33),
        ("PCS_HUD", 1),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

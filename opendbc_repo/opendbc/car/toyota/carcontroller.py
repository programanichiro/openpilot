import os
import copy
import math
from opendbc.car import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, make_tester_present_msg, rate_limit, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker

SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

ACCELERATION_DUE_TO_GRAVITY = 9.81

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut
# MAX_STEER_RATE = 105  # これを現車で耐えられる可能な限り上げる
# MAX_STEER_RATE_FRAMES = 25 # こちらも耐えられる可能な限り上げる？

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes

# PCM compensatory force calculation threshold interpolation values
COMPENSATORY_CALCULATION_THRESHOLD_V = [-0.2, -0.2, -0.05]  # m/s^2
COMPENSATORY_CALCULATION_THRESHOLD_BP = [0., 20., 32.]  # m/s

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.params = CarControllerParams(self.CP)
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0
    self.prohibit_neg_calculation = True
    self.distance_button = 0

    self.pcm_accel_compensation = 0.0

    self.packer = CANPacker(dbc_name)
    self.accel = 0

    self.now_gear = structs.CarState.GearShifter.park
    self.lock_flag = False
    self.lock_speed = 0
    try:
      with open('../../../run_auto_lock.txt','r') as fp:
        lock_speed_str = fp.read() #ロックするスピードをテキストで30みたいに書いておく。ファイルが無いか0でオートロック無し。
        if lock_speed_str:
          self.lock_speed = int(lock_speed_str)
    except Exception as e:
      pass
    self.before_ang = 0
    self.before_ang_ct = 0
    self.new_steers = []

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # *** control msgs ***
    can_sends = []

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    if (self.CP.carFingerprint not in TSS2_CAR) and (CS.knight_scanner_bit3 & 0x02): # knight_scanner_bit3.txt ⚪︎⚫︎⚪︎
      if abs(self.before_ang - CS.out.steeringAngleDeg) > 3.0/100: #1秒で3度以上
        # ハンドルが大きく動いたら
        self.before_ang_ct *= 0.9
      else:
        if self.before_ang_ct < 100:
          self.before_ang_ct += 1
      self.before_ang = CS.out.steeringAngleDeg

      new_steer0 = new_steer
      self.new_steers.append(float(new_steer0))
      if len(self.new_steers) > 10:
        self.new_steers.pop(0)
        #5〜ct〜55 -> 1〜10回の平均
        l = int(self.before_ang_ct) / 5
        l = int(1 if l < 1 else (l if l < 10 else 10))
        sum_steer = 0
        for i in range(l): #i=0..9
          sum_steer += self.new_steers[9-i]
        new_steer = sum_steer / l
        # with open('/tmp/debug_out_v','w') as fp:
        #   fp.write("ct:%d,%+.2f/%+.2f(%+.3f)" % (int(l),new_steer,new_steer0,new_steer-new_steer0))
    try:
      with open('/tmp/lane_d_info.txt','r') as fp:
        lane_d_info_str = fp.read()
        if lane_d_info_str:
          lane_d_info = float(lane_d_info_str)
          # nn_lane_d_info = 0 if new_steer == 0 else lane_d_info / new_steer
          # with open('/tmp/debug_out_v','w') as fp:
          #   fp.write('ns:%.7f/%.5f(%.1f%%)' % (new_steer,lane_d_info,nn_lane_d_info*100))
          #new_steerはマイナスで右に曲がる。
          new_steer -= lane_d_info * 100 * 30 #引くとセンターへ車体を戻す。
    except Exception as e:
      pass
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req))

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

    # *** gas and brake ***
    comp_thresh = interp(CS.out.vEgo, COMPENSATORY_CALCULATION_THRESHOLD_BP, COMPENSATORY_CALCULATION_THRESHOLD_V)
    # prohibit negative compensatory calculations when first activating long after accelerator depression or engagement
    if not CC.longActive:
      self.prohibit_neg_calculation = True
    # don't reset until a reasonable compensatory value is reached
    if CS.pcm_neutral_force > comp_thresh * self.CP.mass:
      self.prohibit_neg_calculation = False
    # limit minimum to only positive until first positive is reached after engagement, don't calculate when long isn't active
    if CC.longActive and not self.prohibit_neg_calculation:
      accel_offset = CS.pcm_neutral_force / self.CP.mass
    else:
      accel_offset = 0.
    # only calculate pcm_accel_cmd when long is active to prevent disengagement from accelerator depression
    if CC.longActive:
      pcm_accel_cmd = clip(actuators.accel + accel_offset, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      pcm_accel_cmd = clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
    else:
      pcm_accel_cmd = 0.

    # 公式のpcm_accel_compensation制御。上のcydia制御と被る。試してみるか要検討。上のgas and brakeと入れ替えるだけで試せそう
    # # *** gas and brake ***
    # # For cars where we allow a higher max acceleration of 2.0 m/s^2, compensate for PCM request overshoot
    # # TODO: validate PCM_CRUISE->ACCEL_NET for braking requests and compensate for imprecise braking as well
    # if self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and CC.longActive:
    #   # calculate amount of acceleration PCM should apply to reach target, given pitch
    #   accel_due_to_pitch = math.sin(CS.slope_angle) * ACCELERATION_DUE_TO_GRAVITY
    #   net_acceleration_request = actuators.accel + accel_due_to_pitch

    #   pcm_accel_compensation = 2.0 * (CS.pcm_accel_net - net_acceleration_request) if net_acceleration_request > 0 else 0.0

    #   # prevent compensation windup
    #   if actuators.accel - pcm_accel_compensation > self.params.ACCEL_MAX:
    #     pcm_accel_compensation = actuators.accel - self.params.ACCEL_MAX

    #   self.pcm_accel_compensation = rate_limit(pcm_accel_compensation, self.pcm_accel_compensation, -0.01, 0.01)
    #   pcm_accel_cmd = actuators.accel - self.pcm_accel_compensation
    # else:
    #   self.pcm_accel_compensation = 0.0
    #   pcm_accel_cmd = actuators.accel

    # pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

    actuators_accel = actuators.accel
    try:
      with open('/tmp/cruise_info.txt','r') as fp:
        cruise_info_str = fp.read()
        if cruise_info_str:
          if cruise_info_str == "1" or cruise_info_str == ",1": #クリープしたければ以下を通さない。
            with open('/tmp/accel_engaged.txt','r') as fp:
              accel_engaged_str = fp.read()
              eP_iP = False
              if int(accel_engaged_str) == 4 and os.path.exists('/tmp/red_signal_eP_iP_set.txt'):
                with open('/tmp/red_signal_eP_iP_set.txt','r') as fp:
                  red_signal_eP_iP_set_str = fp.read()
                  if red_signal_eP_iP_set_str and int(red_signal_eP_iP_set_str) == 1:
                    eP_iP = True
              if int(accel_engaged_str) == 3 or eP_iP == True: #ワンペダルモードでもeP(一時的な赤信号手前を除く)では通さない
                if pcm_accel_cmd > 0:
                  pcm_accel_cmd = 0
                  actuators_accel = 0
    except Exception as e:
      pass

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.CP.openpilotLongitudinalControl and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    if self.lock_speed > 0: #auto door lock , unlock
      gear = CS.out.gearShifter
      if self.now_gear != gear or (CS.out.doorOpen and self.lock_flag == True): #ギアが変わるか、ドアが開くか。
        if gear == structs.CarState.GearShifter.park and CS.out.doorOpen == False: #ロックしたまま開ける時の感触がいまいちなので、パーキングでアンロックする。
          can_sends.append(CanData(0x750, b'\x40\x05\x30\x11\x00\x40\x00\x00', 0)) #auto unlock
        self.lock_flag = False #ドアが空いてもフラグはおろす。
      elif gear == structs.CarState.GearShifter.drive and self.lock_flag == False and CS.out.vEgo >= self.lock_speed/3.6: #時速30km/h以上でオートロック
        can_sends.append(CanData(0x750, b'\x40\x05\x30\x11\x00\x80\x00\x00', 0)) #auto lock
        self.lock_flag = True
      self.now_gear = gear

    # we can spam can to cancel the system even if we are using lat only control
    if (self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

      # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
      if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
        desired_distance = 4 - hud_control.leadDistanceBars
        if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
          self.distance_button = not self.distance_button
        else:
          self.distance_button = 0

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        # can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, fcw_alert,
        #                                                 self.distance_button))
        can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, actuators_accel, CS.out.aEgo, CC.longActive, pcm_cancel_cmd, self.standstill_req,
                                                        lead, CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd
      else:
        # can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, False, self.distance_button))
        can_sends.append(toyotacan.create_accel_command(self.packer, 0, 0, 0, False, pcm_cancel_cmd, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = copy.copy(actuators)
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends

from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # 2026+ Model Y (Juniper) runs a second gen HW4 stack: DAS_status is at 0x399 instead of 0x39b,
    # UI_warning (blinkers, buckle switch & doors) isn't sent at all, and DAS_settings is on the
    # party bus (it's only sent sporadically on the autopilot party bus, too rarely to parse)
    if 0x399 in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.HW4_GEN2.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.HW4_GEN2.value

      # VCFRONT_lighting (blinkers) and SeatBeltStatus are only on the VEHICLE bus
      if 0x3F5 in fingerprint[CANBUS.vehicle]:
        ret.flags |= TeslaFlags.HW4_GEN2_VEHICLE_BUS.value

    # Model X and HW 2.5 vehicles are missing DAS_settings
    elif 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    # On HW4 gen2 the VEHICLE bus is tapped into bus 1 and carries an unrelated RADAR_START_ADDR
    ret.radarUnavailable = (bool(ret.flags & TeslaFlags.HW4_GEN2) or RADAR_START_ADDR not in fingerprint[1]
                            or Bus.radar not in DBC[candidate])

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    # Tesla expanded DAS_steeringControl->DAS_steeringControlType to 3 bits: first in the FSD 14 builds for HW4 around 10-26-2025,
    # then in the other HW4 builds around 03-02-2026, and for HW3 and HW2.5 with 2026.8.6 around 04-03-2026.
    # The values were pushed to the right by 1 bit and kept the same definitions, plus 4 = FSD was added.
    # These messages are only sent by the 3-bit firmware:
    # - HW3/HW4: DAS_redundantBrakingControl (0x489)
    # - HW2.5/HW3/HW4: DI_autonomyHealth (0x054)
    das_steering_3_bit = 0x489 in fingerprint[CANBUS.autopilot_party] or 0x054 in fingerprint[CANBUS.party]

    ret.dashcamOnly = (candidate in (CAR.TESLA_MODEL_X,) or  # dashcam only, pending find invalidLkasSetting signal
                       (not das_steering_3_bit and not docs))  # the car's software needs to be updated

    return ret

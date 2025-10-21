from cereal import log
from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.list_view import ButtonAction, ListItem
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

# Description constants
DESCRIPTIONS = {
  "OpenpilotEnabledToggle": (
    "Use the openpilot system for adaptive cruise control and lane keep driver assistance. " +
    "Your attention is required at all times to use this feature."
  ),
  "DisengageOnAccelerator": "When enabled, pressing the accelerator pedal will disengage openpilot.",
  "LongitudinalPersonality": (
    "Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. " +
    "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with " +
    "your steering wheel distance button."
  ),
  "IsLdwEnabled": (
    "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line " +
    "without a turn signal activated while driving over 31 mph (50 km/h)."
  ),
  "AlwaysOnDM": "Enable driver monitoring even when openpilot is not engaged.",
  'RecordFront': "Upload data from the driver facing camera and help improve the driver monitoring algorithm.",
  "IsMetric": "Display speed in km/h instead of mph.",
  "RecordAudio": "Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect.",
  "RaylibMode": "Switching to a user interface built with raylib.",
  "AccelMethodSwitch": "Switch Accel Method to Official version or recommendation. A reboot is required.",
  "GpsAlwaysSwitch": "GPS reception starts even when the car is not moving. This speeds up satellite acquisition and prevents GPS reception from being interrupted during temporary Offroad situations. However, it may affect battery consumption when the car is stationary.",
  "DisableMaxSpeedModify": "ACC speeds exceeding 115 km/h will be obtained directly from the vehicle. TSSP 2019 PHV users should enable it.",
  "ForceHybridVehicle": "Turn this switch on if a hybrid vehicle is incorrectly recognized as a gas vehicle. Do not turn it on for gas vehicles, as this will cause a crash.",
  "IgnoreRerouteHarness": "Fix a CAN error on a vehicle that does not have a DSU bypass harness or smartDSU installed.",
}


class TogglesLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      "OpenpilotEnabledToggle": (
        "Enable openpilot",
        DESCRIPTIONS["OpenpilotEnabledToggle"],
        "chffr_wheel.png",
        True,
      ),
      "ExperimentalMode": (
        "Experimental Mode",
        "",
        "experimental_white.png",
        False,
      ),
      "DisengageOnAccelerator": (
        "Disengage on Accelerator Pedal",
        DESCRIPTIONS["DisengageOnAccelerator"],
        "disengage_on_accelerator.png",
        False,
      ),
      "IsLdwEnabled": (
        "Enable Lane Departure Warnings",
        DESCRIPTIONS["IsLdwEnabled"],
        "warning.png",
        False,
      ),
      "AlwaysOnDM": (
        "Always-On Driver Monitoring",
        DESCRIPTIONS["AlwaysOnDM"],
        "monitoring.png",
        False,
      ),
      "RecordFront": (
        "Record and Upload Driver Camera",
        DESCRIPTIONS["RecordFront"],
        "monitoring.png",
        True,
      ),
      "RecordAudio": (
        "Record and Upload Microphone Audio",
        DESCRIPTIONS["RecordAudio"],
        "microphone.png",
        True,
      ),
      "IsMetric": (
        "Use Metric System",
        DESCRIPTIONS["IsMetric"],
        "metric.png",
        False,
      ),
      "GpsAlwaysSwitch": (
        "Always receive GPS signals",
        DESCRIPTIONS["GpsAlwaysSwitch"],
        "../offroad/icon_gps_car.png",
        False,
      ),
      "DisableMaxSpeedModify": (
        "Use the vehicle ACC with TSSP over 115 km/h",
        DESCRIPTIONS["DisableMaxSpeedModify"],
        "../icons/calibration.png",
        False,
      ),
      "ForceHybridVehicle": (
        "Use the vehicle ACC with TSSP over 115 km/h",
        DESCRIPTIONS["ForceHybridVehicle"],
        "disengage_on_accelerator.png",
        False,
      ),
      "IgnoreRerouteHarness": (
        "Ignore DSU bypass harness for TSSPh",
        DESCRIPTIONS["IgnoreRerouteHarness"],
        "../icons/calibration.png",
        False,
      ),
      # "RaylibMode": (
      #   "Use Raylib UI",
      #   DESCRIPTIONS["RaylibMode"],
      #   "warning.png",
      #   True,
      # ),
    }

    # Edit tethering password
    self._keyboard = Keyboard()

    self._long_personality_setting = multiple_button_item(
      "Driving Personality",
      DESCRIPTIONS["LongitudinalPersonality"],
      buttons=["Aggressive", "Standard", "Relaxed"],
      button_width=255,
      callback=self._set_longitudinal_personality,
      selected_index=self._params.get("LongitudinalPersonality", return_default=True),
      icon="speed_limit.png"
    )

    self._accel_method_setting = multiple_button_item(
      "Accel Method",
      DESCRIPTIONS["AccelMethodSwitch"],
      buttons=["Recommend", "Official"],
      button_width=270,
      callback=self._set_accel_method,
      selected_index=self._params.get("AccelMethodSwitch", return_default=True),
      icon="calibration.png"
    )

    self._toggles = {}
    self._locked_toggles = set()
    for param, (title, desc, icon, needs_restart) in self._toggle_defs.items():
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      if needs_restart and not locked:
        toggle.set_description(toggle.description + " Changing this setting will restart openpilot if the car is powered on.")

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

      # insert longitudinal personality after NDOG toggle
      if param == "DisengageOnAccelerator":
        self._toggles["LongitudinalPersonality"] = self._long_personality_setting
        self._toggles["AccelMethodSwitch"] = self._accel_method_setting

        self._tethering_password_action = ButtonAction(text="EDIT")
        self._tethering_password_action.set_enabled(True)
        self._tethering_password_btn = ListItem(title="Tethering Password", icons="../offroad/icon_car_key.png", description="24km/h", action_item=self._tethering_password_action, callback=self._edit_tethering_password)

        self._toggles["Tethering Password"] = self._tethering_password_btn

    self._update_experimental_mode_icon()
    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._long_personality_setting.action_item.set_selected_button(personality)
      ui_state.personality = personality

  def show_event(self):
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    e2e_description = (
      "openpilot defaults to driving in chill mode. Experimental mode enables alpha-level features that aren't ready for chill mode. " +
      "Experimental features are listed below:<br>" +
      "<h4>End-to-End Longitudinal Control</h4><br>" +
      "Let the driving model control the gas and brakes. openpilot will drive as it thinks a human would, including stopping for red lights and stop signs. " +
      "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; " +
      "mistakes should be expected.<br>" +
      "<h4>New Driving Visualization</h4><br>" +
      "The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. " +
      "The Experimental mode logo will also be shown in the top right corner."
    )

    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._toggles["ExperimentalMode"].action_item.set_enabled(True)
        self._toggles["ExperimentalMode"].set_description(e2e_description)
        self._long_personality_setting.action_item.set_enabled(True)
        self._accel_method_setting.action_item.set_enabled(True)
      else:
        # no long for now
        self._toggles["ExperimentalMode"].action_item.set_enabled(False)
        self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._long_personality_setting.action_item.set_enabled(False)
        self._accel_method_setting.action_item.set_enabled(False)
        self._params.remove("ExperimentalMode")

        unavailable = "Experimental mode is currently unavailable on this car since the car's stock ACC is used for longitudinal control."

        long_desc = unavailable + " openpilot longitudinal control may come in a future update."
        if ui_state.CP.alphaLongitudinalAvailable:
          if self._is_release:
            long_desc = unavailable + " " + ("An alpha version of openpilot longitudinal control can be tested, along with " +
                                             "Experimental mode, on non-release branches.")
          else:
            long_desc = "Enable the openpilot longitudinal control (alpha) toggle to allow Experimental mode."

        self._toggles["ExperimentalMode"].set_description("<b>" + long_desc + "</b><br><br>" + e2e_description)
    else:
      self._toggles["ExperimentalMode"].set_description(e2e_description)

    self._update_experimental_mode_icon()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(self._params.get_bool(param))

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

  def _render(self, rect):
    self._scroller.render(rect)

  def _update_experimental_mode_icon(self):
    icon = "experimental.png" if self._toggles["ExperimentalMode"].action_item.get_state() else "experimental_white.png"
    self._toggles["ExperimentalMode"].set_icon(icon)

  def _handle_experimental_mode_toggle(self, state: bool):
    confirmed = self._params.get_bool("ExperimentalModeConfirmed")
    if state and not confirmed:
      def confirm_callback(result: int):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("ExperimentalMode", True)
          self._params.put_bool("ExperimentalModeConfirmed", True)
        else:
          self._toggles["ExperimentalMode"].action_item.set_state(False)
        self._update_experimental_mode_icon()

      # show confirmation dialog
      content = (f"<h1>{self._toggles['ExperimentalMode'].title}</h1><br>" +
                 f"<p>{self._toggles['ExperimentalMode'].description}</p>")
      dlg = ConfirmDialog(content, "Enable", rich=True)
      gui_app.set_modal_overlay(dlg, callback=confirm_callback)
    else:
      self._update_experimental_mode_icon()
      self._params.put_bool("ExperimentalMode", state)

  def _toggle_callback(self, state: bool, param: str):
    if param == "ExperimentalMode":
      self._handle_experimental_mode_toggle(state)
      return

    self._params.put_bool(param, state)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True)

  def _set_longitudinal_personality(self, button_index: int):
    self._params.put("LongitudinalPersonality", button_index)

  def _set_accel_method(self, button_index: int):
    self._params.put_bool("AccelMethodSwitch", button_index == 1)

  def _edit_tethering_password(self):
    def update_password(result):
      if result != 1:
        return

      # password = self._keyboard.text
      # self._wifi_manager.set_tethering_password(password)
      # self._tethering_password_action.set_enabled(False)

    self._keyboard.reset(min_text_size=0)
    self._keyboard.set_title("Enter new tethering password", "")
    self._keyboard.set_text("aaaaaaaaaaaaaa")
    gui_app.set_modal_overlay(self._keyboard, update_password)

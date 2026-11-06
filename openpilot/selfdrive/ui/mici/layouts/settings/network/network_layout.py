from collections.abc import Callable

from openpilot.selfdrive.ui.mici.layouts.settings.network import EsimNetworkButton, WifiNetworkButton
from openpilot.selfdrive.ui.mici.layouts.settings.network.esim_ui import EsimUI
from openpilot.selfdrive.ui.mici.layouts.settings.network.wifi_ui import WifiUIMici
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiToggle, BigParamControl, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.cellular_manager import CellularManager
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network, MeteredType
from openpilot.system.ui.widgets.scroller import NavScroller


AUTH_FIELD_RULE = 'use 1-64 ascii letters, digits or symbols, without spaces, double quotes, backslashes or commas.'


def _valid_auth_field(text: str) -> bool:
  return 1 <= len(text) <= 64 and all(0x21 <= ord(c) <= 0x7e and c not in '"\\,' for c in text)


class ApnCredentialsConfirmDialog(NavScroller):
  def __init__(self, confirm_callback: Callable[[], None], message: str = "also delete the saved apn user name and password?"):
    super().__init__()
    no_btn = BigButton("no", "keep saved credentials")
    no_btn.set_click_callback(self.dismiss)
    yes_btn = BigButton("yes", "delete saved credentials")
    yes_btn.set_click_callback(lambda: self.dismiss(confirm_callback))
    self._scroller.add_widgets([
      GreyBigButton("", message),
      no_btn,
      yes_btn,
    ])


class NetworkLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)
    self._wifi_ui = WifiUIMici(self._wifi_manager)

    self._wifi_manager.add_callbacks(
      networks_updated=self._on_network_updated,
    )

    # ******** Tethering ********
    def tethering_toggle_callback(checked: bool):
      self._tethering_toggle_btn.set_enabled(False)
      self._tethering_password_btn.set_enabled(False)
      self._network_metered_btn.set_enabled(False)
      self._wifi_manager.set_tethering_active(checked)

    self._tethering_toggle_btn = BigToggle("enable tethering", "", toggle_callback=tethering_toggle_callback,
                                           description="Share the device's internet connection through a Wi-Fi hotspot.")

    def tethering_password_callback(password: str):
      if password:
        self._tethering_toggle_btn.set_enabled(False)
        self._tethering_password_btn.set_enabled(False)
        self._wifi_manager.set_tethering_password(password)

    def tethering_password_clicked():
      tethering_password = self._wifi_manager.tethering_password
      dlg = BigInputDialog("enter password...", tethering_password, minimum_length=8,
                           confirm_callback=tethering_password_callback)
      gui_app.push_widget(dlg)

    txt_tethering = gui_app.texture("icons_mici/settings/network/tethering.png", 64, 54)
    self._tethering_password_btn = BigButton("tethering password", "", txt_tethering)
    self._tethering_password_btn.set_click_callback(tethering_password_clicked)

    # ******** Network Metered ********
    def network_metered_callback(value: str):
      self._network_metered_btn.set_enabled(False)
      metered = {
        'default': MeteredType.UNKNOWN,
        'metered': MeteredType.YES,
        'unmetered': MeteredType.NO
      }.get(value, MeteredType.UNKNOWN)
      self._wifi_manager.set_current_network_metered(metered)

    # TODO: signal for current network metered type when changing networks, this is wrong until you press it once
    # TODO: disable when not connected
    self._network_metered_btn = BigMultiToggle("network usage", ["default", "metered", "unmetered"], select_callback=network_metered_callback,
                                               description="Metered prevents large uploads on this Wi-Fi connection. Default uses the network's detected " +
                                                           "setting.")
    self._network_metered_btn.set_enabled(False)

    self._wifi_button = WifiNetworkButton(self._wifi_manager)
    self._wifi_button.set_click_callback(lambda: gui_app.push_widget(self._wifi_ui))

    # ******** eSIM ********
    self._cellular_manager = CellularManager()
    self._esim_ui = EsimUI(
      self._cellular_manager,
      lambda: not ui_state.prime_state.is_full_prime(),
    )
    self._esim_button = EsimNetworkButton(self._cellular_manager)
    self._esim_button.set_click_callback(lambda: gui_app.push_widget(self._esim_ui))

    # ******** Advanced settings ********
    # ******** Roaming toggle ********
    self._roaming_btn = BigParamControl("enable roaming", "GsmRoaming", description="Allow cellular data roaming.")

    # ******** APN settings ********
    self._apn_btn = BigButton("apn settings", "edit",
                              description="Set the access point name required by your cellular carrier. Leave blank for automatic configuration.")
    self._apn_btn.set_click_callback(self._edit_apn)

    self._apn_user_btn = BigButton("apn user name", "not set", description=AUTH_FIELD_RULE)
    self._apn_user_btn.set_click_callback(self._edit_apn_user)
    self._apn_password_btn = BigButton("apn password", "not set", description=AUTH_FIELD_RULE)
    self._apn_password_btn.set_click_callback(self._edit_apn_password)
    self._apn_auth_btn = BigMultiToggle("apn auth", ["off", "chap", "pap"], select_callback=self._set_apn_auth)
    self._apn_delete_btn = BigButton("delete apn credentials", "delete")
    self._apn_delete_btn.set_click_callback(self._delete_apn_credentials)
    self._refresh_apn_auth()

    # ******** Cellular metered toggle ********
    self._cellular_metered_btn = BigParamControl("cellular metered", "GsmMetered", description="Prevent large uploads over the cellular connection.")

    # Main scroller ----------------------------------
    self._scroller.add_widgets([
      self._wifi_button,
      self._esim_button,
      self._network_metered_btn,
      self._tethering_toggle_btn,
      self._tethering_password_btn,
      # /* Advanced settings
      self._roaming_btn,
      self._apn_btn,
      self._apn_user_btn,
      self._apn_password_btn,
      self._apn_auth_btn,
      self._apn_delete_btn,
      self._cellular_metered_btn,
      # */
    ])

  def _update_state(self):
    super()._update_state()

    show_cell_settings = not ui_state.prime_state.is_full_prime()
    self._wifi_manager.set_ipv4_forward(show_cell_settings)
    self._roaming_btn.set_visible(show_cell_settings)
    self._apn_btn.set_visible(show_cell_settings)
    self._apn_user_btn.set_visible(show_cell_settings)
    self._apn_password_btn.set_visible(show_cell_settings)
    self._apn_auth_btn.set_visible(show_cell_settings)
    self._apn_delete_btn.set_visible(show_cell_settings and bool(self._apn_user or self._apn_has_password))
    self._cellular_metered_btn.set_visible(show_cell_settings)

  def show_event(self):
    super().show_event()
    self._refresh_apn_auth()
    self._wifi_manager.set_active(True)

    # Process wifi and esim callbacks while at any point in the nav stack
    gui_app.add_nav_stack_tick(self._wifi_manager.process_callbacks)
    gui_app.add_nav_stack_tick(self._cellular_manager.process_callbacks)

  def hide_event(self):
    super().hide_event()
    self._wifi_manager.set_active(False)

    gui_app.remove_nav_stack_tick(self._wifi_manager.process_callbacks)
    gui_app.remove_nav_stack_tick(self._cellular_manager.process_callbacks)

  def _edit_apn(self):
    def update_apn(apn: str):
      apn = apn.strip()
      if apn == "":
        ui_state.params.remove("GsmApn")
      else:
        ui_state.params.put("GsmApn", apn)
      self._refresh_apn_auth()

    current_apn = ui_state.params.get("GsmApn") or ""
    dlg = BigInputDialog("enter APN...", current_apn, minimum_length=0, confirm_callback=update_apn)
    gui_app.push_widget(dlg)

  def _refresh_apn_auth(self):
    auth = ui_state.params.get("GsmAuth")
    self._apn_auth = auth if auth in ("chap", "pap") else "off"
    self._apn_user = ui_state.params.get("GsmUsername") or ""
    self._apn_has_password = bool(ui_state.params.get("GsmPassword"))
    self._apn_auth_btn.set_value(self._apn_auth)
    self._apn_user_btn.set_value("set" if self._apn_user else "not set")
    self._apn_password_btn.set_value("set" if self._apn_has_password else "not set")

  def _delete_apn_credentials(self):
    def clear_credentials():
      ui_state.params.put("GsmAuth", "none", block=True)
      ui_state.params.remove("GsmUsername")
      ui_state.params.remove("GsmPassword")
      self._refresh_apn_auth()

    gui_app.push_widget(ApnCredentialsConfirmDialog(clear_credentials, message="delete the saved apn user name and password?"))

  def _set_apn_auth(self, value: str):
    # BigMultiToggle changes its value before calling us; compare with the cached param.
    if value == self._apn_auth:
      return
    ui_state.params.put("GsmAuth", "none" if value == "off" else value, block=True)
    self._refresh_apn_auth()
    if value == "off":
      def clear_credentials():
        ui_state.params.remove("GsmUsername")
        self._refresh_apn_auth()
        ui_state.params.remove("GsmPassword")
        self._refresh_apn_auth()

      gui_app.push_widget(ApnCredentialsConfirmDialog(clear_credentials))

  def _edit_apn_user(self):
    self._edit_apn_field("GsmUsername", "enter user name...", self._apn_user)

  def _edit_apn_password(self):
    self._edit_apn_field("GsmPassword", "enter password...", "")

  def _edit_apn_field(self, key: str, hint: str, default_text: str):
    def update(value: str):
      if _valid_auth_field(value):
        ui_state.params.put(key, value, block=True)
        self._refresh_apn_auth()

    gui_app.push_widget(BigInputDialog(hint, default_text, minimum_length=1, confirm_callback=update,
                                      text_validator=_valid_auth_field))

  def _on_network_updated(self, networks: list[Network]):
    # Update tethering state
    tethering_active = self._wifi_manager.is_tethering_active()
    # TODO: use real signals (like activated/settings changed, etc.) to speed up re-enabling buttons
    self._tethering_toggle_btn.set_enabled(True)
    self._tethering_password_btn.set_enabled(True)
    self._network_metered_btn.set_enabled(lambda: not tethering_active and bool(self._wifi_manager.ipv4_address))
    self._tethering_toggle_btn.set_checked(tethering_active)

    # Update network metered
    self._network_metered_btn.set_value(
      {
        MeteredType.UNKNOWN: 'default',
        MeteredType.YES: 'metered',
        MeteredType.NO: 'unmetered'
      }.get(self._wifi_manager.current_network_metered, 'default'))

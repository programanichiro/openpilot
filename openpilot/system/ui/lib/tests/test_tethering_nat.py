"""Tests for the legacy iptables NAT rule managed by WifiManager tethering."""
import subprocess
from unittest.mock import MagicMock, call

import pytest

from openpilot.system.ui.lib import wifi_manager
from openpilot.system.ui.lib.wifi_manager import TETHERING_NAT_RULE, WifiManager


@pytest.fixture
def wm(monkeypatch):
  monkeypatch.setattr(WifiManager, '_initialize', MagicMock())
  manager = WifiManager.__new__(WifiManager)
  manager._exit = True
  manager._conn_monitor = MagicMock()
  manager._tethering_ssid = "weedle"
  manager._ipv4_forward = True
  manager.activate_connection = MagicMock()
  manager._deactivate_connection = MagicMock()

  def synchronous_thread(*, target, daemon):
    thread = MagicMock()
    thread.start.side_effect = target
    return thread

  monkeypatch.setattr(wifi_manager.threading, 'Thread', synchronous_thread)
  return manager


@pytest.fixture
def run(monkeypatch):
  run = MagicMock(return_value=subprocess.CompletedProcess([], 0))
  monkeypatch.setattr(wifi_manager.subprocess, 'run', run)
  return run


def test_activate_adds_missing_rule_after_connection(wm, run):
  run.side_effect = [subprocess.CompletedProcess([], 1), subprocess.CompletedProcess([], 0)]
  calls = MagicMock()
  calls.attach_mock(wm.activate_connection, 'activate')
  calls.attach_mock(run, 'run')

  wm.set_tethering_active(True)

  assert calls.mock_calls == [
    call.activate(wm._tethering_ssid, block=True),
    call.run(["sudo", "iptables-legacy", "-t", "nat", "-C", *TETHERING_NAT_RULE],
             check=False, capture_output=True, text=True),
    call.run(["sudo", "iptables-legacy", "-t", "nat", "-A", *TETHERING_NAT_RULE],
             check=True, capture_output=True, text=True),
  ]


def test_activate_keeps_existing_rule(wm, run):
  wm.set_tethering_active(True)

  run.assert_called_once_with(["sudo", "iptables-legacy", "-t", "nat", "-C", *TETHERING_NAT_RULE],
                              check=False, capture_output=True, text=True)


def test_activate_missing_iptables_does_not_raise(wm, run, monkeypatch):
  run.side_effect = FileNotFoundError("iptables-legacy")
  log_exception = MagicMock()
  monkeypatch.setattr(wifi_manager.cloudlog, 'exception', log_exception)

  wm.set_tethering_active(True)

  run.assert_called_once_with(["sudo", "iptables-legacy", "-t", "nat", "-C", *TETHERING_NAT_RULE],
                              check=False, capture_output=True, text=True)
  log_exception.assert_called_once_with("Failed to add tethering NAT rule")


def test_deactivate_removes_rule_after_connection(wm, run):
  run.return_value = subprocess.CompletedProcess([], 1)
  calls = MagicMock()
  calls.attach_mock(wm._deactivate_connection, 'deactivate')
  calls.attach_mock(run, 'run')

  wm.set_tethering_active(False)

  assert calls.mock_calls == [
    call.deactivate(wm._tethering_ssid),
    call.run(["sudo", "iptables-legacy", "-t", "nat", "-D", *TETHERING_NAT_RULE],
             check=False, capture_output=True, text=True),
  ]


def test_activate_without_forwarding_still_disables_ip_forward(wm, run, monkeypatch):
  wm._ipv4_forward = False
  sleep = MagicMock()
  monkeypatch.setattr(wifi_manager.time, 'sleep', sleep)
  calls = MagicMock()
  calls.attach_mock(run, 'run')
  calls.attach_mock(sleep, 'sleep')

  wm.set_tethering_active(True)

  assert calls.mock_calls == [
    call.run(["sudo", "iptables-legacy", "-t", "nat", "-C", *TETHERING_NAT_RULE],
             check=False, capture_output=True, text=True),
    call.sleep(5),
    call.run(["sudo", "sysctl", "net.ipv4.ip_forward=0"], check=False),
  ]

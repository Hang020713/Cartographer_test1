from types import SimpleNamespace

import onboard_scripts.mavlink_controller as controller_mod


class DummyThread:
    def __init__(self, target, daemon=True):
        self.target = target
        self.daemon = daemon

    def start(self):
        return None

    def is_alive(self):
        return False


class FakeMaster:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1

    def wait_heartbeat(self):
        return None

    def mode_mapping(self):
        return {}

    def recv_match(self, *args, **kwargs):
        return None


def test_get_imu_data_returns_scaled_imu2_fields(monkeypatch):
    monkeypatch.setattr(controller_mod.mavutil, "mavlink_connection", lambda *args, **kwargs: FakeMaster())
    monkeypatch.setattr(controller_mod.threading, "Thread", DummyThread)

    controller = controller_mod.MavController(port="dummy", baud=115200, imu_enabled=True)
    msg = SimpleNamespace(xacc=100, yacc=-200, zacc=300, xgyro=10, ygyro=-20, zgyro=30)

    controller._update_imu_data(msg)
    imu_data = controller.get_imu_data()

    assert imu_data["acc_x"] == 100
    assert imu_data["acc_y"] == -200
    assert imu_data["acc_z"] == 300
    assert imu_data["gyro_x"] == 10
    assert imu_data["gyro_y"] == -20
    assert imu_data["gyro_z"] == 30

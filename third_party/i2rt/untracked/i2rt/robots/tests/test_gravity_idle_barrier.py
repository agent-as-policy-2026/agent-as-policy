import threading

import numpy as np
import pytest

from i2rt.motor_drivers.dm_driver import MotorInfo
from i2rt.robots.motor_chain_robot import MotorChainRobot


class _RecordingMotorChain:
    running = True

    def __init__(self) -> None:
        self.applied: list[dict[str, np.ndarray]] = []

    def __len__(self) -> int:
        return 2

    def read_states(self) -> list[MotorInfo]:
        return [
            MotorInfo(
                id=index + 1,
                error_code=1,
                pos=0.1 * (index + 1),
                vel=0.0,
                eff=0.0,
                timestamp=1.0,
            )
            for index in range(2)
        ]

    def set_commands(self, *args: object, **kwargs: object) -> list[MotorInfo]:
        raise AssertionError("gravity idle must not use the asynchronous command buffer")

    def apply_commands(
        self,
        torques: np.ndarray,
        *,
        pos: np.ndarray,
        vel: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
    ) -> list[MotorInfo]:
        self.applied.append(
            {
                "torques": torques.copy(),
                "pos": pos.copy(),
                "vel": vel.copy(),
                "kp": kp.copy(),
                "kd": kd.copy(),
            }
        )
        return self.read_states()

    def close(self) -> None:
        pass


def test_gravity_idle_returns_only_after_zero_gain_command_is_applied(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Hand guiding must not begin while an earlier position hold can still be active."""

    class _DormantThread:
        def __init__(self, *, target: object, name: str) -> None:
            self.target = target

        def start(self) -> None:
            pass

        def join(self) -> None:
            pass

    monkeypatch.setattr(threading, "Thread", _DormantThread)
    chain = _RecordingMotorChain()
    robot = MotorChainRobot(
        motor_chain=chain,
        use_gravity_comp=False,
        joint_limits=np.array([[-1.0, 1.0], [-1.0, 1.0]]),
        kp=np.array([80.0, 40.0]),
        kd=np.array([5.0, 3.0]),
        grav_comp_kd=np.array([0.25, 0.15]),
        zero_gravity_mode=False,
    )

    robot.command_joint_pos(np.array([0.3, -0.2]))
    robot.enter_gravity_comp_idle()

    assert len(chain.applied) == 1
    applied = chain.applied[0]
    np.testing.assert_allclose(applied["torques"], np.zeros(2))
    np.testing.assert_allclose(applied["pos"], np.zeros(2))
    np.testing.assert_allclose(applied["vel"], np.zeros(2))
    np.testing.assert_allclose(applied["kp"], np.zeros(2))
    np.testing.assert_allclose(applied["kd"], [0.25, 0.15])

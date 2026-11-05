import math

from rdw.controllers import discover_controllers
from rdw.controllers.base import Redirector
from rdw.controllers.none import NoRedirection
from rdw.controllers.s2c import SteerToCenter
from rdw.models import AxisAlignedSpace, Observation, RedirectionCommand


def observation(position: tuple[float, float], heading: float) -> Observation:
    return Observation(
        step=0,
        dt=0.05,
        physical_position=position,
        physical_heading=heading,
        virtual_position=(0.0, 0.0),
        virtual_heading=0.0,
        walking_speed=1.2,
        space=AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0),
    )


def test_controllers_share_the_same_input_output_contract() -> None:
    controllers = (NoRedirection(), SteerToCenter())

    for controller in controllers:
        command = controller.compute(observation((1.0, 0.0), math.pi / 2))
        assert isinstance(command, RedirectionCommand)


def test_controller_modules_are_discovered_automatically() -> None:
    controllers = discover_controllers()

    assert set(controllers) == {"apf", "none", "s2c", "s2o"}
    assert issubclass(NoRedirection, Redirector)
    assert issubclass(SteerToCenter, Redirector)
    assert isinstance(controllers["none"](), NoRedirection)
    assert isinstance(controllers["s2c"](), SteerToCenter)


def test_s2c_turns_toward_room_center() -> None:
    controller = SteerToCenter()

    turn_from_north = controller.compute(observation((2.0, 0.0), math.pi / 2))
    controller.reset()
    turn_from_south = controller.compute(observation((2.0, 0.0), -math.pi / 2))

    assert turn_from_north.curvature > 0
    assert turn_from_south.curvature < 0
    assert abs(turn_from_north.curvature) <= 1.0 / controller.curvature_radius

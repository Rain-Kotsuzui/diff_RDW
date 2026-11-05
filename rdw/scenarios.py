from rdw.models import AxisAlignedSpace, Scenario


def default_scenarios() -> tuple[Scenario, ...]:
    square_path = ((0.0, 0.0), (12.0, 0.0), (12.0, 12.0), (0.0, 12.0))
    zigzag_path = (
        (0.0, 0.0),
        (10.0, 3.0),
        (2.0, 7.0),
        (11.0, 11.0),
        (1.0, 15.0),
    )
    return (
        Scenario(
            name="square_room",
            space=AxisAlignedSpace(-3.0, 3.0, -3.0, 3.0),
            virtual_waypoints=square_path,
        ),
        Scenario(
            name="narrow_room",
            space=AxisAlignedSpace(-2.0, 2.0, -4.0, 4.0),
            virtual_waypoints=zigzag_path,
        ),
    )

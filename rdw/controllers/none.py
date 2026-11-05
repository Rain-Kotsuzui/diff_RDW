from rdw.controllers.base import Redirector
from rdw.models import Observation, RedirectionCommand


class NoRedirection(Redirector):
    @property
    def name(self) -> str:
        return "none"

    def reset(self) -> None:
        pass

    def compute(self, observation: Observation) -> RedirectionCommand:
        return RedirectionCommand()


CONTROLLER_FACTORY = NoRedirection

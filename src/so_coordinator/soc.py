"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

from behaviour_components.network_behaviour import NetworkBehavior
from behaviour_components.managers import Manager


class SoCoordinator(NetworkBehavior):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour. This creates a meta level and eases the modellation of complex
    problems
    """

    def __init__(self, effects, name, requires_execution_steps=False,
                 only_running_for_deciding_interruptible=
                 Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE,
                 **kwargs):

        super(SoCoordinator, self).__init__(self, effects=effects, name=name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            only_running_for_deciding_interruptible=
                                            only_running_for_deciding_interruptible,
                                            **kwargs)
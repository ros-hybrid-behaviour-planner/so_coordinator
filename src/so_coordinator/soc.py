"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

from behaviour_components.network_behavior import NetworkBehavior
from behaviour_components.managers import Manager
from so_data.chemotaxis import ChemotaxisBalch


class SoCoordinator(NetworkBehavior):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour. This creates a meta level and eases modelling complex
    problems
    """

    def __init__(self, effect, class_name=None, name='SoCoordinator', requires_execution_steps=False,
                 only_running_for_deciding_interruptible=
                 Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE,
                 turtle_number=1, min_activation=0, max_activation=1,
                 min_distance=0, max_distance=1, clock_topic=None,
                 **kwargs):

        self.name = name

        self.manager_name = name + NetworkBehavior.MANAGER_POSTFIX

        super(SoCoordinator, self).__init__(effects=effect, name=self.name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            only_running_for_deciding_interruptible=
                                            only_running_for_deciding_interruptible,
                                            **kwargs)

        behaviours = class_name({
            'buffer': {'chem': {}},
            'mechanism': {'chem': [ChemotaxisBalch, 'chem', {'moving': False, 'static': True}]},
        'activator': {},
        'sensor': {},
        'condition': {},
        'behaviour': {},
        'goal': {}},
            turtle_number, min_activation, max_activation,
                                min_distance, max_distance, clock_topic) #, planner_prefix=self.manager_name)
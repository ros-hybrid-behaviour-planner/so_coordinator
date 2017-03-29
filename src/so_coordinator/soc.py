"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

import os
from behaviour_components.network_behavior import NetworkBehavior
from so_coordinator.so_components import SOComponents, create_from_yaml
from so_mapping import SO_MAPPING


class SoCoordinator(NetworkBehavior):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour. This creates a meta level and eases modelling complex
    problems
    """

    # requires execution steps = True is super important!
    def __init__(self, effects, id=1, so_goal='ReachGoal',
                 expert_knowledge='so_knowledge.yaml',
                 components_class=SOComponents,
                 name='SoCoordinator',
                 mapping=SO_MAPPING,
                 requires_execution_steps=True,
                 pose_frame='robot',
                 motion_topic='',
                 **kwargs):
        """
        initialization
        :param effect:
        :param components_class:
        :param id:
        :param so_goal:
        :param expert_knowledge:
        :param name:
        :param requires_execution_steps: whether the execution steps should be
                                         caused from the parent manager or not.
        :param kwargs:
        """

        self.name = name

        self.manager_name = name + NetworkBehavior.MANAGER_POSTFIX

        super(SoCoordinator, self).__init__(effects=effects, name=self.name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            **kwargs)

        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   expert_knowledge),
                                      id,
                                      planner_prefix=self.get_manager_prefix(),
                                      so_goal=so_goal,
                                      motion_topic=motion_topic,
                                      mapping=mapping,
                                      components_class=components_class,
                                      pose_frame=pose_frame)


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

    def __init__(self, effects, so_goal, id='robot1',
                 expert_knowledge='so_knowledge.yaml',
                 components_class=SOComponents, name='SoCoordinator',
                 mapping=SO_MAPPING, requires_execution_steps=True,
                 params=None, **kwargs):

        """
        initialization
        :param effects: effects of SoCoordinator (on higher level)
        :param so_goal: self-organization goal used in coordination mechanism
                        selection
        :param id: id of the agent
        :param expert_knowledge: yaml file containing mapping between so goal
                                 and setup of RHBP components
        :param components_class: class containing factory to create RHBP
                                 components
        :param name: unique name of the object
        :param mapping: dictionary mapping strings to classes
        :param requires_execution_steps: whether the execution steps should be
                                         caused from the parent manager or not.
        :param kwargs: keyword arguments
        """

        # init Network Behaviour
        super(SoCoordinator, self).__init__(effects=effects, name=name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            **kwargs)

        # Coordination Mechanism Selection

        # create SO components based on expert knowledge
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   expert_knowledge),
                                      id,
                                      planner_prefix=self.get_manager_prefix(),
                                      so_goal=so_goal,
                                      params=params,
                                      mapping=mapping,
                                      components_class=components_class)

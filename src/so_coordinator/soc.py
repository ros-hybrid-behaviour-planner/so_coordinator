"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

import os
from behaviour_components.network_behavior import NetworkBehavior
from so_coordinator.so_components import SOComponents, create_from_yaml
from so_mapping import SO_MAPPING
from decision_strategy import DecisionStrategy


class SoCoordinator(NetworkBehavior):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour and eases modelling complex problems
    """

    def __init__(self, effects, so_goal, id='robot1', name='SoCoordinator',
                 path=os.path.dirname(__file__),
                 expert_knowledge='so_expert_knowledge.yaml',
                 pattern_knowledge='so_specification.yaml',
                 decision=DecisionStrategy,
                 components_class=SOComponents,
                 mapping=SO_MAPPING,
                 requires_execution_steps=True,
                 params=None, optional_params=None, **kwargs):

        """
        initialization
        :param effects: effects of SoCoordinator (on higher level)
        :param so_goal: self-organization goal used in coordination mechanism
                        selection
        :param id: id of the agent
        :param name: unique name of the object
        :param path: path to the yaml files
        :param expert_knowledge: yaml file containing mapping goal - option
        :param pattern_knowledge: yaml file containing option configurations
        :param decision: class containing decision strategy
        :param components_class: class containing factory to create RHBP
                                 components
        :param mapping: dictionary mapping strings to classes
        :param requires_execution_steps: whether the execution steps should be
                                         caused from the parent manager or not.
        :param params: agent specific parameters for creation of RHBP
                       components
        :param optional_params: dictionary indicating component parameters to
                                be adjusted; form: component_key:  {parameters}
        :param kwargs: keyword arguments
        """

        # init parent class (NetworkBehaviour)
        super(SoCoordinator, self).__init__(effects=effects, name=name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            **kwargs)

        # Coordination Mechanism Selection

        # 1) Expert Knowledge + Decision Making Strategy
        self.selector = decision(so_goal, os.path.join(path, expert_knowledge))
        config_key = self.selector.select()

        # 2) Creation of SO Components based on selected so configuration
        self.components = create_from_yaml(
            os.path.join(path, pattern_knowledge), id,
            planner_prefix=self.get_manager_prefix(), config_key=config_key,
            params=params, mapping=mapping, components_class=components_class,
            optional_params=optional_params)

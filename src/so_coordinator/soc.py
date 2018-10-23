"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

import os
import rospy
from rhbp_core.srv import RemoveBehaviour, RemoveGoal
from behaviour_components.network_behavior import NetworkBehaviour
from so_coordinator.so_components import SOComponents, create_from_yaml
from so_mapping import SO_MAPPING
from decision_strategy import DecisionStrategy


class SoCoordinator(NetworkBehaviour):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour and eases modelling complex problems
    """

    def __init__(self, so_goal, correlations=None, id='robot1',
                 name='SoCoordinator', path=os.path.dirname(__file__),
                 expert_knowledge='so_expert_knowledge.yaml',
                 pattern_knowledge='so_specification.yaml',
                 decision=DecisionStrategy,
                 components_class=SOComponents,
                 mapping=SO_MAPPING,
                 requires_execution_steps=True,
                 params=None, optional_params=None, **kwargs):

        """
        initialization
        :param correlations: effects of SoCoordinator (on higher level)
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

        self.path = path
        self.pattern_knowledge = pattern_knowledge
        self.params = params
        self.mapping = mapping
        self.components_class = components_class
        if optional_params is None:
            self.optional_params = {}
        else:
            self.optional_params = optional_params
        self.id = id

        # init parent class (NetworkBehaviour)
        super(SoCoordinator, self).__init__(name=name, requires_execution_steps=requires_execution_steps,
                                            correlations=correlations, always_update_activation=True, **kwargs)

        # Coordination Mechanism Selection

        # 1) Expert Knowledge + Decision Making Strategy
        self.selector = decision(so_goal, os.path.join(path, expert_knowledge))
        selection = self.selector.select()
        # combine optional params and parameters included in expert knowledge
        # (optional params dominate params included in expert knowledge)
        selection[1].update(self.optional_params)

        # 2) Creation of SO Components based on selected so configuration
        self.components = create_from_yaml(
            os.path.join(path, pattern_knowledge), id,
            planner_prefix=self.get_manager_prefix(), config_key=selection[0],
            params=params, mapping=mapping, components_class=components_class,
            optional_params=selection[1])

    def remove_components(self):
        """
        method to remove components from manager and from SOComponents class
        :return:
        """
        rospy.wait_for_service(self.get_manager_prefix() +
                               '/RemoveBehaviour', 5)
        remove = rospy.ServiceProxy(self.get_manager_prefix() +
                                    '/RemoveBehaviour', RemoveBehaviour)

        for b in self.components.behaviours.values():
            remove(b._name)

        # delete Goal
        rospy.wait_for_service(self.get_manager_prefix() +
                               '/RemoveGoal', 5)
        remove_goal = rospy.ServiceProxy(self.get_manager_prefix() +
                                         '/RemoveGoal', RemoveGoal)

        for g in self.components.goals.values():
            remove_goal(g._name)

        # delete in SoC
        self.components.delete_components()

    def replace_components(self, config_key):
        """
        method to remove existing components and create new ones
        :return:
        """
        # remove current components
        self.remove_components()

        # create new components
        self.components = create_from_yaml(
            os.path.join(self.path, self.pattern_knowledge), self.id,
            planner_prefix=self.get_manager_prefix(), config_key=config_key,
            params=self.params, mapping=self.mapping,
            components_class=self.components_class,
            optional_params=self.optional_params)

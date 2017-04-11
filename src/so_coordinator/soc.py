"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

import os
import yaml
import rospy
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
                 expert_knowledge='so_expert_knowledge.yaml',
                 pattern_knowledge='so_specification.yaml',
                 components_class=SOComponents, name='SoCoordinator',
                 mapping=SO_MAPPING, requires_execution_steps=True,
                 params=None, optional_params=None, **kwargs):

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
        pattern_key = self.coordination_mechanism_selection(so_goal,
                                                            expert_knowledge)

        # create SO components based on expert knowledge
        self.components = create_from_yaml(
            os.path.join(os.path.dirname(__file__), pattern_knowledge), id,
            planner_prefix=self.get_manager_prefix(), pattern_key=pattern_key,
            params=params, mapping=mapping, components_class=components_class,
            optional_params=optional_params)

    def coordination_mechanism_selection(self, so_goal, expert_knowledge):
        """
        method to determine coordination mechanism for self-organization
        :return: pattern key
        """

        data = self.load_expert_knowledge(expert_knowledge)[so_goal]

        # only one element available
        if len(data) == 1:
            return data[0][0]

        # several elements: take element with highest score
        else:
            max_vals = [item[1] for item in data]
            return data[max_vals.index(max(max_vals))][0]

    @staticmethod
    def load_expert_knowledge(expert_knowledge):
        """
        method to lead yaml file with expert knowledge/
        :return: data set related to expert knowledge
        """

        # load yaml file
        data = None

        with open(os.path.join(os.path.dirname(__file__), expert_knowledge),
                  'r') as stream:
            try:
                data = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # determine key for component creation
        if data:
            return data
        else:
            rospy.logerr("Loading expert knowledge failed.")

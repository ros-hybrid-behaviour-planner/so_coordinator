"""
Created on 22.03.2017

@author: kaiser

Module including self-organization coordinator
"""

import yaml
import os
import rospy
from behaviour_components.network_behavior import NetworkBehavior


class SoCoordinator(NetworkBehavior):
    """
    Self-organization coordinator which allows to encapsulate self-organising
    behaviour. This creates a meta level and eases modelling complex
    problems
    """

    # requires execution steps = True is super important!
    def __init__(self, effects, class_name=None, id=1, so_goal='ReachGoal',
                 expert_knowledge='so_knowledge.yaml',
                 name='SoCoordinator',
                 requires_execution_steps=True,
                 **kwargs):
        """
        initialization
        :param effect:
        :param class_name:
        :param id:
        :param so_goal:
        :param expert_knowledge:
        :param name:
        :param requires_execution_steps: whether the execution steps should be caused from the parent manager or not.
                If not, the step method must be called manually
        :param kwargs:
        """

        self.name = name

        self.manager_name = name + NetworkBehavior.MANAGER_POSTFIX

        super(SoCoordinator, self).__init__(effects=effects, name=self.name,
                                            requires_execution_steps=
                                            requires_execution_steps,
                                            **kwargs)

        data = None

        with open(os.path.join(os.path.dirname(__file__), expert_knowledge),
                  'r') as stream:
            try:
                data = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # create
        if data:
            behaviours = class_name(data[so_goal], id,
                                    planner_prefix=self.get_manager_prefix())
        else:
            rospy.logerr("SO behaviour creation in " + self.name + " failed.")

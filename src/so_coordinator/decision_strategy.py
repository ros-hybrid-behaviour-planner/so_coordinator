"""
Created on 11.04.2017

@author: kaiser

Module including class for decision making strategy
"""

import yaml
from option import Option


class DecisionStrategy(object):
    """
    class containing the decision strategy to select a coordination mechanism
    """
    def __init__(self, so_goal, expert_knowledge):
        """
        initialization
        :param so_goal: self organization goal
        :param expert_knowledge: path to expert knowledge yaml file
        """
        # store self-organization options
        self.options = self.load_options(so_goal, expert_knowledge)

    def select(self):
        """
        method selecting the coordination mechanism
        :return: configuration key for self-organization components creation
        """

        # only one element available
        if len(self.options) == 1:
            return self.options[0].config_key

        # several elements: take element with highest score
        else:
            max_vals = [item.score for item in self.options]
            return self.options[max_vals.index(max(max_vals))].config_key

    @staticmethod
    def load_options(so_goal, expert_knowledge):
        """
        method to load expert knowledge / options matching to given goal
        :param so_goal: self organization goal
        :param expert_knowledge: path to expert knowledge yaml file
        :return: list of Option objects
        """
        # load yaml file
        data = None
        with open(expert_knowledge, 'r') as stream:
            try:
                data = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # determine data related to so goal
        data = data[so_goal]

        # transform options to class objects
        options = []
        for element in data:
            options.append(Option(element))

        return options

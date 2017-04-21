"""
Created on 11.04.2017

@author: kaiser

Module including class to store options defined in expert knowledge
"""


class Option(object):
    """class to store option"""

    def __init__(self, option):
        """
        initialization: store option parameters
        :param option: [config_key, score, parameter]
        """
        self._config_key = option[0]
        self.score = option[1]
        self.params = option[2]

    @property
    def config_key(self):
        return self._config_key
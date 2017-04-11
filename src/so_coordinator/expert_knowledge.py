"""
Created on 11.04.2017

@author: kaiser

Module including class to store mapping / options of expert knowledge
"""


class Option(object):
    """class to store option"""

    def __init__(self, option):
        """
        initialization
        :param option: [config_key, score]
        """
        self._config_key = option[0]
        self.score = option[1]

    @property
    def config_key(self):
        return self._config_key
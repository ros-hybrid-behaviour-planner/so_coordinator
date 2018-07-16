"""
Created on 05.04.2017

@author: kaiser, hrabia

Module containing validator for expert knowledge files (yaml)
"""

import yaml
import os
import unittest
from so_coordinator.so_mapping import SO_MAPPING


class DecisionStrategyTest(unittest.TestCase):

    def validate(self, file, mapping):
        """
        method to check whether so_specification is in required format
        :param file: yaml file to be checked
        :param mapping: mapping containing mapping string - class
        """

        # keys which are allowed to be used in a configuration
        keys = ['buffer', 'mechanisms', 'activators', 'sensors', 'conditions',
                'behaviours', 'preconditions', 'goals']

        # 1) load yaml file
        with open(file, 'r') as stream:
            try:
                data = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

        for config in data.keys():
            # check configuration specification == dict
            if not isinstance(data[config], dict):
                self.assertTrue(False, "Configuration " + config + " is not a dictionary.")

            # 2) check keys of component levels (goal, buffer etc.)
            for k in data[config].keys():
                # check if component config is a dict
                if not isinstance(data[config][k], dict):
                    self.assertTrue(False, "Component specification " + k + " of " + config + " is not a dictionary.")

                if k not in keys:
                    self.assertTrue(False, "Unknown component key used in " + config + ": " + k)

                # 3) check specification of individual components (length etc.)
                if k == 'buffer':
                    for el in data[config][k].keys():
                        if not isinstance(data[config][k][el], dict):
                            self.assertTrue(False, "Wrong data type to specify SoBuffer parameters of " + el + " in "
                                            + config)

                elif k == 'preconditions':
                    for el in data[config][k].keys():
                        if not all(isinstance(i, list) for i
                                   in data[config][k][el]):
                            self.assertTrue(False, "Wrong data type to specify preconditions of behaviour " + el
                                            + " in " + config)
                else:
                    for el in data[config][k].values():
                        # component config has to be a list
                        if not isinstance(el, list):
                            self.assertTrue(False, "Component specification " + el[0] + " of " + config +
                                            " is not an array.")

                        # length has to be 2
                        if len(el) != 2:
                            self.assertTrue(False, "Component specification " + el[0] + " of " + \
                                  config + " has the wrong length.")

                        # first element has to be in mapping
                        if el[0] not in mapping.keys():
                            self.assertTrue(False, "Class " + el[0] + " used in " + \
                                  config + " is not in mapping!")

                        # second element has to be a dictionary
                        if not isinstance(el[1], dict):
                            self.assertTrue(False, "Second element of component configuration " +  \
                                  el[0] + " in " + config + " is not a dictionary.")

    def test_specification_file(self):
        self.validate(os.path.join(os.path.dirname(__file__), '../so_coordinator/so_specification.yaml'), SO_MAPPING)


if __name__ == '__main__':
    unittest.main()

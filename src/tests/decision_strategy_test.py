"""
Created on 12.04.2017

@author: kaiser

Module containing unit test for decision_strategy.py
"""

import os
import unittest
from so_coordinator.decision_strategy import DecisionStrategy
from so_coordinator.option import Option


class DecisionStrategyTest(unittest.TestCase):
    """
    class to test DecisionStrategy
    """
    def test_selection(self):
        """
        test selection
        :return:
        """
        path = os.path.join(os.path.dirname(__file__),
                            '../so_coordinator/so_expert_knowledge.yaml')

        # only one possible answer / direct mapping
        strategy = DecisionStrategy('ReachGoal', path)
        self.assertEqual(strategy.select(), ['ChemotaxisGe', {}])
        self.assertEqual(len(strategy.options), 2)

        # more than one option available
        strategy = DecisionStrategy('AgentCollisionAvoidance', path)
        self.assertEqual(strategy.select(), ['RepulsionGradient', {}])
        self.assertTrue(all(isinstance(el, Option) for el in strategy.options))
        self.assertEqual(len(strategy.options), 2)

        # add option with score with lower value
        strategy.options.append(Option(['Test', 0.75, {}]))
        self.assertEqual(strategy.select(), ['RepulsionGradient', {}])

        # add option with score with higher value
        strategy.options.append(Option(['Test', 1.75, {}]))
        self.assertEqual(strategy.select(), ['Test', {}])


# run tests - start roscore before running tests
if __name__ == '__main__':
    unittest.main()


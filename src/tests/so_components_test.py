"""
Created on 04.04.2017

@author: kaiser

Module containing unit test for so_components.py
"""

import unittest
import rospy
from so_coordinator.so_components import SOComponents
from behaviour_components.managers import Manager
from behaviour_components.activators import MultiSensorCondition, \
    PublisherCondition, BooleanActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator
from behaviour_components.conditions import Conjunction, Negation
from behaviour_components.goals import GoalBase
from rhbp_selforga.behaviours import MoveBehaviour, DecisionBehaviour
from rhbp_selforga.conditions import VectorDistCondition, GoalBoolCondition, \
    VectorBoolCondition, ChangeFloatCondition
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from so_data.chemotaxis import ChemotaxisGe
from so_data.sobuffer import SoBuffer


class SoComponentsTest(unittest.TestCase):
    """
    Test of component creation with SoComponents class
    """
    def __init__(self, *args, **kwargs):
        """
        test creation of SOComponents
        :return:
        """
        super(SoComponentsTest, self).__init__(*args, **kwargs)

    def test_buffer(self):
        """
        method to test initialization of buffer
        """
        # buffer created with specified parameter
        self.assertEqual(len(components.buffer), 1)
        self.assertEqual(components.buffer['bf_chem'].view_distance, 1.5)
        self.assertTrue(isinstance(components.buffer['bf_chem'], SoBuffer),
                        "Not the correct object")

    def test_mechanisms(self):
        """
        method to test initialization of mechanisms
        """
        self.assertEqual(len(components.mechanisms), 1)
        self.assertEqual(components.mechanisms['m_chem']._buffer,
                         components.buffer['bf_chem'])
        self.assertTrue(isinstance(components.mechanisms['m_chem'],
                                   ChemotaxisGe), "Not the correct object")
        self.assertEqual(components.mechanisms['m_chem'].frames, ['Center'])

    def test_activators(self):
        """
        method to test initialization of activators
        """
        self.assertEqual(len(components.activators), 2)
        self.assertTrue('a_bool' in components.activators.keys())
        self.assertEqual(components.activators['a_lin'].zeroActivationValue,
                         1.0)
        self.assertTrue(isinstance(components.activators['a_bool'],
                                   BooleanActivator), "Not the correct object")
        self.assertTrue(isinstance(components.activators['a_lin'],
                                   LinearActivator), "Not the correct object")

    def test_sensors(self):
        """
        method to test initialization of sensors
        """
        self.assertEqual(len(components.sensors), 3)
        self.assertEqual(components.sensors['s_goal'].mechanism,
                        [components.mechanisms['m_chem'],
                         components.mechanisms['m_chem']])
        self.assertTrue(isinstance(components.sensors['s_goal'],
                                   GradientSensor), "Not the correct object")

    def test_conditions(self):
        """
        method to test initialization of conditions
        """
        self.assertEqual(len(components.conditions), 4)
        self.assertEqual(components.conditions['c_goal'].sensor,
                         components.sensors['s_goal'])
        self.assertTrue(components.conditions['c_dist'].optional)
        self.assertEqual(components.conditions['c_goal'].activator,
                         components.activators['a_bool'])
        self.assertEqual(components.conditions['c_goal_reached'].sensor,
                         components.sensors['s_goal_reached'])
        self.assertEqual(components.conditions['c_goal_reached'].activator,
                         components.activators['a_bool'])
        self.assertFalse(components.conditions['c_goal_reached'].optional)
        self.assertEqual(components.conditions['c_dist'].sensor,
                         components.sensors['s_dist'])
        self.assertEqual(components.conditions['c_dist'].activator,
                         components.activators['a_lin'])
        self.assertTrue(components.conditions['c_dist'].optional)

        self.assertTrue(isinstance(components.conditions['c_dist'],
                                   VectorDistCondition),
                        "Not the correct object")
        self.assertTrue(isinstance(components.conditions['c_goal'],
                                   VectorBoolCondition),
                        "Not the correct object")

        # Multi Sensor Condition
        self.assertTrue(isinstance(components.conditions['c_morph'],
                                   ChangeFloatCondition),
                        "Not the correct object")
        self.assertEqual(components.conditions['c_morph'].sensors,
                         [components.sensors['s_goal'],
                          components.sensors['s_dist']])

    def test_behaviours(self):
        """
        method to test initialization of behaviours
        """
        # behaviours
        self.assertEqual(len(components.behaviours), 2)
        self.assertEqual(components.behaviours['b_chem'].mechanism,
                         components.mechanisms['m_chem'])
        self.assertTrue(isinstance(components.behaviours['b_chem'],
                                   MoveBehaviour), "Not the correct object")

        # preconditions
        self.assertEqual(components.behaviours['b_chem']._preconditions[1],
                         components.conditions['c_dist'])
        self.assertEqual(len(components.behaviours['b_chem']._preconditions),
                         3)

        self.assertTrue(isinstance(components.behaviours['b_dec'],
                                   DecisionBehaviour),
                        "Not the correct object")

    def test_goals(self):
        """
        method to test initialization of goals
        """
        self.assertEqual(len(components.goals), 1)
        self.assertEqual(components.goals['g_chem']._conditions[0],
                         components.conditions['c_goal_reached'])
        self.assertTrue(isinstance(components.goals['g_chem'], GoalBase),
                        "Not the correct object")
        self.assertTrue(components.goals['g_chem']._permanent)

    def test_effect(self):
        """
        test for method create_effect
        :return:
        """
        self.assertEqual(components.create_effect(['c_dist', -1.0, 'float']),
                         [components.conditions['c_dist'], -1.0, float])
        self.assertEqual(components.create_effect(['c_dist', -1.0, 'int']),
                         [components.conditions['c_dist'], -1.0, float])
        self.assertEqual(components.create_effect(['c_goal', 1.0, 'bool']),
                         [components.conditions['c_goal'], 1.0, bool])

    def test_condition(self):
        """
        test method for create_condition
        :return:
        """
        # basic condition
        self.assertEqual(components.create_condition(['None', 'c_dist']),
                         components.conditions['c_dist'])

        # Negation
        self.assertTrue(isinstance(components.create_condition(
            ['Negation', 'c_dist']), Negation), "Not the correct object.")
        self.assertEqual(components.create_condition(['Negation', 'c_dist']
                                                     )._condition,
                         components.conditions['c_dist'])

        # Conjunction
        c = components.create_condition(['Conjunction',
                                         [['None', 'c_dist'],
                                          ['Negation', 'c_goal']]])
        self.assertTrue(isinstance(c, Conjunction), "Not the correct object.")
        self.assertEqual(c._conditions[0], components.conditions['c_dist'])
        self.assertTrue(isinstance(c._conditions[1], Negation),
                       "Not the correct object.")
        self.assertEqual(c._conditions[1]._condition,
                         components.conditions['c_goal'])


# run tests - start roscore before running tests
if __name__ == '__main__':
    rospy.init_node('test')
    m = Manager()
    specs = {'buffer': {'bf_chem': {'view_distance': 2.0}},
             'mechanisms': {'m_chem': ['ChemotaxisGe', {
                 'buffer': 'bf_chem', 'moving': False, 'static': True}]},
             'activators': {'a_bool': ['BooleanActivator', {}],
                                 'a_lin': ['LinearActivator',
                                           {'zeroActivationValue': 2.0,
                                            'fullActivationValue': 0.0,
                                            'minActivation': 0.0,
                                            'maxActivation': 1.0}]},
                  'sensors': {'s_goal': ['GradientSensor', {'mechanism':
                                                                'm_chem'}],
                              's_goal_reached': ['GradientSensor',
                                                 {'mechanism': 'm_chem',
                                                  'sensor_type': 'goal'}],
                              's_dist': ['GradientSensor',
                                         {'mechanism': 'm_chem',
                                          'sensor_type': 'goal'}]},
                  'conditions': {'c_goal': ['VectorBoolCondition',
                                            {'sensor': 's_goal',
                                             'activator': 'a_bool'}],
                                 'c_goal_reached': ['GoalBoolCondition',
                                                    {'sensor': 's_goal_reached',
                                                        'activator': 'a_bool'}],
                                 'c_dist': ['VectorDistCondition',
                                            {'sensor': 's_dist',
                                             'activator': 'a_lin',
                                             'optional': True}],
                                 'c_morph': ['ChangeFloatCondition',
                                             {'sensors': ['s_goal', 's_dist'],
                                              'activator': 'a_bool'}]},
                  'behaviours': {'b_chem': ['MoveBehaviour',
                                            {'mechanism': 'm_chem',
                                             'param_keys': ['motion_topic'],
                                             'effects': [['c_dist', -1.0,
                                                          'float'],
                                                         ['c_goal_reached',
                                                          1.0, 'bool']]}],
                                 'b_dec':  ['DecisionBehaviour',
                                            {'mechanism': 'm_chem',
                                             'effects': [['c_morph', -1.0, bool],
                                                         ['c_goal', 1.0, bool]],
                                             'value_key': 'value',
                                             'state_key': 'state'}]},
                  'preconditions': {'b_chem': [['Negation', 'c_goal_reached'],
                                               ['None', 'c_dist'],
                                               ['None', 'c_goal']]},
                  'goals': {'g_chem': ['GoalBase', {'permanent': False,
                                                    'conditions':
                                                        [['None',
                                                          'c_goal_reached']]}]}
                  }

    components = SOComponents(specs, 'r1', params={'motion_topic': '/cmd_vel'},
                              optional_params={
                                  'a_lin': {'zeroActivationValue': 1.0},
                                  'bf_chem': {'view_distance': 1.5},
                                  'm_chem': {'frames': ['Center']},
                                  's_goal': {'mechanism': ['m_chem', 'm_chem']},
                                  'c_goal': {'optional': True},
                                  'b_chem': {'motion_topic': '/cmd_vel2'},
                                  'g_chem': {'permanent': True}
                              })

    unittest.main()
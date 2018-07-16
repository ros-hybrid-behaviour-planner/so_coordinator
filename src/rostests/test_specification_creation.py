#!/usr/bin/env python
"""
Created on 13.04.2017

@author: kaiser, hrabia

Module containing unit test for creation of specified components
"""

import os
import rospy
import unittest
import rostest
from behaviour_components.managers import Manager
from so_coordinator.so_components import create_from_yaml, SOComponents
from so_coordinator.so_mapping import SO_MAPPING


class TestConfigurationCreation(unittest.TestCase):
    """
    unit test to test if all configurations are created without errors
    """

    def __init__(self, *args, **kwargs):
        super(TestConfigurationCreation, self).__init__(*args, **kwargs)

    @classmethod
    def setUpClass(self):
        rospy.init_node('TestConfigurationCreation_node')
        TestConfigurationCreation.m = Manager()
        # Wait for manager becoming available
        while True:
            try:
                rospy.wait_for_service('/AddBehaviour', timeout=0.1)
            except rospy.ROSException:
                rospy.sleep(1)
                continue
            break

    @classmethod
    def tearDownClass(self):
        TestConfigurationCreation.m.unregister()

    def test_chemotaxisge(self):
        """
        test creation of ChemotaxisGe
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='ChemotaxisGe',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents)
        self.assert_(True)

    def test_flocking_chemo(self):
        """
        test creation of FlockingReyChemotaxisBalch
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='FlockingReyChemotaxisBalch',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='flocktest1')
        self.assert_(True)

    def test_flocking_exploration(self):
        """
        test creation of FlockingReyExploration
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='FlockingReyExploration',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents)
        self.assert_(True)

    def test_morpho_chem(self):
        """
        test creation of MorphogenesisBarycenterChemotaxisBalch
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='MorphogenesisBarycenterChemotaxisBalch',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='chemtest2')
        self.assert_(True)

    def test_foraging(self):
        """
        test creation of Foraging
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='Foraging',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='foragingtest')
        self.assert_(True)

    def test_exploration(self):
        """
        test creation of ExplorationChemotaxisGe
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='ExplorationChemotaxisGe',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='explorationchemtest')
        self.assert_(True)

    def test_center(self):
        """
        test creation of Foraging
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='MorphogenesisBarycenter',
                                      params={'pose_frame': 'pose_frame'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='centertest')
        self.assert_(True)

    def test_gossip(self):
        """
        test creation of GossipMax
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='GossipMax',
                                      params={'pose_frame': 'pose_frame',
                                              'value': 0,
                                              'initial_value': 0},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='gossipmax')
        self.assert_(True)

    def test_repFernandez(self):
        """
        test creation of RepulsionFernandez
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='RepulsionFernandez',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='repFernandez')
        self.assert_(True)

    def test_repGradient(self):
        """
        test creation of RepulsionFernandez
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='RepulsionGradient',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='repGrad')
        self.assert_(True)

    def test_coll_avoidance(self):
        """
        test creation of CollisionAvoidance
        """
        components = create_from_yaml(os.path.join(os.path.dirname(__file__),
                                                   '../so_coordinator/'
                                                   'so_specification.yaml'),
                                      'test1',
                                      config_key='CollisionAvoidance',
                                      params={'pose_frame': 'pose_frame',
                                              'motion_topic': 'motion_topic'},
                                      mapping=SO_MAPPING,
                                      components_class=SOComponents,
                                      name='CollisionAvoidance')
        self.assert_(True)


if __name__ == '__main__':
    rostest.rosrun("so_coordinator", 'TestConfigurationCreation', TestConfigurationCreation)

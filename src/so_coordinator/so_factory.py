"""
Created on 22.03.2017

@author: kaiser
"""

from __future__ import division # force floating point division when using plain /
from rhbp_selforga.behaviours import MoveBehaviour, DecisionBehaviour
from behaviour_components.conditions import Negation
from behaviour_components.activators import LinearActivator, BooleanActivator
from rhbp_selforga.conditions import VectorBoolCondition, GoalBoolCondition, \
    VectorDistCondition
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from so_data.sobuffer import SoBuffer
from so_data.chemotaxis import ChemotaxisBalch

class SoFactory(SOAgent):
    """
    factory to create so mechanisms
    """

    behaviours = [MoveBehaviour, DecisionBehaviour]

    def create_mechanism(type, id='', pose_frame='robot', clock_topic='None',
                         min_distance=0, max_distance=1, min_activation=0,
                         max_activation=1, motion_topic=None, moving=False, static=True, **kwargs):
        if type == 'move':
            chem_buffer = SoBuffer(id=id, pose_frame=pose_frame)
            chem = ChemotaxisBalch(chem_buffer, moving=False, static=True)

            # attractive gradient within view
            goal_sensor = GradientSensor('goalSensor' + id, chem, clock_topic)

            bool_activator = BooleanActivator()

            goal_condition = VectorBoolCondition(goal_sensor,
                                                 bool_activator,
                                                 name=id + "GoalCondition")

            # Behaviour
            gradient_behaviour = MoveBehaviour(mechanism=chem,
                                               # distance_condition=
                                               # distance_condition,
                                               # bool_p_condition=[
                                               #     goal_reached_condition],
                                                motion_topic=motion_topic,
                                                name='chemotaxisBehaviour' +
                                                     id)

           # gradient_behaviour.addPrecondition(goal_condition)

            return [[gradient_behaviour], [goal_condition]]

        if type == 'decision':
            return DecisionBehaviour(**kwargs)

    factory = staticmethod(create_mechanism)

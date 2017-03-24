"""
Created on 24.03.2017

@author: kaiser
"""

from so_data.chemotaxis import ChemotaxisBalch
from behaviour_components.activators import LinearActivator, BooleanActivator
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from rhbp_selforga.conditions import VectorBoolCondition, GoalBoolCondition, \
    VectorDistCondition
from rhbp_selforga.behaviours import MoveBehaviour
from behaviour_components.goals import GoalBase
from behaviour_components.conditions import Negation, Conjunction
from so_data.sobuffer import SoBuffer


SO_MAPPING = {
    'Negation': Negation,
    'GoalBase': GoalBase,
    'MoveBehaviour': MoveBehaviour,
    'VectorBoolCondition': VectorBoolCondition,
    'GoalBoolCondition': GoalBoolCondition,
    'VectorDistCondition': VectorDistCondition,
    'GradientSensor': GradientSensor,
    'SENSOR': SENSOR,
    'LinearActivator': LinearActivator,
    'BooleanActivator': BooleanActivator,
    'ChemotaxisBalch': ChemotaxisBalch,
    'SoBuffer': SoBuffer,
    'Conjunction': Conjunction
}
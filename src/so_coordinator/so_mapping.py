"""
Created on 24.03.2017

@author: kaiser

Module containing mapping from strings to classes
"""

from behaviour_components.activators import MultiSensorCondition, \
    PublisherCondition, BooleanActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator
from behaviour_components.conditions import Disjunction, Conjunction, Negation
from behaviour_components.goals import GoalBase, OfflineGoal, PublisherGoal
from behaviour_components.sensors import PassThroughTopicSensor, \
    SimpleTopicSensor, DynamicSensor

from rhbp_utils.knowledge_sensors import KnowledgeSensor, KnowledgeFactSensor

from rhbp_selforga.behaviours import MoveBehaviour, TurnBehaviour, \
    DecisionBehaviour, DecisionStateBehaviour, SetStateBehaviour
from rhbp_selforga.conditions import ChangeFloatCondition, \
    ChangeStringCondition, BoolTCondition, BoolFCondition, VectorBoolCondition,\
    GoalBoolCondition, MaxDistCondition, VectorDistCondition
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

from so_data.chemotaxis import ChemotaxisGe, ChemotaxisBalch, \
    CollisionAvoidance, FollowAll, AvoidAll, FollowStrongest, FollowMin, \
    FollowMinReach, FollowMaxReach
from so_data.decisions import MorphogenesisBarycenter, GossipMax, Quorum
from so_data.flocking import Flocking
from so_data.flockingrey import FlockingRey
from so_data.foraging import ForagingDecision, DepositPheromones, \
    Exploitation, Exploration, STATE
from so_data.repulsion import RepulsionFernandez, RepulsionGradient
from so_data.sobuffer import SoBuffer, AGGREGATION
from so_data.supplements import DepositPheromonesMin, DepositPheromonesRandom, \
    SpreadGradient


SO_MAPPING = {

    # RHBP COMPONENTS
    # Activators
    'MultiSensorCondition': MultiSensorCondition,
    'PublisherCondition': PublisherCondition,
    'BooleanActivator': BooleanActivator,
    'ThresholdActivator': ThresholdActivator,
    'GreedyActivator': GreedyActivator,
    'LinearActivator': LinearActivator,
    # Conditions
    'Disjunction': Disjunction,
    'Conjunction': Conjunction,
    'Negation': Negation,
    # Goals
    'GoalBase': GoalBase,
    'OfflineGoal': OfflineGoal,
    'PublisherGoal': PublisherGoal,
    # Sensors
    'PassThroughTopicSensor': PassThroughTopicSensor,
    'SimpleTopicSensor': SimpleTopicSensor,
    'DynamicSensor': DynamicSensor,

    # RHBP UTILS
    # knowledge sensors
    'KnowledgeSensor': KnowledgeSensor,
    'KnowledgeFactSensor': KnowledgeFactSensor,

    # RHBP_SELFORGA COMPONENTS
    # Behaviours
    'MoveBehaviour': MoveBehaviour,
    'TurnBehaviour': TurnBehaviour,
    'DecisionBehaviour': DecisionBehaviour,
    'DecisionStateBehaviour': DecisionStateBehaviour,
    'SetStateBehaviour': SetStateBehaviour,
    # Conditions
    'ChangeFloatCondition': ChangeFloatCondition,
    'ChangeStringCondition': ChangeStringCondition,
    'BoolTCondition': BoolTCondition,
    'BoolFCondition': BoolFCondition,
    'VectorBoolCondition': VectorBoolCondition,
    'MaxDistCondition': MaxDistCondition,
    'GoalBoolCondition': GoalBoolCondition,
    'VectorDistCondition': VectorDistCondition,
    # Gradient Sensor
    'GradientSensor': GradientSensor,
    'SENSOR': SENSOR,

    # SO_DATA COMPONENTS
    # Chemotaxis
    'ChemotaxisGe': ChemotaxisGe,
    'ChemotaxisBalch': ChemotaxisBalch,
    'CollisionAvoidance': CollisionAvoidance,
    'FollowAll': FollowAll,
    'AvoidAll': AvoidAll,
    'FollowStrongest': FollowStrongest,
    'FollowMin': FollowMin,
    'FollowMinReach': FollowMinReach,
    'FollowMaxReach': FollowMaxReach,
    # Decisions
    'MorphogenesisBarycenter': MorphogenesisBarycenter,
    'GossipMax': GossipMax,
    'Quorum': Quorum,
    # Flocking
    'Flocking': Flocking,
    # FlockingRey
    'FlockingRey': FlockingRey,
    # Foraging
    'ForagingDecision': ForagingDecision,
    'DepositPheromones': DepositPheromones,
    'Exploitation': Exploitation,
    'Exploration': Exploration,
    'STATE': STATE,
    # Repulsion
    'RepulsionFernandez': RepulsionFernandez,
    'RepulsionGradient': RepulsionGradient,
    # Sobuffer
    'SoBuffer': SoBuffer,
    'AGGREGATION': AGGREGATION,
    # supplements
    'DepositPheromonesMin': DepositPheromonesMin,
    'DepositPheromonesRandom': DepositPheromonesRandom,
    'SpreadGradient': SpreadGradient
}
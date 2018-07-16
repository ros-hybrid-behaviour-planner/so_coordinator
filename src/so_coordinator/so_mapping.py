"""
Created on 24.03.2017

@author: kaiser, hrabia

Module containing mapping from strings to classes
contains rhbp components, rhbp utils, rhbp_selforga, so_data
"""

from behaviour_components.activators import BooleanActivator, ThresholdActivator, GreedyActivator, \
    LinearActivator
from behaviour_components.conditions import Disjunction, Conjunction, Negation, PublisherCondition
from behaviour_components.goals import GoalBase, OfflineGoal, PublisherGoal
from behaviour_components.sensors import RawTopicSensor, \
    TopicSensor, DynamicSensor, PassThroughTopicSensor, SimpleTopicSensor

from rhbp_utils.knowledge_sensors import KnowledgeSensor, KnowledgeFactSensor, KnowledgeFirstFactSensor

from rhbp_selforga.behaviours import MoveBehaviour, TurnBehaviour, \
    DecisionBehaviour, DecisionStateBehaviour, SetStateBehaviour
from rhbp_selforga.conditions import ChangeKBFloatCondition, ChangeKBStringCondition, BoolTCondition, BoolFCondition, VectorBoolCondition,\
    GoalBoolCondition, MaxDistCondition, VectorDistCondition, \
    GradientBoolCondition
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
from so_data.patrolling import Patrol


SO_MAPPING = {

    # RHBP COMPONENTS
    # Activators
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
    'RawTopicSensor': RawTopicSensor,
    'TopicSensor': TopicSensor,
    'PassThroughTopicSensor': PassThroughTopicSensor,
    'SimpleTopicSensor': SimpleTopicSensor,
    'DynamicSensor': DynamicSensor,

    # RHBP UTILS
    # knowledge sensors
    'KnowledgeSensor': KnowledgeSensor,
    'KnowledgeFactSensor': KnowledgeFactSensor,
    'KnowledgeFirstFactSensor': KnowledgeFirstFactSensor,

    # RHBP_SELFORGA COMPONENTS
    # Behaviours
    'MoveBehaviour': MoveBehaviour,
    'TurnBehaviour': TurnBehaviour,
    'DecisionBehaviour': DecisionBehaviour,
    'DecisionStateBehaviour': DecisionStateBehaviour,
    'SetStateBehaviour': SetStateBehaviour,
    # Conditions
    'ChangeKBFloatCondition': ChangeKBFloatCondition,
    'ChangeKBStringCondition': ChangeKBStringCondition,
    'BoolTCondition': BoolTCondition,
    'BoolFCondition': BoolFCondition,
    'VectorBoolCondition': VectorBoolCondition,
    'MaxDistCondition': MaxDistCondition,
    'GoalBoolCondition': GoalBoolCondition,
    'GradientBoolCondition': GradientBoolCondition,
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
    'SpreadGradient': SpreadGradient,
    # patrolling
    'Patrol': Patrol
}

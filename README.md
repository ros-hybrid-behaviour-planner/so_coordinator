SO_COORDINATOR
==============

Package so_coordinator includes modules which provide a meta-level for the integration of self-organization into the RHBP.

The following files and modules are included:

* **so_knowledge.yaml**: expert knowledge to create RHBP components for self-organization based on a self-organization goal
* **so_mapping.py**: dictionary mapping strings to mechanism/component classes
* **so_components.py**: factory to create RHBP components based on a given specification
* **soc.py**: module containing self-organization coordinator 





so_knowledge(.yaml)
-------------------

YAML files can be used to map application scenarios of self-organization to the setup of mechanisms in the RHBP.
 
It should have the following structure:

```yamlex
SO_SCENARIO: {
  buffer: {buffer_key: {buffer parameters} },
  mechanisms: {mechanism_key: [so_mechanism_implementation, {mechanism parameters}]},
  activators: {activator_key: [rhbp_activator, {activator parameters}]},
  sensors: {sensor_key: [rhbp_sensor, {sensor parameters}]}, 
  conditions: {goal_key: [rhbp_condition, {condition parameters}, optional_bool]},
  behaviours: {behaviour_key: [rhbp_behaviour, {behaviour parameters}]},
  preconditions: {behaviour_key: [preconditions]},
  goals: {goal_key: [rhbp_goal, permanent_bool, [conditions]]}
}
```

The SO_SCENARIO specifies thereby a value which can be set as the so_goal of the so_coordinator. 
Not all aspects have to be specified, e.g. when no buffer is needed for the implementation, this part of the specification can simply be omitted. 

In general, there are no restrictions for the parameters to be specified within the parameter dictionaries.
However, some parameters require particular specifications to allow a correct component creation in class so_coordinator. 
The following list indicates these components:   

* **mechanism/mechanisms**: the mechanism_key of the used mechanism has to be specified and will be mapped within so_coordinator to the mechanism object
* **sensor**: the sensor_key of the used sensor has to be specified and will be mapped to the sensor object
* **activator**: the activator_key of the used activator has to be specified and will be mapped to the activator object
* **motion_topic**: boolean, if specified as True, the motion topic will be inserted in so_coordinator
 

Furthermore, conditions and effects have to be specified as follows: 
```yamlex
condition: [Modifier, condition_key / List of conditions]
effect: [condition, indicator, type]
```
Conditions consists of a modifier, e.g. Negation or Disjunction, and a condition key or list of conditions. 
One condition key is specified, when the precondition or goal is composed of one condition. 
A list of conditions can be handed over in case that more complex constructs are build, e.g. a Disjunction requires that a list of two conditions is handed over. 

Effects are composed of one condition as specified above, an indicator which can be a number between + and - 1 as well as a type, like bool or float. 

####Example: 

The following example allows to create a chemotaxis scenario to reach a goal while avoiding repulsive gradients. 

```yamlex
ReachGoal: {
  buffer: {bf_chem: {} },
  mechanisms: {m_chem: [ChemotaxisGe, {buffer: 'bf_chem', moving: False, static: True}]},
  activators: {a_bool: [BooleanActivator, {}],
               a_lin: [LinearActivator, {zeroActivationValue: 2.0, fullActivationValue: 0.0, minActivation: 0.0, maxActivation: 1.0}]},
  sensors: {s_goal: [GradientSensor, {mechanism: 'm_chem'}],
            s_goal_reached: [GradientSensor, {mechanism: 'm_chem', sensor_type: goal}],
            s_dist: [GradientSensor, {mechanism: 'm_chem', sensor_type: goal}]},
  conditions: {c_goal: [VectorBoolCondition, {sensor: 's_goal', activator: 'a_bool'}, False],
               c_goal_reached: [GoalBoolCondition, {sensor: 's_goal_reached', activator: 'a_bool'}, False],
               c_dist: [VectorDistCondition, {sensor: 's_dist', activator: 'a_lin'}, True]},
  behaviours: {b_chem: [MoveBehaviour, {mechanism: 'm_chem', motion_topic: True, effects: [[[None, 'c_dist'], -1.0, float], [[None, 'c_goal_reached'], 1.0, bool]]}]},
  preconditions: {b_chem: [[Negation, 'c_goal_reached'], [None, 'c_dist'], [None, 'c_goal']]},
  goals: {g_chem: [GoalBase, {permanent: False, conditions: [[None, 'c_goal_reached']]}]}
}
```

so_mapping(.py)
---------------


so_components(.py)
------------------

Module so_components includes a factory which allows to create RHBP components as specified by a specification dictionary. 
The structure has to be equal to the one used in the yaml files. 



soc(.py)
--------
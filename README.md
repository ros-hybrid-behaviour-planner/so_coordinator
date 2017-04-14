SO_COORDINATOR
==============

Package so_coordinator includes modules which provide a meta-level for the integration of self-organization into the RHBP.

The following files and modules are included:

1) Self-Organization Coordinator 
* **soc.py**: module containing self-organization coordinator 

2) Expert Knowledge
* **so_expert_knowledge.yaml**: mapping self-organization goal to options [configuration key, score]
* **so_specification.py**: dictionary including configurations per configuration key
* **so_mapping.py**: dictionary including mapping string to class for RHBP components 

3) Decision Strategy 
* **decision_strategy.py**: class containing decision mechanism to select coordination mechanism 
* **option.py**: class to store options from expert knowledge in decision strategy 

4) Components Factory
* **so_components.py**: class to create and store RHBP components based on a given specification



Self-Organization Coordinator: soc(.py)
---------------------------------------

Module soc includes the implementation of the Self-Organization Coordinator. 
The Self-Organization Coordinator inherits from the rhbp_core class NetworkBehaviour.
Therewith, each Self-Organization Coordinator has an own RHBP instance which will handle the components created within it. 

The use of the Self-Organization Coordinator has two advantages:
1) It encapsulates the self-organization behaviour and eases the use of different self-organization patterns / configurations in a complex setting
2) It takes over the selection of the coordination mechanism from the system designer

```python
class SoCoordinator(NetworkBehavior):
    def __init__(self, effects, so_goal, id='robot1', name='SoCoordinator',
                 path=os.path.dirname(__file__),
                 expert_knowledge='so_expert_knowledge.yaml',
                 pattern_knowledge='so_specification.yaml',
                 decision=DecisionStrategy,
                 components_class=SOComponents,
                 mapping=SO_MAPPING,
                 requires_execution_steps=True,
                 params=None, optional_params=None, **kwargs):
```

Class SoCoordinator has the following parameters: 
`effects` specifies the effects the SoCoordinator Behaviour has related to the higher-level RHBP instance (= RHBP instance which the SoCoordinator instance is assigned to).
`so_goal` indicates the self-organization goal. It has to match one of the keys used in the expert_knowledge yaml file. 
`id` is the identifier of the robot using the SoCoordinator.
`name` is a unique name for the SoCoordinator object.
`path` specifies the path to the expert_knowledge and pattern_knowledge yaml files.
`expert_knowledge` specifies the yaml file containing the expert knowledge (mapping so goal - options).
`pattern_knowledge` specifies the yaml file containing the component configuration per configuration key. 
`decision` indicates the class containing the decision making strategy implementation.
`components` specifies the class containing the self-organization components factory.
`mapping` specifies the dictionary containing the mapping from string to class.
`requires_execution_steps` is a RHBP parameter and indicates that the behaviour execution requires several steps to fulfill the goal. 
`params` includes the parameters to be replaced by agent specific values when creating the RHBP components as a dictionary, e.g. `{'motion_topic': '\cmd_vel'}`.
`optional_params` is a dictionary of dictionaries which specifies per component (component key from pattern_knowledge yaml) a dictionary with parameters to be adjusted. 
This allows to manually adapt the specifications for a specific scenario. 

As expert_knowledge, pattern_knowledge, mapping, decision and components are specified via the parameters, they can be replaced easily.
The modules presented below are the default options. 

When an object of SoCoordinator is created, the init method of NetworkBehaviour will be called first. 
Afterwards, the Coordination Mechanism Selection takes place. 
First the decision making strategy implementation decides based on the provided expert knowledge which configuration to use.
It returns a configuration key which is handed over to the components class. 
Furthermore, a dictionary with parameters is returned which will be updated with the optional parameters and handed over to SOComponents using keyword argument `optional_params`. 
The components class creates and stores the required components, which are specified in the pattern knowledge yaml. 
The created components are associated with the RHBP instance of the SoCoordinator. 

The SoCoordinator allows to change the used components or rather configuration.
It provides method `remove_components()` to delete all components and to remove them from the RHBP instance. 
Method `replace_components(config_key)` can be called with a configuration key and will first remove the existing components and create the new ones based on the configuration keys. 
Therewith, the SoCoordinator allows reconfigurations of the used self-organization configuration during runtime. 



Expert Knowledge
----------------

All of the files presented below are the default options used in the SoCoordinator.
They can be exchanged by custom yaml files which have the presented structure.

### so_expert_knowledge(.yaml)

The so_expert_knowledge(.yaml) file contains a mapping between self-organization goal and a list of options. 
Each option consists of a configuration key, a score and a parameter dictionary in the following form: `[config_key, score, params]`.
The score is a float number while the config_key has to match one of the keys in so_specifications.
The self-organization goal has to be a string. 
The parameter dictionary maps component key to a dictionary of keyword arguments. 

Example:
```yamlex
CollisionAvoidance: [[CollisionAvoidance, 1.0, {}]]
AgentCollisionAvoidance: [[RepulsionGradient, 1.0, {}], 
                          [RepulsionFernandez, 1.0, {bf_rep: {view_distance: 1.5} } ]]
```
Each self-organization goal can have one or more options associated to it. 


### so_specification(.yaml)

The so_specification file contains self-organization configuration specifications.
Their keys are used in the so_expert_knowledge.yaml file to map a so_goal to a configuration.
 
The so_specification should have the following structure:

```yamlex
# First configuration
config_key: {
  buffer: {buffer_key: {buffer parameters} },
  mechanisms: {mechanism_key: [so_mechanism_implementation, {mechanism parameters}]},
  activators: {activator_key: [rhbp_activator, {activator parameters}]},
  sensors: {sensor_key: [rhbp_sensor, {sensor parameters}]}, 
  conditions: {goal_key: [rhbp_condition, {condition parameters}]},
  behaviours: {behaviour_key: [rhbp_behaviour, {behaviour parameters}]},
  preconditions: {behaviour_key: [preconditions]},
  goals: {goal_key: [rhbp_goal, {goal parameter}]}
}

# Next configuration 
config_key2: {...}
```

Each configuration is structured as follows.
`config_key` is the unique identifier for the specified configuration.
It is used in the mapping between so_goal and so configuration in so_expert_knowledge.yaml.
The value of each config_key is a dictionary including the specification of the RHBP components. 

The specification dictionary itself can include the following keys:

* **buffer**: to define SoBuffer objects
* **mechanisms**: to define mechanisms included in so_data
* **activators**: RHBP activators 
* **sensors**: RHBP sensors (e.g. from package rhbp_core, rhbp_selforga or rhbp_utils)
* **conditions**: RHBP conditions (e.g. from package rhbp_selforga)
* **behaviours**: RHBP behaviours (e.g. from package rhbp_selforga)
* **preconditions**: to specify preconditions of RHBP behaviours
* **goals**: RHBP Goals (e.g. from package rhbp_core)

Not all components have to be specified, e.g. when no goal is needed in the implementation, the goal dictionary can be left empty. 

Except for the buffer and preconditions, each component definition follows the same structure:

```yamlex
{key: [class_name, {parameters}]}
```

The `key` is used to create a unique name of the component and has to be unique, too. 
Naming conventions are described in the end of this section. 
`class_name` is a string which can mapped to a class using the dictionary provided in so_mapping.py.
`{parameters}` is a dictionary of keyword arguments for the classes. 
They are defined similar as being handed over to `__init__()` of a class but replacing `=` by `:`, e.g. `moving: True`. 
All parameters which can be handed over to the component class can be specified there. 

buffer requires no class_name to be indicated as only one typ of SoBuffer exists. 
Therewith, only the parameters have to be specified. 
preconditions takes as the key the behaviour they are connected to and has a list of conditions as its value.

Some parameters required particular specifications to ensure a correct component creation in class SoCoordinator: 

* **mechanism/mechanisms**: the mechanism_key of the used mechanism has to be specified and will be mapped within so_coordinator to the mechanism object
* **sensor**: the sensor_key of the used sensor has to be specified and will be mapped to the sensor object
* **activator**: the activator_key of the used activator has to be specified and will be mapped to the activator object
* **param_keys**: this parameter is usually not part of the component creation when using the classes directly. 
It contains a list of parameters which have to be replaced with  agent specific values, e.g. motion_topic or pose_frame. 
SOComponents will add the specified parameter keys and their value to the parameter dictionary and removes the param_keys element afterwards. 

Furthermore, conditions and effects have to be specified in the following format: 
```yamlex
condition: [Modifier, condition_key / List of conditions]
effect: [condition, indicator, type]
```
Conditions consists of a modifier, e.g. Negation or Disjunction, and a condition key or list of conditions. 
One condition key is specified, when the precondition or goal is composed of one condition. 
A list of conditions can be handed over in case that more complex constructs are build, e.g. a Disjunction requires that a list of two conditions is handed over. 

Effects are composed of one condition as specified above, an indicator which can be a number between + and - 1 as well as a type, like bool or float. 


#### Naming Conventions

For the sake of clarity, it is proposed to use the following prefixes for each component type:

* buffer: bf_
* mechanism: m_
* activators: a_
* sensors: s_
* conditions: c_
* behaviours: b_
* goals: g_ 

For example, a buffer could have the key `bf_chemotaxis`. 
Following the naming increases the transparency of the complete configuration as components refer to each other. 

Furthermore, it is recommended to put component keys used in the parameter list of other components in single quotes to mark them. 

#### Example

The following example allows to create a chemotaxis scenario to reach a goal while avoiding repulsive gradients. 

```yamlex
# required params: pose_frame, motion_topic
ReachGoal: {
  buffer: {bf_chem: {param_keys: [pose_frame]} },
  mechanisms: {m_chem: [ChemotaxisGe, {buffer: 'bf_chem', moving: False, static: True}]},
  activators: {a_bool: [BooleanActivator, {}],
               a_lin: [LinearActivator, {zeroActivationValue: 2.0, fullActivationValue: 0.0, minActivation: 0.0, maxActivation: 1.0}]},
  sensors: {s_goal: [GradientSensor, {mechanism: 'm_chem'}],
            s_goal_reached: [GradientSensor, {mechanism: 'm_chem', sensor_type: goal}],
            s_dist: [GradientSensor, {mechanism: 'm_chem', sensor_type: goal}]},
  conditions: {c_goal: [VectorBoolCondition, {sensor: 's_goal', activator: 'a_bool'}],
               c_goal_reached: [GoalBoolCondition, {sensor: 's_goal_reached', activator: 'a_bool'},],
               c_dist: [VectorDistCondition, {sensor: 's_dist', activator: 'a_lin', optional: True}]},
  behaviours: {b_chem: [MoveBehaviour, {mechanism: 'm_chem', param_keys: [motion_topic], effects: [['c_dist', -1.0, float], ['c_goal_reached', 1.0, bool]]}]},
  preconditions: {b_chem: [[Negation, 'c_goal_reached'], [None, 'c_dist'], [None, 'c_goal']]},
  goals: {g_chem: [GoalBase, {permanent: False, conditions: [[None, 'c_goal_reached']]}]}
}
```

**Important Note**: The successful creation of the components is only ensured when all required param_keys are handed over to the SoCoordinator/SOComponents class. 
 To ease the implementation, the required param keys should be stated in a comment above the configuration specification! 

### so_mapping(.py)

so_mapping contains a dictionary mapping strings to classes. 
This mapping is used in SoComponents to map the specified string from so_specification to a class. 
An object of this class will then be created using the parameters indicated in so_specification.

Example:

```python 
from behaviour_components.activators import MultiSensorCondition, \
    PublisherCondition, BooleanActivator

SO_MAPPING = {
    'MultiSensorCondition': MultiSensorCondition,
    'PublisherCondition': PublisherCondition,
    'BooleanActivator': BooleanActivator
}

```


Decision Strategy
-----------------

### option(.py)

Module option includes class Option which allows to store the options defined in so_expert_knowledge.yaml as python class objects. 

The class is fairly simple and solely contains three variables:

```python
class Option(object):

    def __init__(self, option):
        self._config_key = option[0]
        self.score = option[1]
        self.params = option[2]
```

### decision_strategy(.py)

Module decision_strategy includes class DecisionStrategy which contains the coordination mechanism selection. 

```python
class DecisionStrategy(object):
    def __init__(self, so_goal, expert_knowledge):
        self.options = self.load_options(so_goal, expert_knowledge)
    
    def select(self):
        [selection mechanism]
        return [config_key, params]
```
The class needs two parameters:
`so_goal` is the self-organization goal and has to match one key used in the provided expert knowledge.
 `expert_knowledge` is the path to the expert knowledge yaml file, e.g. so_expert_knowledge.yaml.

The options available for the DecisionStrategy are loaded first. 
This is done by reading in the expert_knowledge yaml file. 
The options mapped to the specified so_goal will be stored as a list of Option objects in `self.options`.

The stored options are used in method select to determine the most appropriate coordination mechanism. 
Method select should return a configuration key as this will be used in the components factory to create the RHBP components.

The currently implemented selection mechanism checks if only one option is mapped to the self-organization goal. 
In case that yes, it will return the configuration key of this option. 
Otherwise, it will check which option has the highest score and return the configuration key of this option.
Additionally to the configuration key, the select method returns the parameters specified with the option. 
These will be used as optional parameters in SOComponents. 


Components Factory: so_components(.py)
-------------------------------------

Module so_components includes class SOComponents and method `create_from_yaml()`.
 
##### SOComponents
SOComponents includes a factory to create the specified RHBP components and variables to store the created objects. 

```python
class SOComponents(object):
    def __init__(self, specs, id, planner_prefix='', mapping=SO_MAPPING,
                 params=None, optional_params=None, name='')
```

`specs` is a dictionary consisting of datatypes as supported by yaml. 
The dictionary has to have the same form like presented as the value of one configuration key in so_specification.yaml.
`id` is the ID of the robot.
`planner_prefix` specifies the prefix of the planner the created components should be assigned to.
`mapping` indicates the dictionary which contains the mapping from strings to classes (see so_mapping(.py)).
`params` includes the parameters to be replaced by agent specific values when creating the RHBP components as a dictionary, e.g. `{'motion_topic': '\cmd_vel'}`.
`optional_params` is a dictionary of dictionaries which specifies per component (component key from pattern_knowledge yaml) a dictionary with parameters to be adjusted. 
This allows to manually adapt the specifications for a specific scenario. 
`name` is an unique name of the SOComponents instance and helps to create unique names for all components used in one behaviour planner. 
It does not have to be specified if only one configuration is used in one planner. 

Method `create_components()` is a factory which creates the components specified in `specs` using all given parameters.
The created components are stored in dictionaries mapping component_key to object instance. 

Method `delete_components()` allows to clear all dictionaries storing RHBP components. 

##### create_from_yaml

Method `create_from_yaml()` allows to create a SOComponents object by using a specification provided by a yaml file. 

```python 
def create_from_yaml(file_path, id, components_class=SOComponents,
                     planner_prefix='', config_key=None, mapping=SO_MAPPING,
                     params=None, optional_params=None, name=''):
```

Next to the parameters which are required for the SOComponents class, it includes three additional parameters.
`file_path` is the path to the yaml file containing the specifications or configurations of the components. 
In the SoCoordinator, it is file so_specification.yaml.
`components_class` allows to specify the class which will create and store the RHBP components. 
Last but not least, a `config_key` can be specified which indicates the configuration to be created.
In case that no configuration key is specified, it is considered that the yaml file consists of solely one configuration. 

The methods reads in the provided yaml file and creates a SOComponents object instance with the given parameters. 


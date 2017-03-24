SO_COORDINATOR
==============


YAML file
---------

YAML files can be used to map application scenarios of self-organization to the setup of mechanisms in the RHBP.
 
It should have the following structure:

```yamlex
SO_SCENARIO: {
  buffer: {buffer_key: {buffer parameters} },
  mechanisms: {mechanism_key: [so_mechanism_implementation, buffer_key, {mechanism parameters}]},
  activators: {activator_key: [rhbp_activator, {activator parameters}]},
  sensors: {sensor_key: [rhbp_sensor, mechanism_key, {sensor parameters}]}, 
  conditions: {goal_key: [rhbp_condition, sensor_key, activator_key, optional_bool]},
  behaviours: {behaviour_key: [rhbp_behaviour, mechanism_key, {behaviour parameters}]},
  preconditions: {behaviour_key: [preconditions]},
  goals: {goal_key: [rhbp_goal, permanent_bool, [conditions]]}
}
```
"""
Created on 29.03.2017

@author: kaiser

Module containing SOComponents class to create RHBP components
"""

import yaml
import rospy
from so_mapping import SO_MAPPING


class SOComponents(object):
    """
    class which creates RHBP components for self-organization based on a given
    specification (required form see README)
    """

    def __init__(self, specs, id, planner_prefix='', mapping=SO_MAPPING,
                 params=None, optional_params=None):
        """
        initialization of class
        :param specs: specification of RHBP components
        :param id: robot ID
        :param planner_prefix: prefix of used RHBP
        :param mapping: mapping from strings to classes
        :param params: dictionary of parameters which will be inserted when
        specified as param_keys
        :param optional_params: dictionary of parameters to be adjusted for
        each component as required by overall setting (e.g. frame IDs)
        """

        # parameters of SOComponents instance
        self.id = id
        self.mapping = mapping
        self.planner_prefix = planner_prefix

        if params is None:
            self.params = {}
            rospy.logwarn("No agent specific parameters specified in "
                          "SoComponents.")
        else:
            self.params = params

        if optional_params is None:
            self.optional_params = {}
        else:
            self.optional_params = optional_params

        # storage for components
        self.buffer = {}
        self.mechanisms = {}
        self.activators = {}
        self.sensors = {}
        self.conditions = {}
        self.behaviours = {}
        self.goals = {}

        # create components
        self.create_components(specs)

    def create_components(self, specs):
        """
        Method to dynamically create required components for self-organization
        based on given specification
        stores components in dictionaries defined in init
        :param specs: specification of RHBP components
        """

        # create buffer
        bffr = specs.get('buffer')
        for b in bffr.keys():
            # update params with individual keys
            if b in self.optional_params.keys():
                bffr[b].update(self.optional_params[b])

            if 'param_keys' in bffr[b].keys():
                for key in bffr[b]['param_keys']:
                    bffr[b][key] = self.params[key]
                bffr[b].pop('param_keys', None)

            self.buffer[b] = self.mapping.get('SoBuffer')(id=self.id,
                                                          **bffr[b])

        # create mechanisms
        mechanisms = specs.get('mechanisms')
        for m in mechanisms.keys():
            # update params with individual keys
            if m in self.optional_params.keys():
                mechanisms[m][1].update(self.optional_params[m])

            if 'buffer' in mechanisms[m][1].keys():
                mechanisms[m][1]['buffer'] = self.buffer[mechanisms[m][1]
                                                         ['buffer']]

            if 'param_keys' in mechanisms[m][1].keys():
                for key in mechanisms[m][1]['param_keys']:
                    mechanisms[m][1][key] = self.params[key]
                mechanisms[m][1].pop('param_keys', None)

            self.mechanisms[m] = self.mapping.get(mechanisms[m][0])(
                **mechanisms[m][1])

        # create activators
        activators = specs.get('activators')
        for a in activators.keys():
            # update params with individual keys
            if a in self.optional_params.keys():
                activators[a][1].update(self.optional_params[a])

            self.activators[a] = self.mapping.get(activators[a][0])(
                **activators[a][1])

        # create sensors
        sensors = specs.get('sensors')
        for s in sensors.keys():
            # update params with individual keys
            if s in self.optional_params.keys():
                sensors[s][1].update(self.optional_params[s])

            # adjust mechanism parameter
            if 'param_keys' in sensors[s][1].keys():
                for key in sensors[s][1]['param_keys']:
                    sensors[s][1][key] = self.params[key]
                sensors[s][1].pop('param_keys', None)

            if 'mechanism' in sensors[s][1].keys():
                if isinstance(sensors[s][1]['mechanism'], list):
                    sensors[s][1]['mechanism'] = [self.mechanisms[m] for m in
                                                  sensors[s][1]['mechanism']]
                else:
                    sensors[s][1]['mechanism'] = self.mechanisms[sensors[s][1]
                                                                 ['mechanism']]
            # adjust pattern parameter
            if 'pattern' in sensors[s][1].keys():
                sensors[s][1]['pattern'][0] += self.id

            self.sensors[s] = self.mapping.get(sensors[s][0])(
                name=s + self.id + 'sensor', **sensors[s][1])

        # create conditions
        conditions = specs.get('conditions')
        for c in conditions.keys():
            # update params with individual keys
            if c in self.optional_params.keys():
                conditions[c][1].update(self.optional_params[c])

            if 'sensor' in conditions[c][1].keys():
                conditions[c][1]['sensor'] = self.sensors[conditions[c][1]
                                                          ['sensor']]
            # Multi-Sensor-Conditions
            elif 'sensors' in conditions[c][1].keys():
                conditions[c][1]['sensors'] = [self.sensors[s] for s in
                                               conditions[c][1]['sensors']]

            if 'activator' in conditions[c][1].keys():
                conditions[c][1]['activator'] = self.activators[
                    conditions[c][1]['activator']]

            self.conditions[c] = self.mapping.get(conditions[c][0])(
                name=c + self.id + 'condition', **conditions[c][1])

        # create behaviours
        behaviours = specs.get('behaviours')
        for b in behaviours.keys():
            # update params with individual keys
            if b in self.optional_params.keys():
                behaviours[b][1].update(self.optional_params[b])

            # rework conditions
            if 'effects' in behaviours[b][1].keys():
                    sub = []
                    for c in behaviours[b][1]['effects']:
                        sub.append(self.create_effect(c))
                    behaviours[b][1]['effects'] = sub

            # ensure unique value and state keys
            if 'value_key' in behaviours[b][1].keys():
                behaviours[b][1]['value_key'] += self.id
            if 'state_key' in behaviours[b][1].keys():
                behaviours[b][1]['state_key'] += self.id

            if 'mechanism' in behaviours[b][1].keys():
                if isinstance(behaviours[b][1]['mechanism'], list):
                    behaviours[b][1]['mechanism'] = [self.mechanisms[m] for m
                                                     in behaviours[b][1]
                                                     ['mechanism']]
                else:
                    behaviours[b][1]['mechanism'] = self.mechanisms[
                        behaviours[b][1]['mechanism']]

            if 'param_keys' in behaviours[b][1].keys():
                for key in behaviours[b][1]['param_keys']:
                    behaviours[b][1][key] = self.params[key]
                behaviours[b][1].pop('param_keys', None)

            self.behaviours[b] = self.mapping.get(behaviours[b][0])(
                name=b+self.id+'behaviour',
                plannerPrefix=self.planner_prefix,
                **behaviours[b][1])

        # add preconditions
        preconds = specs.get('preconditions')
        for p in preconds.keys():
            for e in preconds[p]:
                self.behaviours[p].addPrecondition(self.create_condition(e))

        # create goals
        goals = specs.get('goals')
        for g in goals.keys():
            # update params with individual keys
            if g in self.optional_params.keys():
                goals[g][1].update(self.optional_params[g])

            if 'conditions' in goals[g][1].keys():
                goals[g][1]['conditions'] = [self.create_condition(c) for c in
                                             goals[g][1]['conditions']]

            self.goals[g] = self.mapping.get(goals[g][0])(
                name=g+self.id+'goal',
                plannerPrefix=self.planner_prefix,
                **goals[g][1])

    def create_condition(self, lst):
        """
        method to create conditions for goals and preconditions
        :param lst: list of conditions, each condition has the form
                    [modifier, condition_key], e.g. [Negation, c_goal]
        :return: condition object
        """

        condition = None

        # simple condition
        if lst[0] == 'None':
            condition = self.conditions[lst[1]]

        # nested conditions
        else:
            # several connected conditions, e.g. Disjunction, Conjunction
            # e.g. [Disjunction, [[None, 'c_goal'], [None, 'c_dist']]]
            if all(isinstance(elem, list) for elem in lst[1]):
                sub = [self.create_condition(l) for l in lst[1]]
                condition = self.mapping.get(lst[0])(*sub)

            # one condition with modifier, e.g. Negation
            # e.g. [Negation, [None, c_goal]]
            elif isinstance(lst[1], list):
                subcond = self.create_condition(lst[1])
                condition = self.mapping.get(lst[0])(subcond)
            # e.g. [Negation, c_goal]
            else:
                condition = self.mapping.get(lst[0])(self.conditions[lst[1]])

        return condition

    def create_effect(self, eff):
        """
        method to create effect list as required to be handed over to
        behaviours
        :return: list [condition, indicator, type]
        """

        # RHBP effects only differentiate between bool and not bool
        if eff[2] == 'bool':
            return [self.conditions[eff[0]], eff[1], bool]
        else:
            return [self.conditions[eff[0]], eff[1], float]

    def delete_components(self):
        """
        method to delete all stored components
        """
        self.buffer.clear()
        self.mechanisms.clear()
        self.activators.clear()
        self.sensors.clear()
        self.conditions.clear()
        self.behaviours.clear()
        self.goals.clear()


def create_from_yaml(file_path, id, components_class=SOComponents,
                     planner_prefix='', config_key=None, mapping=SO_MAPPING,
                     params=None, optional_params=None):
    """
    create SO components from yaml specification
    either hand over yaml file with one specification only or specify config
    key
    :param file_path: path to yaml file
    :param id: id of the robot
    :param components_class: factory to create RHBP components
    :param planner_prefix: prefix of RHBP instance
    :param config_key: key for specification in yaml file
    :param mapping: mapping from strings to classes
    :param params: list of agent specific parameters to be inserted by
                   pattern creation
    :param optional_params: dictionary of parameters to be adjusted for
                            each component as required by overall setting
    :return: components_class instance containing required RHBP components
    """

    # load yaml file
    data = None

    with open(file_path, 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # create components_class instance
    if data:
        if config_key:
            return components_class(data[config_key], id, mapping=mapping,
                                    planner_prefix=planner_prefix,
                                    params=params,
                                    optional_params=optional_params)
        else:
            return components_class(data, id, planner_prefix=planner_prefix,
                                    mapping=mapping, params=params,
                                    optional_params=optional_params)

    else:
        rospy.logerr("SO components creation failed.")

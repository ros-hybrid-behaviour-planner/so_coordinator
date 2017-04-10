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
                 params=None):
        """
        initialization of class
        :param specs: specification of RHBP components
        :param id: robot ID
        :param planner_prefix: prefix of used RHBP
        :param mapping: mapping from strings to classes
        :param motion_topic: topic where twist has to be published to (agent)
        :param pose_frame: message frame ID for position data of agents
        """

        # parameters of SOComponents instance
        self.id = id
        self.params = params
        self.mapping = mapping
        self.planner_prefix = planner_prefix

        # storage for RHBP components
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
        based on specification
        stores components in dictionaries defined in init
        :param specs: specification of RHBP components
        """

        # create buffer
        bffr = specs.get('buffer')
        for b in bffr.keys():

            if 'param_keys' in bffr[b].keys():
                for key in bffr[b]['param_keys']:
                    bffr[b][key] = self.params[key]
                bffr[b].pop('param_keys', None)

            self.buffer[b] = self.mapping.get('SoBuffer')(id=self.id,
                                                          **bffr[b])

        # create mechanisms
        mechanisms = specs.get('mechanisms')
        for m in mechanisms.keys():
            if 'buffer' in mechanisms[m][1].keys():
                mechanisms[m][1]['buffer'] = self.buffer[mechanisms[m][1]
                                                         ['buffer']]

            self.mechanisms[m] = self.mapping.get(mechanisms[m][0])(
                **mechanisms[m][1])

        # create activators
        activators = specs.get('activators')
        for a in activators.keys():
            self.activators[a] = self.mapping.get(activators[a][0])(
                **activators[a][1])

        # create sensors
        sensors = specs.get('sensors')
        for s in sensors.keys():
            # adjust mechanism parameter
            if 'mechanism' in sensors[s][1].keys():
                if isinstance(sensors[s][1], list):
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

            if conditions[c][2]:
                self.conditions[c].optional = True

        # create behaviours
        behaviours = specs.get('behaviours')
        for b in behaviours.keys():
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
        :return: list [condition, +/- 1, type]
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
                     planner_prefix='', pattern_key=None, mapping=SO_MAPPING,
                     params=None):
    """
    create SO components from yaml specification
    either hand over yaml file with one specification only or specify
    so_goal/key
    :param file_path: path to yaml file
    :param id: id of the robot
    :param components_class: factory to create RHBP components
    :param planner_prefix: prefix of RHBP instance
    :param pattern_key: key for specification in yaml file
    :param mapping: mapping from strings to classes
    :param params: list of agent specific parameters to be inserted by
                   pattern creation
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
        if pattern_key:
            return components_class(data[pattern_key], id, mapping=mapping,
                                    planner_prefix=planner_prefix,
                                    params=params)
        else:
            return components_class(data, id, planner_prefix=planner_prefix,
                                    mapping=mapping,
                                    params=params)

    else:
        rospy.logerr("SO components creation failed.")

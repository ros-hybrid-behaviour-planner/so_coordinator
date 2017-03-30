"""
Created on 29.03.2017

@author: kaiser

Module containing SOComponents class to create RHBP components
"""

from so_mapping import SO_MAPPING
import yaml
import rospy


class SOComponents(object):
    """
    class which creates RHBP components for self-organization based on a given
    specification (required form see README)
    """

    def __init__(self, specs, id, planner_prefix='', mapping=SO_MAPPING,
                 motion_topic='', pose_frame='robot'):
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
        self.pose_frame = pose_frame
        self.id = self.pose_frame + str(id)
        self.planner_prefix = planner_prefix
        self.mapping = mapping
        self.motion_topic = motion_topic

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
        :param specs: specification of RHBP components
        """
        # create buffer
        buffer = specs.get('buffer')
        for b in buffer.keys():
            self.buffer[b] = self.mapping.get('SoBuffer')(id=self.id,
                                                          pose_frame=
                                                          self.pose_frame,
                                                          **buffer[b])

        # create mechanisms
        mechanisms = specs.get('mechanisms')
        for m in mechanisms.keys():
            self.mechanisms[m] = self.mapping.get(mechanisms[m][0])(
                self.buffer[mechanisms[m][1]], **mechanisms[m][2])

        # create activator
        activators = specs.get('activators')
        for a in activators.keys():
            self.activators[a] = self.mapping.get(activators[a][0])(
                **activators[a][1])

        # create sensor
        sensors = specs.get('sensors')
        for s in sensors.keys():
            if sensors[s][0] == 'GradientSensor':
                # several mechanisms
                if isinstance(sensors[s][1], list):
                    self.sensors[s] = self.mapping.get(sensors[s][0])(
                        name=s + self.id + 'sensor',
                        mechanism=[self.mechanisms[m] for m in sensors[s][1]],
                        **sensors[s][2])
                # one mechanism
                else:
                    self.sensors[s] = self.mapping.get(sensors[s][0])(
                        s + self.id + 'sensor', self.mechanisms[sensors[s][1]],
                        **sensors[s][2])
            else:
                if 'pattern' in sensors[s][1].keys():
                    sensors[s][1]['pattern'][0] += self.id

                self.sensors[s] = self.mapping.get(sensors[s][0])(
                    name=s + self.id + 'sensor', **sensors[s][1])

        # create condition
        conditions = specs.get('conditions')
        for c in conditions.keys():

            if isinstance(conditions[c][1], list):
                self.conditions[c] = self.mapping.get(conditions[c][0])(
                    [self.sensors[s] for s in conditions[c][1]],
                    self.activators[conditions[c][2]], name=c+self.id+'condition')
            else:
                self.conditions[c] = self.mapping.get(conditions[c][0])(
                    self.sensors[conditions[c][1]],
                    self.activators[conditions[c][2]], name=c+self.id+'condition')
            if conditions[c][3]:
                self.conditions[c].optional = True

        # create behaviours
        behaviours = specs.get('behaviours')
        for b in behaviours.keys():
            # rework conditions
            if 'effects' in behaviours[b][2].keys():
                    sub = []
                    for c in behaviours[b][2]['effects']:
                        sub.append(self.create_effect(c))
                    behaviours[b][2]['effects'] = sub

            # ensure unique value and state keys
            if 'value_key' in behaviours[b][2].keys():
                behaviours[b][2]['value_key'] += self.id
            if 'state_key' in behaviours[b][2].keys():
                behaviours[b][2]['state_key'] += self.id

            # several mechanisms
            if isinstance(behaviours[b][1], list):
                self.behaviours[b] = self.mapping.get(behaviours[b][0])(
                    name=b+self.id+'behaviour',
                    plannerPrefix=self.planner_prefix,
                    motion_topic=self.motion_topic,
                    mechanism=[self.mechanisms[m] for m in behaviours[b][1]],
                    **behaviours[b][2])
            # one mechanism
            else:
                self.behaviours[b] = self.mapping.get(behaviours[b][0])(
                    name=b+self.id+'behaviour',
                    plannerPrefix=self.planner_prefix,
                    motion_topic=self.motion_topic,
                    mechanism=self.mechanisms[behaviours[b][1]],
                    **behaviours[b][2])

        print 'preconditions'
        # add preconditions
        preconds = specs.get('preconditions')
        for p in preconds.keys():
            for e in preconds[p]:
                self.behaviours[p].addPrecondition(self.create_condition(e))

        print 'goals'
        # create goals
        goals = specs.get('goals')
        for g in goals.keys():
            conds = []
            for c in goals[g][2]:
                conds.append(self.create_condition(c))

            self.goals[g] = self.mapping.get(goals[g][0])(
                name=g+self.id+'goal', plannerPrefix=self.planner_prefix,
                permanent=goals[g][1], conditions=conds)

    def create_condition(self, lst):
        """
        method to specify conditions for
        :param lst: list of conditions
        :return: conditions
        """
        print lst
        # solely return condition object
        if lst[0] == 'None':
            condition = self.conditions[lst[1]]
        # return nested conditions (e.g. Negation, Disjunction)
        else:
            if all(isinstance(elem, list) for elem in lst[1]):
                sub = [self.create_condition(l) for l in lst[1]]
                condition = self.mapping.get(lst[0])(*sub)
            elif isinstance(lst[1], list):
                subcond = self.create_condition(lst[1])
                condition = self.mapping.get(lst[0])(subcond)
            else:
                condition = self.mapping.get(lst[0])(self.conditions[lst[1]])

        return condition

    def create_effect(self, eff):
        """
        method to create effect parameter to be handed over to behaviours
        :return: list [condition, +/- 1, type]
        """
        # create condition for effect
        cond = self.create_condition(eff[0])

        # RHBP effects only differentiate between bool and not bool, so this
        # setup should be sufficient
        if eff[2] == 'bool':
            return [cond, eff[1], bool]
        else:
            return [cond, eff[1], float]

    def delete_components(self):
        """
        delete all stored components
        """
        self.buffer.clear()
        self.mechanisms.clear()
        self.activators.clear()
        self.sensors.clear()
        self.conditions.clear()
        self.behaviours.clear()
        self.goals.clear()


def create_from_yaml(file_path, id, components_class=SOComponents,
                     planner_prefix='', so_goal=None, mapping=SO_MAPPING,
                     motion_topic='', pose_frame='robot'):
    """
    create SO components from yaml specification
    either hand over yaml file with one specification only or specify so_goal
    :param file_path: path to yaml file
    :param id: id of the robot
    :param components_class: factory to create RHBP components
    :param planner_prefix: prefix of RHBP instance
    :param so_goal: key for specification in yaml file
    :param mapping: mapping from strings to classes
    :param motion_topic: topic where twist has to be published to (agent)
    :param pose_frame: message frame ID for position data of agents
    :return: components_class instance containing required RHBP components
    """

    # open yaml file
    data = None

    with open(file_path, 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # create components_class instance
    if data:
        if so_goal:
            return components_class(data[so_goal], id, mapping=mapping,
                                    planner_prefix=planner_prefix,
                                    motion_topic=motion_topic,
                                    pose_frame=pose_frame)
        else:
            return components_class(data, id, planner_prefix=planner_prefix,
                                    mapping=mapping, motion_topic=motion_topic,
                                    pose_frame=pose_frame)

    else:
        rospy.logerr("SO components creation failed.")

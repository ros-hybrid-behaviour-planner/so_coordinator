from so_mapping import SO_MAPPING
import yaml
import rospy


class SOComponents(object):
    """
    class which creates SO components based on specification
    """

    def __init__(self, setting, id, planner_prefix='',
                 mapping=SO_MAPPING, motion_topic='', pose_frame='robot'):
        # id
        self.pose_frame = pose_frame
        self.id = self.pose_frame + str(id)
        self.setting = setting
        self.planner_prefix = planner_prefix

        # topics
        self.motion_topic = motion_topic
        self.mapping = mapping

        # store components
        self.buffer = {}
        self.mechanisms = {}
        self.activators = {}
        self.sensors = {}
        self.conditions = {}
        self.behaviours = {}
        self.goals = {}

        # create components
        self.create_components()

    def create_components(self):
        # create buffer
        buffer = self.setting.get('buffer')
        for b in buffer.keys():
            self.buffer[b] = self.mapping.get('SoBuffer')(id=self.id,
                                                          pose_frame=
                                                          self.pose_frame,
                                                          **buffer[b])

        # create mechanisms
        mechanisms = self.setting.get('mechanisms')
        for m in mechanisms.keys():
            self.mechanisms[m] = self.mapping.get(mechanisms[m][0])(
                self.buffer[mechanisms[m][1]], **mechanisms[m][2])

        # create activator
        activators = self.setting.get('activators')
        for a in activators.keys():
            self.activators[a] = self.mapping.get(activators[a][0])(
                **activators[a][1])

        # create sensor
        sensors = self.setting.get('sensors')
        for s in sensors.keys():
            if sensors[s][0] == 'GradientSensor':
                self.sensors[s] = self.mapping.get(sensors[s][0])(s + self.id +
                                                                'sensor',
                                                self.mechanisms[sensors[s][1]],
                                                **sensors[s][2])
            else:
                self.sensors[s] = self.mapping.get(sensors[s][0])(s + self.id +
                                                                'sensor',
                                                 **sensors[s][1])

        # create condition
        conditions = self.setting.get('conditions')
        for c in conditions.keys():
            self.conditions[c] = self.mapping.get(conditions[c][0])(
                self.sensors[conditions[c][1]], self.activators[conditions[c][2]],
                name=c+self.id+'condition')
            if conditions[c][3]:
                self.conditions[c].optional = True

        # create behaviours
        behaviours = self.setting.get('behaviours')
        for b in behaviours.keys():
            # rework conditions
            if 'effects' in behaviours[b][2].keys():
                    sub = []
                    for c in behaviours[b][2]['effects']:
                        sub.append(self.create_effect(c))
                    behaviours[b][2]['effects'] = sub

            self.behaviours[b] = self.mapping.get(behaviours[b][0])(
                name=b+self.id+'behaviour', plannerPrefix=self.planner_prefix,
                motion_topic=self.motion_topic, mechanism=
                self.mechanisms[behaviours[b][1]], **behaviours[b][2])

        # add preconditions
        preconds = self.setting.get('preconditions')
        for p in preconds.keys():
            for e in preconds[p]:
                self.behaviours[p].addPrecondition(self.create_condition(e))

        # create goals
        goals = self.setting.get('goals')
        for g in goals.keys():
            conds = []
            for c in goals[g][2]:
                conds.append(self.create_condition(c))

            self.goals[g] = self.mapping.get(goals[g][0])(name=g+self.id+'goal',
                                        plannerPrefix=self.planner_prefix,
                                        permanent=goals[g][1],
                                        conditions=conds)

    def create_condition(self, lst):
        """
        method to create conditions
        :param lst: list of conditions
        :return: conditions
        """
        if lst[0] == 'None':
            condition = self.conditions[lst[1]]
        else:
            if isinstance(lst[1], list):
                sub = [self.create_condition(l) for l in lst[1]]
                condition = self.mapping.get(lst[0])(*sub)
            else:
                condition = self.mapping.get(lst[0])(self.conditions[lst[1]])

        return condition

    def create_effect(self, eff):
        """
        method to create effects for behaviours
        """
        # create condition
        cond = self.create_condition(eff[0])

        if eff[2] == 'bool':
            return [cond, eff[1], bool]
        elif eff[2] == 'float':
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


def create_from_yaml(file_path, id, components_class=SOComponents, planner_prefix='', so_goal=None,
                 mapping=SO_MAPPING, motion_topic='', pose_frame='robot'):
    """
    create SO components from yaml specification
    :param file_path:
    :return:
    """

    data = None

    with open(file_path, 'r') as stream:
        try:
            data = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # create
    if data:
        if so_goal:
            return components_class(data[so_goal], id,
                                    planner_prefix=planner_prefix, mapping=mapping,
                                    motion_topic=motion_topic, pose_frame=pose_frame)
        else:
            return components_class(data, id, planner_prefix=planner_prefix, mapping=mapping,
                                    motion_topic=motion_topic, pose_frame=pose_frame)

    else:
        rospy.logerr("SO components creation failed.")

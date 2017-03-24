from so_mapping import SO_MAPPING


class SOAgent(object):

    def __init__(self, setting, turtle_number, clock_topic, planner_prefix=''):
        # id
        self.pose_frame = 'robot'
        self.id = self.pose_frame + str(turtle_number)

        # topics
        motion_topic = 'turtle' + str(turtle_number) + '/cmd_vel'

        # CHEMOTAXIS

        # create buffer
        self.buffer = {}
        buffer = setting.get('buffer')
        for b in buffer.keys():
            self.buffer[b] = SO_MAPPING.get('SoBuffer')(id=self.id, pose_frame=self.pose_frame,
                                      **buffer[b])

        # create mechanisms
        self.mechanisms = {}
        mechanisms = setting.get('mechanisms')
        for m in mechanisms.keys():
            self.mechanisms[m] = SO_MAPPING.get(mechanisms[m][0])(
                self.buffer[mechanisms[m][1]], **mechanisms[m][2])

        # create activator
        self.activators = {}
        activators = setting.get('activators')
        for a in activators.keys():
            self.activators[a] = SO_MAPPING.get(activators[a][0])(**activators[a][1])

        # create sensor
        self.sensors = {}
        sensors = setting.get('sensors')
        for s in sensors.keys():
            if sensors[s][0] == 'GradientSensor':
                self.sensors[s] = SO_MAPPING.get(sensors[s][0])(s + self.id + 'sensor',
                                                self.mechanisms[sensors[s][1]],
                                                **sensors[s][2])
            else:
                self.sensors[s] = SO_MAPPING.get(sensors[s][0])(s + self.id + 'sensor',
                                                **sensors[s][1])

        # create condition
        self.conditions = {}
        conditions = setting.get('conditions')
        for c in conditions.keys():
            self.conditions[c] = SO_MAPPING.get(conditions[c][0])(self.sensors[conditions[c][1]],
                                                  self.activators[conditions[c][2]],
                                                  name=c+self.id+'condition')
            if conditions[c][3]:
                self.conditions[c].optional = True

        # create behaviours
        self.behaviours = {}
        behaviours = setting.get('behaviours')
        for b in behaviours.keys():
            # rework conditions
            for p in behaviours[b][2].keys():
                if all(isinstance(e, list) for e in behaviours[b][2][p]):
                    sub = []
                    for c in behaviours[b][2][p]:
                        sub.append(self.create_condition(c))
                    behaviours[b][2][p] = sub
                else:
                    behaviours[b][2][p] = self.create_condition(behaviours[b][2][p])

            self.behaviours[b] = SO_MAPPING.get(behaviours[b][0])(name=b+self.id+'behaviour',
                                                  planner_prefix=planner_prefix,
                                                  motion_topic=motion_topic,
                                                  mechanism=self.mechanisms[behaviours[b][1]],
                                                  **behaviours[b][2])

        # add preconditions
        preconds = setting.get('preconditions')
        for p in preconds.keys():
            for e in preconds[p]:
                self.behaviours[p].addPrecondition(self.create_condition(e))

        # create goals
        self.goals = {}
        goals = setting.get('goals')
        for g in goals.keys():
            conds = []
            for c in goals[g][2]:
               conds.append(self.create_condition(c))

            self.goals[g] = SO_MAPPING.get(goals[g][0])(name=g+self.id+'goal',
                                       plannerPrefix=planner_prefix,
                                       permanent=goals[g][1],
                                       conditions=conds)

    def create_condition(self, lst):
        """
        method to create conditions
        :param list: list of conditions
        :return: conditions
        """
        if lst[0] == 'None':
            condition = self.conditions[lst[1]]
        else:
            if isinstance(lst[1], list):
                sub = [self.create_condition(l) for l in lst[1]]
                condition = SO_MAPPING.get(lst[0])(*sub)
            else:
                condition = SO_MAPPING.get(lst[0])(self.conditions[lst[1]])

        return condition
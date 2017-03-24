from so_data.sobuffer import SoBuffer
from behaviour_components.conditions import Negation
from so_data.chemotaxis import ChemotaxisBalch
from behaviour_components.activators import LinearActivator, BooleanActivator
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from rhbp_selforga.conditions import VectorBoolCondition, GoalBoolCondition, \
    VectorDistCondition
from rhbp_selforga.behaviours import MoveBehaviour
from behaviour_components.goals import GoalBase
from behaviour_components.conditions import Negation


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
            self.buffer[b] = SoBuffer(id=self.id, pose_frame=self.pose_frame,
                                      **buffer[b])

        # create mechanisms
        self.mechanisms = {}
        mechanisms = setting.get('mechanisms')
        for m in mechanisms.keys():
            self.mechanisms[m] = globals()[mechanisms[m][0]](
                self.buffer[mechanisms[m][1]], **mechanisms[m][2])

        # create activator
        self.activators = {}
        activators = setting.get('activators')
        for a in activators.keys():
            self.activators[a] = globals()[activators[a][0]](**activators[a][1])

        # create sensor
        self.sensors = {}
        sensors = setting.get('sensors')
        for s in sensors.keys():
            self.sensors[s] = globals()[sensors[s][0]](s + self.id + 'sensor',
                                            self.mechanisms[sensors[s][1]],
                                            **sensors[s][2])

        # create condition
        self.conditions = {}
        conditions = setting.get('conditions')
        for c in conditions.keys():
            self.conditions[c] = globals()[conditions[c][0]](self.sensors[conditions[c][1]],
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
                if isinstance(behaviours[b][2][p], list):
                    for e in range(0, len(behaviours[b][2][p])):
                        behaviours[b][2][p][e] = self.conditions[behaviours[b][2][p][e]]
                else:
                    behaviours[b][2][p] = self.conditions[behaviours[b][2][p]]

            self.behaviours[b] = globals()[behaviours[b][0]](name=b+self.id+'behaviour',
                                                  planner_prefix=planner_prefix,
                                                  motion_topic=motion_topic,
                                                  mechanism=self.mechanisms[behaviours[b][1]],
                                                  **behaviours[b][2])

        # add preconditions
        preconds = setting.get('preconditions')
        for p in preconds.keys():
            for e in preconds[p]:
                if e[0]=='None':
                    self.behaviours[p].addPrecondition(self.conditions[e[1]])
                else:
                    self.behaviours[p].addPrecondition(globals()[e[0]](self.conditions[e[1]]))

        # create goals
        self.goals = {}
        goals = setting.get('goals')
        for g in goals.keys():
            conds = []
            for c in goals[g][2]:
                if not c[0]:
                    conds.append(self.conditions[c[1]])
                else:
                    if isinstance(c[1], list):
                        subconds = []
                        for s in c[1]:
                            if s[0] == 'None':
                                subconds.append(self.conditions[s[1]])
                            else:
                                subconds.append(s[0](self.conditions[s[1]]))
                                #TODO subconds verwerten
                    else:
                        if c[0] == 'None':
                            conds.append(self.conditions[c[1]])
                        else:
                            conds.append(globals()[c[0]](self.conditions[c[1]]))

            self.goals[g] = globals()[goals[g][0]](name=g+self.id+'goal',
                                       plannerPrefix=planner_prefix,
                                       permanent=goals[g][1],
                                       conditions=conds)

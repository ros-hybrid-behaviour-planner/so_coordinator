
from behaviour_components.goals import GoalBase
from behaviour_components.conditions import Negation
from behaviour_components.activators import LinearActivator, BooleanActivator
from rhbp_selforga.behaviours import MoveBehaviour
from rhbp_selforga.conditions import VectorBoolCondition, GoalBoolCondition, \
    VectorDistCondition
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from so_data.sobuffer import SoBuffer


class SOAgent(object):

    def __init__(self, setting, turtle_number, min_activation, max_activation,
                 min_distance, max_distance, clock_topic, planner_prefix=''):
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
        mechanisms = setting.get('mechanism')
        for mechanism in mechanisms.keys():
            self.mechanisms[mechanism] = mechanisms[mechanism][0](
                self.buffer[mechanisms[mechanism][1]],
                **mechanisms[mechanism][2])

        # attractive gradient within view
        self.goal_sensor = GradientSensor('goalSensor' + self.id, self.mechanisms['chem'],
                                          clock_topic)

        self.bool_activator = BooleanActivator()

        self.goal_condition = VectorBoolCondition(self.goal_sensor,
                                             self.bool_activator,
                                             name= self.id + "GoalCondition")

        # attractive gradient reached
        self.goal_reached_sensor = GradientSensor(
            'goalReachedSensor' + self.id,
            self.mechanisms['chem'], clock_topic,
            sensor_type=SENSOR.GOAL)

        self.goal_reached_condition = GoalBoolCondition(
            self.goal_reached_sensor, self.bool_activator,
            name=self.id + "GoalReachedCondition")

        # distance based activation
        self.distance_sensor = GradientSensor('distanceSensor' + self.id,
                                         self.mechanisms['chem'], clock_topic,
                                         sensor_type=SENSOR.GOAL)

        self.distance_activator = LinearActivator(min_distance,
                                             max_distance,
                                             min_activation,
                                             max_activation, self.id +
                                             "DistanceActivator")

        self.distance_condition = VectorDistCondition(self.distance_sensor,
                                                 self.distance_activator,
                                                 name=self.id +
                                                      "DistanceCondition")

        self.distance_condition.optional = True

        # Behaviour
        gradient_behaviour = MoveBehaviour(mechanism=self.mechanisms['chem'],
                                            distance_condition=
                                            self.distance_condition,
                                            bool_p_condition=[
                                                self.goal_reached_condition],
                                            motion_topic=motion_topic,
                                            name='chemotaxisBehaviour' +
                                                 self.id,
                                           plannerPrefix=planner_prefix)

        # add preconditions
        gradient_behaviour.addPrecondition(Negation(
            self.goal_reached_condition))
        gradient_behaviour.addPrecondition(self.distance_condition)
        gradient_behaviour.addPrecondition(self.goal_condition)


        # Goal: reach attractive gradient
        self.goal = GoalBase("goal" + self.id, permanent=False,
                             conditions=[self.goal_reached_condition],
                             plannerPrefix=planner_prefix)

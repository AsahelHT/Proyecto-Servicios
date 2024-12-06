import rospy
import smach_ros
import math
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist

# Constantes
TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'
TOPIC_COLOR = '/color_detected'
TOPIC_COMMAND = '/robot_command'

ANG_IZQ = 30 * math.pi / 180.0
ANG_DER = -ANG_IZQ

# Estado: Patrullar
class Patrol(State):
    def __init__(self, waypoints):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.waypoints = waypoints
        self.current_wp = 0

    def execute(self, userdata):
        rospy.loginfo("Ejecutando estado Patrol")

        while not rospy.is_shutdown():
            goal_pose = MoveBaseGoal()
            wp = self.waypoints[self.current_wp]
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose.position.x = wp[1][0]
            goal_pose.target_pose.pose.position.y = wp[1][1]
            goal_pose.target_pose.pose.orientation.w = wp[2][3]

            client = smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=goal_pose)
            outcome = client.execute(userdata)

            if outcome == 'succeeded':
                self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            else:
                return outcome

        return 'succeeded'

# Estado: WanderAndDetect
class WanderAndDetect(State):
    def __init__(self):
        State.__init__(self, outcomes=['color_detected', 'preempted'])
        self.color_detected = False
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)

    def execute(self, userdata):
        rospy.loginfo("Ejecutando estado WanderAndDetect")

        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        self.subColor = rospy.Subscriber(TOPIC_COLOR, Int32, self.color_detected_callback)
        rate = rospy.Rate(10)

        while not self.color_detected:
            if self.preempt_requested():
                self.subScan.unregister()
                self.subColor.unregister()
                self.service_preempt()
                return 'preempted'
            rate.sleep()

        self.subScan.unregister()
        self.subColor.unregister()
        return "color_detected"

    def laser_callback(self, msg):
        pos_izq = int((ANG_IZQ - msg.angle_min) / msg.angle_increment)
        pos_der = int((ANG_DER - msg.angle_min) / msg.angle_increment)

        cmd = Twist()
        # Rellena cmd con valores calculados basados en los datos del escáner láser
        self.pub.publish(cmd)

    def color_detected_callback(self, msg):
        self.color_detected = True

# Callback para cambiar de estado
current_state = None

def command_callback(msg):
    global current_state, sm
    rospy.loginfo(f"Comando recibido: {msg.data}")

    if msg.data == "start_patrol":
        sm.set_initial_state(['Patrol'])
    elif msg.data == "start_wander":
        sm.set_initial_state(['WanderAndDetect'])
    elif msg.data == "stop":
        sm.request_preempt()

if __name__ == '__main__':
    rospy.init_node("state_machine_robot")

    waypoints = [
        ['one', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)],
        ['two', (6.5, 4.43), (0.0, 0.0, -0.984, 0.178)]
    ]

    sm = StateMachine(outcomes=['end'])
    with sm:
        StateMachine.add('Patrol', Patrol(waypoints), transitions={'succeeded': 'Patrol', 'preempted': 'end', 'aborted': 'end'})
        StateMachine.add('WanderAndDetect', WanderAndDetect(), transitions={'color_detected': 'end', 'preempted': 'end'})

    rospy.Subscriber(TOPIC_COMMAND, String, command_callback)

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()

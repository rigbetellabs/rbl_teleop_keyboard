#!/usr/bin/env python

from __future__ import print_function

import threading
import roslib
roslib.load_manifest('rbl_teleop_keyboard')
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
import sys
from select import select
from std_msgs.msg import Int32
from actionlib_msgs.msg import GoalID
import std_srvs.srv

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

1 : store pose 1    ! : navigate to stored pose 1
2 : store pose 2    @ : navigate to stored pose 2
3 : store pose 3    # : navigate to stored pose 3

H : Return Home
Tab : clear costmaps
Enter : cancel current goal

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            self.publisher.publish(twist_msg)

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def go_home():
    # Define the home pose
    home_pose = PoseStamped()
    home_pose.header.stamp = rospy.Time.now()
    home_pose.header.frame_id = "map"
    home_pose.pose.position.x = 0.0
    home_pose.pose.position.y = 0.0
    home_pose.pose.position.z = 0.0
    home_pose.pose.orientation.x = 0.0
    home_pose.pose.orientation.y = 0.0
    home_pose.pose.orientation.z = 0.0
    home_pose.pose.orientation.w = 1.0
    
    # Publish the home pose
    pose_pub.publish(home_pose)
    rospy.loginfo("Going home")
    
def store_pose(tf_listener, pose, pose_name):
    try:
        now = rospy.Time.now()
        tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform("map", "base_link", now)
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        rospy.loginfo(f"{pose_name} pose stored")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Failed to store {pose_name}: {e}")

if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node('rbl_teleop_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 5.0)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped()

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    pose_1 = PoseStamped()
    pose_2 = PoseStamped()
    pose_3 = PoseStamped()

    tf_listener = tf.TransformListener()

    pose_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    goal_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
    goal_status_pub = rospy.Publisher('robot/nav_status', Int32, queue_size=1)

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while not rospy.is_shutdown():
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                speed = min(speed, speed_limit)
                turn = min(turn, turn_limit)

                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == '1':
                store_pose(tf_listener, pose_1, "Pose 1")
                goal_status_pub.publish(Int32(data=5))
            elif key == '2':
                store_pose(tf_listener, pose_2, "Pose 2")
                goal_status_pub.publish(Int32(data=5))
            elif key == '3':
                store_pose(tf_listener, pose_3, "Pose 3")
                goal_status_pub.publish(Int32(data=5))
            elif key == '!':
                pose_pub.publish(pose_1)
                rospy.loginfo("Navigating to Pose 1")
            elif key == '@':
                pose_pub.publish(pose_2)
                rospy.loginfo("Navigating to Pose 2")
            elif key == '#':
                pose_pub.publish(pose_3)
                rospy.loginfo("Navigating to Pose 3")
            elif key == 'H':
                go_home()
            elif key == '\t':
                rospy.wait_for_service('move_base/clear_costmaps')
                clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)
                clear_costmaps()
                rospy.loginfo("Costmaps cleared")
                goal_status_pub.publish(Int32(data=4))
            elif key == '\r':
                goal_cancel_pub.publish(GoalID())
                rospy.loginfo("Current goal canceled")
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

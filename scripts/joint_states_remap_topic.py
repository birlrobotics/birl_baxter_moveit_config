#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState

goal_pos = 0;
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

def remap_callback(msg):
    pub.publish(msg)

def control():
    rospy.init_node('joint_control', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, remap_callback)
    rospy.spin()
if __name__ == "__main__":
    try:
        rospy.loginfo("..remaping")
        control()   
    except KeyboardInterrupt:
        raise


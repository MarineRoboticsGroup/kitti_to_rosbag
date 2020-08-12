import rospy
import roslib
import sensor_msgs.msg
from std_msgs.msg import Header
import numpy as np
import tf
import sys

MIN_RANGE = 0.1
MAX_RANGE = 10

if __name__ == '__main__':
    """
    inputs
    - nbRobots
    
    python addRangeMeasurements.py 4

    This file is intended to add range measurements to an already split 
    KITTY bag 

    """

    if len(sys.argv) != 2:            # print

        print("\nRequires 1 arguments")
        print("(1) Number of robots ")
        print('\n\n')
        assert(len(sys.argv) == 2)

    rospy.init_node('range_creator')
    nbRobots = int(sys.argv[1])

    listener = tf.TransformListener()

    rangePublisher = rospy.Publisher('range_measurement', sensor_msgs.msg.Range,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for i in range(nbRobots):
            for j in range(nbRobots):
                if i != j:
                    try:
                        (transA,rotA) = listener.lookupTransform('world', 'imu_'+str(i), rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                    try:
                        (transB, rotB) = listener.lookupTransform('world', 'imu_'+str(j), rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    msg = sensor_msgs.msg.Range()
                    head =  Header()
                    head.stamp = rospy.Time.now()
                    head.frame_id = "imu_"+str(i)
                    msg.header = head
                    msg.radiation_type = j
                    msg.min_range = MIN_RANGE
                    msg.max_range = MAX_RANGE
                    msg.range = np.linalg.norm(np.array(transB) - np.array(transA))
                    
                    rangePublisher.publish(msg)

        rate.sleep()

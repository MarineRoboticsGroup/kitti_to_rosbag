import sys
import os

import rosbag
import rospy



if __name__ == '__main__':
    """
    inputs
    - path to file
    - 'n' number of robots
    - 't_overlap' amount of overlap between robots that are adjacent in time

    This file is intended to split single KITTI rosbags
    into a format suitable for multi-robot testing. The
    rosbag will be split such that it will be of shorter
    length and have multiple sections which overlap by a
    specified amount.

    If the original length in time is 't_orig' the new
    time will be t_orig / n + t_overlap
    """

    if len(sys.argv) != 4:            # print

        print("\nRequires 3 arguments")
        print("(1) Path to .bag file")
        print("(2) Number of robots")
        print("(3) Amount of overlap between adjacent robots (in seconds)")
        print('\n\n')
        assert(len(sys.argv) == 4)

    bag_path = sys.argv[1]
    num_sections = int(sys.argv[2])
    overlap = float(sys.argv[3])
    file_name = os.path.basename(bag_path)
    write_name = bag_path[:bag_path.find('.bag')]+"_%d_sections.bag"%(num_sections)

    assert('kitti' in file_name and '.bag' in file_name)
    assert(type(num_sections) is int and type(overlap) is float)

    read_bag = rosbag.Bag(bag_path)
    start_time = read_bag.get_start_time()
    end_time = read_bag.get_end_time()
    runtime = end_time-start_time

    split_length = runtime/num_sections
    section_length = split_length + overlap

    cur_section_start = start_time
    cur_section_id = 0

    write_bag = rosbag.Bag(write_name, 'w')
    while(cur_section_start+overlap < end_time):
        print("Section:", cur_section_id)

        # Manage so all sections will have approx equal length and number of messages
        # Means will be more overlap between last two sections
        if cur_section_id == num_sections-1:
            ros_time_start = rospy.Time.from_seconds(cur_section_start-overlap)
        else:
            ros_time_start = rospy.Time.from_seconds(cur_section_start)

        ros_time_end = rospy.Time.from_seconds(cur_section_start+section_length)
        for topic, msg, t in read_bag.read_messages(start_time=ros_time_start, end_time = ros_time_end):
            t_offset = ros_time_start - rospy.Time.from_seconds(start_time)
            new_t = t - t_offset

            # modify /tf messages so there are several tf trees
            if(topic == '/tf'):
                for tf in msg.transforms:
                    # dont want to modify world frame
                    if(tf.header.frame_id != 'world'):
                        tf.header.frame_id = tf.header.frame_id + "_%d"%(cur_section_id)
                    tf.child_frame_id = tf.child_frame_id + "_%d"%(cur_section_id)
                # print(msg.transforms)
            else:
                topic = topic+"_%d"%(cur_section_id)
                if (msg.header.frame_id != 'world'):
                    msg.header.frame_id = msg.header.frame_id + "_%d"%(cur_section_id)
            write_bag.write(topic, msg, new_t)
        cur_section_start += split_length
        cur_section_id += 1

    write_bag.close()
    print("Done splitting rosbag")


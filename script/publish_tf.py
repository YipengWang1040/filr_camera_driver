#!/usr/bin/env python
import rospy
import sys
import os
import tf
import tf2_ros
import geometry_msgs.msg
import yaml

if __name__ == '__main__':
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("config_file", help="Path to yaml file containing " +\
                                             "camera transform data",
                                        default=os.path.dirname(__file__)+'/camera_tf.yaml',
                                        nargs='?')
    args = arg_parser.parse_args()
    config_file = args.config_file
    
    with open(config_file, "r") as file_handle:
        tf_data = yaml.load(file_handle,Loader=yaml.SafeLoader)
                                        
    rospy.init_node('camera_tf_publisher_node')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "velo_link"
    static_transformStamped.child_frame_id = "camera_color"

    static_transformStamped.transform.translation.x = float(tf_data['x'])
    static_transformStamped.transform.translation.y = float(tf_data['y'])
    static_transformStamped.transform.translation.z = float(tf_data['z'])

    quat = tf.transformations.quaternion_from_euler(
                   float(tf_data['roll']),float(tf_data['pitch']),float(tf_data['yaw']))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()

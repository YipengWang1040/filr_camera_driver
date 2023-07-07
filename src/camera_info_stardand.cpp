#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "camerainfo_pub");
  ros::NodeHandle n;
  ros::Publisher camera_info = n.advertise<sensor_msgs::CameraInfo>("/blackfly/cam0/camera_info", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    sensor_msgs::CameraInfo cam_inf;

    cam_inf.header.frame_id = "image0";
    cam_inf.header.stamp = ros::Time::now();
    cam_inf.height =  1080;
    cam_inf.width =  1440;
    cam_inf.distortion_model = "plumb_bob";

    // The intrinsics and the camera parameters below, have been obtained by using Matlab standard model under
    // 3 polynomial estimation and skew enabled. This camera is fish-eye, so probably these parameters may not
    // be the ideal intrinsics for the camera.
   
    cam_inf.K[0]= 529.5179;
    cam_inf.K[1]= 1.7018;
    cam_inf.K[2]= 725.4780;
    cam_inf.K[3]= 0;
    cam_inf.K[4]= 527.9958;
    cam_inf.K[5]= 561.1358;
    cam_inf.K[6]= 0;
    cam_inf.K[7]= 0;
    cam_inf.K[8]= 1;

    cam_inf.D.push_back(-0.2530);
    cam_inf.D.push_back(0.0589);
    cam_inf.D.push_back(0.0014);
    cam_inf.D.push_back(0.0011);
    cam_inf.D.push_back(-0.0056);

    cam_inf.R[0]= 1;
    cam_inf.R[1]= 0;
    cam_inf.R[2]= 0;
    cam_inf.R[3]= 0;
    cam_inf.R[4]= 1;
    cam_inf.R[5]= 0;
    cam_inf.R[6]= 0;
    cam_inf.R[7]= 0;
    cam_inf.R[8]= 1; 

    cam_inf.P[0]= 529.5179;
    cam_inf.P[1]= 1.7018;
    cam_inf.P[2]= 725.4780;
    cam_inf.P[3]= 0;
    cam_inf.P[4]= 0;
    cam_inf.P[5]= 527.9958;
    cam_inf.P[6]= 561.1358;
    cam_inf.P[7]= 0;
    cam_inf.P[6]= 0;
    cam_inf.P[7]= 0;
    cam_inf.P[8]= 1;
    cam_inf.P[9]= 0;

    camera_info.publish(cam_inf);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

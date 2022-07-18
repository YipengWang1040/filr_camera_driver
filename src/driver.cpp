#include <iostream>
#include <csignal>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "device.h"
#include "config.h"

//#include "../../devel/include/flir_camera_driver/ImageAddition.h"   // incase where IDE cannot correctly find the generated message
#include "flir_camera_driver/ImageAddition.h"

using namespace std;

static int skip=0;
static string topic="/blackfly/cam0/image_raw";
static string topic_addition="/blackfly/additional";
static int64_t delta=-1;

static bool run=true;

void sigint_handler(int){
    if(run){
        ROS_INFO("SIGINT recieved. Exiting....");
        run=false;
    }
}

void apply_configs(Device& camera, configure::config_reader& reader){
    cout<<reader.get_camera_configs().size()<<endl;
    for(auto&& config:reader.get_camera_configs()){
        if(!camera.configure(config)){
            ROS_FATAL("Failed to config camera at %s",config.field.c_str());
        }
    }

    skip=reader.get("skip",0);
    topic=reader.get("ros_topic",string("/blackfly/cam0/image_raw"));
    topic_addition=reader.get("additional_topic",string("/blackfly/additional"));
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "flir_driver");
    ros::NodeHandle nh("~");
    ros::Publisher pub_image=nh.advertise<sensor_msgs::Image>(topic,1000);
    ros::Publisher pub_addition=nh.advertise<flir_camera_driver::ImageAddition>(topic_addition,1000);

    signal(SIGINT,sigint_handler);

    Spinnaker::SystemPtr system=Spinnaker::System::GetInstance();
    auto list=Device::enumerate();

    Device camera=std::move(*list.begin());
    if(!camera.default_initialization()){
        ROS_FATAL("Failed to initialze the camera!");
        return -1;
    }

    string config_file=nh.param("config_file",string("src/myflir/config/default.yaml"));
    configure::config_reader reader(config_file);
    apply_configs(camera,reader);

    ROS_INFO("starting");
    int counter=0;
    while(run){
        cv::Mat image;
        size_t time_stamp;
        double exposure_time, gain;
        if(camera.grab(image,time_stamp,exposure_time,gain)){
            // roughly synchronize the device timestamp (from 0 when connected)
            // with the unix time
            if(delta<0)
                delta=time(nullptr)*1000000000-int64_t(time_stamp);

            // skip publishing if 'skip' was set
            // this could be used in calibration where lower framerate is sufficient
            counter+=1;
            if(counter<=skip)
                continue;
            counter=0;

            int64_t timestamp=delta+int64_t(time_stamp);
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp.fromNSec(time_stamp);
            cv_image.header.frame_id="image0";
            cv_image.encoding="bgr8";
            cv_image.image=image;

            flir_camera_driver::ImageAddition addition;
            addition.header=cv_image.header;
            addition.exposure=exposure_time;
            addition.gain=gain;

            pub_image.publish(cv_image.toImageMsg());
            pub_addition.publish(addition);
            usleep(10000);
        }
        else{
            usleep(100000);
        }
    }

    camera.clear();
    system->ReleaseInstance();
    return 0;
}

#include <iostream>
#include <sstream>
#include <numeric>
#include <algorithm>

#include "device.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

Device::~Device(){
    clear();
}

std::list<Device> Device::enumerate(){
    std::list<Device> ret;

    SystemPtr system = System::GetInstance();

    CameraList camera_list = system->GetCameras();
    if(camera_list.GetSize()==0){
        // cleanup
        ROS_FATAL("No camera detected!");
        camera_list.Clear();
        system->ReleaseInstance();
        return ret;
    }

    for(uint32_t i=0;i<camera_list.GetSize();++i){
        CameraPtr cam_ptr=camera_list.GetByIndex(i);
        cam_ptr->Init();
        ROS_INFO("found device id=%s",cam_ptr->DeviceID.ToString().c_str());
        ret.push_back(Device(cam_ptr));
    }


    camera_list.Clear();
    return ret;
}

void Device::clear(){
    if(is_valid()){
        camera->EndAcquisition();
        camera->DeInit();
        camera=nullptr;
    }
}

bool Device::grab(cv::Mat &image_raw, cv::Mat &image_color, size_t &time_stamp, double& exposure_time, double& gain){
    thread_local int64_t delta=-1;
    if(!is_valid())
        return false;

    try {
        image_raw=cv::Mat();
        image_color=cv::Mat();

        INodeMap& ptr=camera->GetNodeMap();
        ImagePtr grab_image = camera->GetNextImage(500); // note that timeout is in milliseconds
        if(grab_image->IsIncomplete()){
            ROS_FATAL("incomplete image recieved!");
            return false;
        }
        time_stamp=grab_image->GetTimeStamp();
        // roughly sychronize the timestamp form the camera to the host.
        if(delta<0){
            delta=time(nullptr)*1000000000-int64_t(time_stamp);
        }
        time_stamp=delta+time_stamp;

        // read gain and exposure time for this specific frame
        {
            CFloatPtr eptr=ptr.GetNode("ExposureTime");
            CFloatPtr gptr=ptr.GetNode("Gain");
            exposure_time=eptr->GetValue(true,true)/1000000;
            gain=gptr->GetValue(true,true);
            ROS_INFO("%f,%f",exposure_time,gain);
        }

        if(use_rgb_image){
            ImagePtr converted_image=image_processor.Convert(grab_image,PixelFormat_BGR8); 
            image_color=cv::Mat(int(converted_image->GetHeight()),int(converted_image->GetWidth()),CV_8UC3);
            memcpy(image_color.data,converted_image->GetData(),size_t(image_color.cols*image_color.rows*3));
        }
        if(use_raw_image){
            uchar* raw_data = reinterpret_cast<uchar*>(grab_image->GetData());
            int width=int(grab_image->GetWidth());
            int height=int(int(grab_image->GetHeight()));
            if(bayer2rgb){
                image_raw=cv::Mat(height,width,CV_8UC3);
                uchar* dst_data=image_raw.data;

                // only copy valid channel from RGGB to BGR
                // branch elimination trick: i%2+j%2-> 0:r, 1: g, 2: b
                for(int i=0;i<height;++i){
                    for(int j=0;j<width;++j){
                        int offset=2-i%2-j%2;
                        dst_data[width*i*3 + j*3 + offset]=raw_data[width*i+j];
                    }
                }
            }
            else{
                // use green as gray and interpolate other pixels

                // branch elimination trick: i%2+j%2-> 0: r, 1: g, 2: b
                // is_green: (i%2+j%2)%2
                
                image_raw=cv::Mat(height,width,CV_8UC1);
                uchar* dst_data=image_raw.data;
                for(int i=1;i<height-1;++i){
                    for(int j=1;j<width-1;++j){
                        int is_green=(i%2+j%2)%2;
                        int neighbors[]={raw_data[width*i+j-1],raw_data[width*i+j+1],raw_data[width*(i-1)+j],raw_data[width*(i+1)+j]};
                        int avg_neighbors=(std::accumulate(neighbors, neighbors+4, 0))/4;
                        std::sort(neighbors,neighbors+4,[avg_neighbors](auto&& a, auto&& b){return abs(avg_neighbors-a)<abs(avg_neighbors-b);});
                        dst_data[width*i+j]=raw_data[width*i+j]*is_green+(neighbors[0]+neighbors[1])/2*(1-is_green);
                    }
                }
            }
        }

        grab_image->Release();

    } catch (Spinnaker::Exception& e) {
        ROS_FATAL("Error: %s",e.what());
        return false;
    }
    return true;
}


#define CHECK_WRITABLE(ptr) \
    if(!IsAvailable(ptr) ||!IsWritable(ptr)) \
        return false

#define CHECK_READABLE(ptr) \
    if(!IsAvailable(ptr) ||!IsReadable(ptr)) \
        return false

bool Device::default_initialization(){
    if(!is_valid())
        return false;

    std::list<configure::config> configs={
        {"exposure_target_value",50.0},
        {"exposure_time_min",100l},
        {"exposure_time_max",15000l},
        {"exposure_gain_min",0.0},
        {"exposure_gain_max",5.0},
        {"fps",30l},
    };
    try {
        INodeMap& node_map=camera->GetNodeMap();
        // set 'AcquisitionMode' mode to 'Continuous'
        {
            CEnumerationPtr ptr=node_map.GetNode("AcquisitionMode");
            CHECK_WRITABLE(ptr);
            ptr->SetIntValue(Spinnaker::AutoExposureTargetGreyValueAuto_Off);
            CEnumEntryPtr ptr_acquision = ptr->GetEntryByName("Continuous");
            CHECK_READABLE(ptr_acquision);
            ptr->SetIntValue(ptr_acquision->GetValue());
            ROS_INFO("AcquisitionMode switched to Continuous");
        }

        // and then configure a set of default configurations
        for(auto&& config:configs){
            if(!configure(config)){
                cout<<config.field<<endl;
                return false;
            }
        }

        // start streaming
        camera->BeginAcquisition();

    } catch (Spinnaker::Exception& e) {
        ROS_FATAL("Exception: %s",e.what());
        return false;
    }
    return true;
}


bool Device::configure(const configure::config &config){
    if(!is_valid())
        return false;

    try {
        INodeMap& node_map=camera->GetNodeMap();
        if(config.field=="exposure_target_value"){
            // set exposure target grey value auto to 'off'
            {
                CEnumerationPtr ptr=node_map.GetNode("AutoExposureTargetGreyValueAuto");
                CHECK_WRITABLE(ptr);
                ptr->SetIntValue(Spinnaker::AutoExposureTargetGreyValueAuto_Off);
            }
            // set exposure target grey value
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureTargetGreyValue");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64f);
            }
            // read exposure target grey value back
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureTargetGreyValue");
                CHECK_READABLE(ptr);
                double val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %f","AutoExposureTargetGreyValue",val);
            }
        }
        else if(config.field=="exposure_time_min"){
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureExposureTimeLowerLimit");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64i);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureExposureTimeLowerLimit");
                CHECK_READABLE(ptr);
                double val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %f","AutoExposureExposureTimeLowerLimit",val);
            }
        }
        else if(config.field=="exposure_time_max"){
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureExposureTimeUpperLimit");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64i);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureExposureTimeUpperLimit");
                CHECK_READABLE(ptr);
                double val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %f","AutoExposureExposureTimeUpperLimit",val);
            }
        }
        else if(config.field=="exposure_gain_min"){
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureGainLowerLimit");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64f);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureGainLowerLimit");
                CHECK_READABLE(ptr);
                double val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %f","AutoExposureGainLowerLimit",val);
            }
        }
        else if(config.field=="exposure_gain_max"){
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureGainUpperLimit");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64f);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AutoExposureGainUpperLimit");
                CHECK_READABLE(ptr);
                double val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %f","AutoExposureGainUpperLimit",val);
            }
        }
        else if(config.field=="fps"){
            {
                CBooleanPtr ptr=node_map.GetNode("AcquisitionFrameRateEnable");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(true);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AcquisitionFrameRate");
                CHECK_WRITABLE(ptr);
                ptr->SetValue(config.val_64i);
            }
            {
                CFloatPtr ptr=node_map.GetNode("AcquisitionFrameRate");
                CHECK_READABLE(ptr);
                int64_t val=ptr->GetValue();
                ROS_INFO("Field '%s' was set to %ld","AcquisitionFrameRate",val);
            }
        }
        else if(config.field=="mode"){
            use_raw_image=config.val_64i & 0b10;
            use_rgb_image=config.val_64i & 0b01;
        }
        else if(config.field=="bayer_as_rgb"){
            bayer2rgb=config.val_64i;
        }
    } catch (Spinnaker::Exception& e) {
        ROS_FATAL("Exception: %s",e.what());
        return false;
    }
    return true;
}

#include <fstream>
bool Device::photometric_calibration(const std::string& file_name){
    std::ofstream out(file_name);
    INodeMap& node_map=camera->GetNodeMap();

    // disable auto exposure
    {
        CBooleanPtr ptr=node_map.GetNode("ExposureAuto");
        CHECK_WRITABLE(ptr);
        ptr->SetValue(false);
    }
    // disable auto gain
    {
        CBooleanPtr ptr=node_map.GetNode("GainAuto");
        CHECK_WRITABLE(ptr);
        ptr->SetValue(false);
    }
    // set gain to 0
    {
        CFloatPtr ptr=node_map.GetNode("Gain");
        CHECK_WRITABLE(ptr);
        ptr->SetValue(0);
    }






    return true;
}















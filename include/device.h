#ifndef DEVICE_H
#define DEVICE_H

#include <memory>
#include <list>

#include <opencv2/opencv.hpp>

#include "Spinnaker.h"
#include "config.h"

class Device{

public:
    Device(Spinnaker::CameraPtr _camera){
        camera=_camera;
    }
    Device(const Device&)=delete;
    Device(Device&& another){
        camera=another.camera;
        another.camera=nullptr;
    }
    ~Device();

    static std::list<Device> enumerate();

    // destroy the underlying camera object
    void clear();

    bool is_valid(){
        return camera.IsValid();
    }

    bool configure(const configure::config& config);

    // time_stamp is in nanoseconds
    // time_stamp represents the running time after camera was powered on
    bool grab(cv::Mat& image, size_t& time_stamp, double& exposure_time, double& gain);

    // default parameters
    bool default_initialization();


private:
    Spinnaker::CameraPtr camera;

};





#endif //DEVICE_H

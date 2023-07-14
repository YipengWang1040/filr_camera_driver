#ifndef DEVICE_H
#define DEVICE_H

#include <memory>
#include <list>

#include <opencv2/opencv.hpp>

#include "Spinnaker.h"
#include "config.h"

class Device{

public:
    Device(Spinnaker::CameraPtr _camera):timestamp_delta(0),use_raw_image(false), use_rgb_image(true), bayer2rgb(true){
        camera=_camera;
        image_processor.SetColorProcessing(Spinnaker::ColorProcessingAlgorithm::HQ_LINEAR);
    }
    Device(const Device&)=delete;
    Device(Device&& another){
        camera=another.camera;
        use_raw_image=another.use_raw_image;
        use_rgb_image=another.use_rgb_image;
        another.camera=nullptr;
    }
    ~Device();

    static std::list<Device> enumerate();

    // destroy the underlying camera object
    void clear();

    bool is_valid() const{
        return camera.IsValid();
    }

    std::string ID() const{
        if(is_valid())
            return camera->DeviceID.ToString().c_str();
        else
            return "-1";
    }

    bool configure(const configure::config& config);

    // time_stamp is in nanoseconds
    // time_stamp represents the running time after camera was powered on
    bool grab(cv::Mat& image_raw, cv::Mat& image_color, size_t& time_stamp, double& exposure_time, double& gain);

    // default parameters
    bool default_initialization();

    bool photometric_calibration(const std::string&);

    void calibrate_timestamp(size_t samples);

private:
    Spinnaker::CameraPtr camera;
    Spinnaker::ImageProcessor image_processor;

    int64_t timestamp_delta;

    bool use_raw_image;
    bool use_rgb_image;
    
    bool bayer2rgb;

};





#endif //DEVICE_H

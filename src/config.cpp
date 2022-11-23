#include "config.h"

void configure::config_reader::read_config(const std::string &file_name){
    if(!fs.open(file_name,cv::FileStorage::READ))
        return;
    if(!fs["exposure_target_value"].empty()){
        camera_configs.push_back(config{"exposure_target_value",double(fs["exposure_target_value"])});
    }
    if(!fs["exposure_time_min"].empty()){
        camera_configs.push_back(config{"exposure_time_min",int64_t(int(fs["exposure_time_min"]))});
    }
    if(!fs["exposure_time_max"].empty()){
        camera_configs.push_back(config{"exposure_time_max",int64_t(int(fs["exposure_time_max"]))});
    }
    if(!fs["exposure_gain_min"].empty()){
        camera_configs.push_back(config{"exposure_gain_min",double(fs["exposure_gain_min"])});
    }
    if(!fs["exposure_gain_max"].empty()){
        camera_configs.push_back(config{"exposure_gain_max",double(fs["exposure_gain_max"])});
    }
    if(!fs["fps"].empty()){
        camera_configs.push_back(config{"fps",int64_t(int(fs["fps"]))});
    }
    if(!fs["mode"].empty()){
        camera_configs.push_back(config{"mode",int64_t(int(fs["mode"]))});
    }
    if(!fs["bayer_as_rgb"].empty()){
        camera_configs.push_back(config{"bayer_as_rgb",int64_t(int(fs["bayer_as_rgb"]))});
    }
}


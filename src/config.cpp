#include "config.h"

void configure::config_reader::read_config(const std::string &file_name){
    if(!fs.open(file_name,cv::FileStorage::READ))
        return;
    if(!fs["exposure_target_value"].empty()){
        camera_configs.push_back(config{"exposure_target_value",{.val_64f=fs["exposure_target_value"]}});
    }
    if(!fs["exposure_time_min"].empty()){
        camera_configs.push_back(config{"exposure_time_min",{.val_64i=int(fs["exposure_time_min"])}});
    }
    if(!fs["exposure_time_max"].empty()){
        camera_configs.push_back(config{"exposure_time_max",{.val_64i=int(fs["exposure_time_max"])}});
    }
    if(!fs["exposure_gain_min"].empty()){
        camera_configs.push_back(config{"exposure_gain_min",{.val_64f=fs["exposure_gain_min"]}});
    }
    if(!fs["exposure_gain_max"].empty()){
        camera_configs.push_back(config{"exposure_gain_max",{.val_64f=fs["exposure_gain_max"]}});
    }
    if(!fs["fps"].empty()){
        camera_configs.push_back(config{"fps",{.val_64i=int(fs["fps"])}});
    }
}

#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace configure {

struct config{
    std::string field;
    union{
        int64_t val_64i;
        double val_64f;
    };
};

class config_reader{
public:
    config_reader()=default;
    config_reader(const std::string& file_name){
        read_config(file_name);
    }
    void read_config(const std::string& file_name);

    std::list<config> get_camera_configs() const{
        return camera_configs;
    }

    template<typename T>
    T get(const std::string& field, const T& default_value){
        if(fs[field].empty())
            return default_value;
        return fs[field];
    }


private:
    std::list<config> camera_configs;
    cv::FileStorage fs;
};



} // namespace configure



#endif //CONFIG_H

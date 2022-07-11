#ifndef DEVICE_H
#define DEVICE_H

#include <memory>
#include <list>

class Device{

public:

Device();
Device(const Device&)=delete;
Device(Device&&)=default;

static std::list<Device> enumerate();


private:



};





#endif #DEVICE_H
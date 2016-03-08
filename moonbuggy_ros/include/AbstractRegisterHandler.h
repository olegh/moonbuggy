
#ifndef ROS_ADAPTER_ABSTRACTREGISTERHANDLER_H
#define ROS_ADAPTER_ABSTRACTREGISTERHANDLER_H

#include "ros/ros.h"


class BaseRegisterHandler : public Handler {
public:
    BaseRegisterHandler(){}

    virtual void onReadReg(uint8_t address){
        //assert(0 && "Not expected behaviour from target");
    }

    virtual void onWriteReg(uint8_t address, uint16_t value){
        if(ros::ok()){
            onWriteRegImpl(address, value);
        }else{
            exit(0);
        }

        ros::spinOnce();
    }

    virtual ~BaseRegisterHandler(){}

private:
    BaseRegisterHandler& operator=(const BaseRegisterHandler& );
    BaseRegisterHandler(const BaseRegisterHandler&);

    virtual void onWriteRegImpl(uint8_t address, uint16_t value) = 0;
};

class AbstractRegisterHandler : public BaseRegisterHandler {

public:
    AbstractRegisterHandler(ros::Publisher& publisher):publisher_(publisher){}

    virtual ~AbstractRegisterHandler(){}

    ros::Publisher& publisher(){
        return publisher_;
    }

private:
    ros::Publisher& publisher_;
};

#endif //ROS_ADAPTER_ABSTRACTREGISTERHANDLER_H

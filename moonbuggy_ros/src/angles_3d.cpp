//
// Created by x0leg on 1/31/16.
//
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"


#include "Protocol.h"
#include "PortDataHandler.h"
#include "AbstractRegisterHandler.h"

static char* SERIAL_PORT = "/dev/ttyACM3";

using namespace boost::asio;


class RegisterHandler : public AbstractRegisterHandler{
public:
    typedef AbstractRegisterHandler super;

    RegisterHandler(ros::Publisher& publisher): super(publisher){
        xReady_ = yReady_ = zReady_= false;
    }

private:
    void onWriteRegImpl(uint8_t address, uint16_t value){
        if(address == XREG){
            point_.x = (int16_t)value;
            xReady_ = true;
        }else if(address == YREG){
            point_.y = (int16_t)value;
            yReady_ = true;
        }else if(address == ZREG){
            point_.z = (int16_t)value;
            zReady_ = true;
        }

        postIfReady();
    }

    void postIfReady(){
        if(xReady_ && yReady_ && zReady_){
            xReady_ = yReady_ = zReady_= false;
            publisher().publish(point_);
        }
    }

    enum{ XREG = 10,
          YREG = 11,
          ZREG = 12};

    bool xReady_;
    bool yReady_;
    bool zReady_;

    geometry_msgs::Point point_;
};

class ProtocolOutput : public Output{
    virtual void put(uint8_t byte){

    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "angles_3d");
    ros::NodeHandle node;
    ros::Publisher angles_pub = node.advertise<geometry_msgs::Point>("angles", 1000);

    char* portTTY = SERIAL_PORT;
    if(argc > 1){
        portTTY = argv[1];
    }

    io_service io_service;
    serial_port port(io_service, portTTY);
    port.set_option(serial_port::baud_rate(57600));

    RegisterHandler registerHandler(angles_pub);
    ProtocolOutput nullOutput;
    Protocol protocol(nullOutput, registerHandler);
    PortDataHandler portDataHandler(protocol, port);

    portDataHandler.async_read();
    io_service.run();

    return 0;
}


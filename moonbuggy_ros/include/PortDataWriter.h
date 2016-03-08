//
// Created by x0leg on 2/28/16.
//

#ifndef ROS_ADAPTER_PORTDATAWRITER_H
#define ROS_ADAPTER_PORTDATAWRITER_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "Protocol.h"

class PortDataWriter{

public:
    PortDataWriter(
            Protocol& protocol,
            boost::asio::serial_port& port)
    :protocol_(protocol),
    port_(port){
    }

private:
    Protocol& protocol_;
    boost::asio::serial_port& port_;
};
#endif //ROS_ADAPTER_PORTDATAWRITER_H

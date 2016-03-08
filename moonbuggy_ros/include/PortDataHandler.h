//
// Created by x0leg on 2/7/16.
//

#ifndef ROS_ADAPTER_PORTDATAHANDLER_H
#define ROS_ADAPTER_PORTDATAHANDLER_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "Protocol.h"

class PortDataHandler{
public:
    PortDataHandler(Protocol& protocol, boost::asio::serial_port& port)
            :protocol_(protocol),
             port_(port){
        buf_.resize(64);

    }

    void data_handler(
            const boost::system::error_code& error,
            std::size_t bytes_transferred){

        if(!error && bytes_transferred > 0){
            for(std::size_t i = 0; i < bytes_transferred; ++i){
                protocol_.onByte(buf_[i]);
            }
        }

        async_read();
    }

    void async_read(){
        port_.async_read_some(
                boost::asio::buffer(buf_),
                boost::bind(&PortDataHandler::data_handler,
                            this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
    }

private:
    Protocol& protocol_;
    std::vector<uint8_t> buf_;
    boost::asio::serial_port& port_;
};

#endif //ROS_ADAPTER_PORTDATAHANDLER_H

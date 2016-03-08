#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

#include "Protocol.h"
#include "PortDataHandler.h"
#include "AbstractRegisterHandler.h"

static const char *SERIAL_PORT = "/dev/ttyACM1";

using namespace boost::asio;

class RegisterHandler : public AbstractRegisterHandler {
public:
    typedef AbstractRegisterHandler super;

    RegisterHandler(ros::Publisher &publisher) : super(publisher) {

    }

private:
    void onWriteRegImpl(uint8_t address, uint16_t value) {
        if (IRDAR_ANGLE == address) {
            angle_ = value;
            return;
        }

        if (IRDAR_ADC == address) {
            /*
                x = [value read from sensor]
                y = x * [your Arduino boards Vcc (5 or 3.3V)] / 1024
                distance = 61.681 * y ^ -1.133
            */
            //double angle = ((double) angle_) * 3.14 / 180;
            double y = ((double)value) * 5.0 / 1024.0;
            double distance = 61.681 * pow(y, -1.133);

            samples_[angle_] = distance;
            return;
        }

        if(IRDAR_DONE == address){
            publishData();
        }
    }

    void publishData() {
        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now();
        scan.header.frame_id = "laser_frame";
        scan.angle_min = 0;
        scan.angle_max = 3.14;
        scan.scan_time = 3;
        scan.angle_increment = 3.14 / 180.0;
        scan.time_increment = 0;
        scan.range_min = 0.2;
        scan.range_max = 4.0;

        for (std::map<int, double>::iterator i = samples_.begin(); i != samples_.end() ; ++i ) {
            double distance = i->second / 100.0;
            if(distance > scan.range_max || distance < scan.range_min)
                distance = 4;

            scan.ranges.push_back(distance);
            //scan.intensities.push_back(distance);
        }

        publisher().publish(scan);
        std::cout << samples_.size() << std::endl;
        samples_.clear();
    }

    enum {
        IRDAR_ADC = 41,
        IRDAR_ANGLE = 42,
        IRDAR_DONE = 43
    };

    int angle_;
    std::map<int,double> samples_;
};

class ProtocolOutput : public Output {
    virtual void put(uint8_t byte) {

    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "base_scan");
    ros::NodeHandle node;
    ros::Publisher angles_pub = node.advertise<sensor_msgs::LaserScan>("/base_scan", 1000);

    io_service io_service;
    serial_port port(io_service, SERIAL_PORT);
    port.set_option(serial_port::baud_rate(57600));

    RegisterHandler registerHandler(angles_pub);
    ProtocolOutput nullOutput;
    Protocol protocol(nullOutput, registerHandler);
    PortDataHandler portDataHandler(protocol, port);

    portDataHandler.async_read();
    io_service.run();

    return 0;
}


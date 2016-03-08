#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/assign.hpp>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/assign.hpp>

#include "Regs.h"
#include "Protocol.h"
#include "PortDataHandler.h"
#include "AbstractRegisterHandler.h"

#if 1
using namespace hardware_interface;
using namespace boost;
using namespace boost::asio;

class MotorsHS : public hardware_interface::RobotHW,
                 public BaseRegisterHandler {
public:

    typedef BaseRegisterHandler super;

    MotorsHS(Protocol &protocol)
            : protocol_(protocol) {

        ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
                ("rear_left_wheel")("front_right_wheel")("rear_right_wheel");

        for (unsigned int i = 0; i < joint_names.size(); i++) {
            hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                    &joints_[i].position, &joints_[i].velocity,
                                                                    &joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(
                    joint_state_handle, &joints_[i].velocity_command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
    }

    void write() {
        protocol_.writeRegister(LEFT_FRONT_ROTOR_RPS, radSecToMotorRPS(joints_[0].velocity_command));
        protocol_.writeRegister(LEFT_REAR_ROTOR_RPS, radSecToMotorRPS(joints_[1].velocity_command));
        protocol_.writeRegister(RIGHT_FRONT_ROTOR_RPS, radSecToMotorRPS(joints_[2].velocity_command));
        protocol_.writeRegister(RIGHT_REAR_ROTOR_RPS, radSecToMotorRPS(joints_[3].velocity_command));
    }


private:
    const double rotorToWheelTransmission = 1.0 / 72.0;

    int16_t radSecToMotorRPS(double radSec) {
        std::cout << "rad/s: " << radSec << std::endl;
        double radSecRotor = radSec / rotorToWheelTransmission;
        double rps = radSecRotor / M_PI;
        return boost::numeric_cast<int16_t>(rps);
    }

    double RPStoRadSec(int16_t rps) {
        return rps * rotorToWheelTransmission * M_PI;
    }

    double rotorRoundToWheelRounds(int16_t rotorRounds){
        return (rotorRounds) * rotorToWheelTransmission;
    }

    virtual void onWriteRegImpl(uint8_t address, uint16_t value) {
        switch (address) {
            case LEFT_FRONT_ROTOR_ROUNDS:
                joints_[0].position = rotorRoundToWheelRounds(value);
                break;

            case LEFT_REAR_ROTOR_ROUNDS:
                joints_[1].position  = rotorRoundToWheelRounds(value);
                break;

            case RIGHT_FRONT_ROTOR_ROUNDS:
                joints_[2].position  = rotorRoundToWheelRounds(value);
                break;

            case RIGHT_REAR_ROTOR_ROUNDS:
                joints_[3].position = rotorRoundToWheelRounds(value);
                break;
        }
    }

    JointStateInterface joint_state_interface_;
    VelocityJointInterface velocity_joint_interface_;
    Protocol &protocol_;

    // These are mutated on the controls thread only.
    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {
        }
    } joints_[4];
};



class ProtocolOutput : public Output{
    virtual void put(uint8_t byte){

    }
};

struct HandlerPImpl : Handler{
    Handler* handler;

    virtual void onReadReg(uint8_t address) {
        handler->onReadReg(address);
    }

    virtual void onWriteReg(uint8_t address, uint16_t value){
        handler->onWriteReg(address, value);
    }
};

void onTime(boost::asio::deadline_timer& timer,
            controller_manager::ControllerManager& cm,
            MotorsHS& motors,
            const boost::system::error_code& error){
    if(!error){
        try {
            timer.expires_from_now(boost::posix_time::millisec(50));
            cm.update(ros::Time::now(), ros::Duration(0.05));
            motors.write();

            timer.async_wait(boost::bind(onTime, ref(timer), ref(cm), ref(motors), _1));
        }catch(...){
            std:: cout << "Error:" <<
                boost::current_exception_diagnostic_information(true) << std::endl;
        }

    }
}

typedef boost::chrono::steady_clock time_source;


void run(boost::asio::io_service& io_service,
         deadline_timer& timer,
         PortDataHandler& portDataHandler,
         controller_manager::ControllerManager& cm,
         MotorsHS& motors  ){
    timer.expires_from_now(boost::posix_time::millisec(50));
    timer.async_wait(boost::bind(onTime, ref(timer), ref(cm), ref(motors), _1));
    portDataHandler.async_read();

    io_service.run();
}

class Serial : public Output {
public:
    Serial(serial_port& port)
            :port_(port){}

    virtual void put(uint8_t byte) {
        port_.write_some(boost::asio::buffer(&byte, 1));
    }

private:
    serial_port& port_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "motor_drive");

    if(argc < 2){
        printf("please, specify serial port");
        exit(-1);
    }

    const char* portTTY = argv[1];

    io_service io_service;
    serial_port port(io_service, portTTY);
    port.set_option(serial_port::baud_rate(57600));

    HandlerPImpl handlerPImpl;
    Serial serialOutput(port);
    Protocol protocol(serialOutput, handlerPImpl);
    PortDataHandler portDataHandler(protocol, port);
    MotorsHS motors(protocol);
    handlerPImpl.handler = &motors;

    // Background thread for the controls callback.
    ros::NodeHandle controller_nh("");
    controller_manager::ControllerManager cm(&motors, controller_nh);

    deadline_timer timer(io_service);

    boost::thread th = boost::thread(boost::bind(
            run,
            ref(io_service),
            ref(timer),
            ref(portDataHandler),
            ref(cm),
            ref(motors)
    ));

    ros::spin();

    return 0;
}
#else

#include <string>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/assign.hpp>

#include "controller_manager/controller_manager.h"
//#include "jackal_base/jackal_diagnostic_updater.h"
//#include "jackal_base/jackal_hardware.h"
#include "ros/ros.h"
//#include "rosserial_server/serial_session.h"

typedef boost::chrono::steady_clock time_source;

namespace jackal_base {

    class JackalHardware : public hardware_interface::RobotHW {
    public:
        JackalHardware() {
            ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
                    ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");

            for (unsigned int i = 0; i < joint_names.size(); i++) {
                hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                        &joints_[i].position, &joints_[i].velocity,
                                                                        &joints_[i].effort);
                joint_state_interface_.registerHandle(joint_state_handle);

                hardware_interface::JointHandle joint_handle(
                        joint_state_handle, &joints_[i].velocity_command);
                velocity_joint_interface_.registerHandle(joint_handle);
            }
            registerInterface(&joint_state_interface_);
            registerInterface(&velocity_joint_interface_);
        }

        void copyJointsFromHardware() {
            boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);

            for (int i = 0; i < 4; i++) {
                joints_[i].position = 1;
                joints_[i].velocity = 2;
                joints_[i].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
            }

        }

        void publishDriveFromController() {
            for(int i = 0; i < 4; i++){
                std::cout << joints_[i].velocity_command << " ";
            }

            std::cout << "\n";
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber feedback_sub_;


        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        // These are mutated on the controls thread only.
        struct Joint {
            double position;
            double velocity;
            double effort;
            double velocity_command;

            Joint() : position(0), velocity(0), effort(0), velocity_command(0) {
            }
        } joints_[4];

        // This pointer is set from the ROS thread.
        boost::mutex feedback_msg_mutex_;
    };

}  // namespace jackal_base
#undef sleep
void controlThread(ros::Rate rate, jackal_base::JackalHardware *robot, controller_manager::ControllerManager *cm) {
    time_source::time_point last_time = time_source::now();

    while (1) {
        // Calculate monotonic time elapsed
        time_source::time_point this_time = time_source::now();
        boost::chrono::duration<double> elapsed_duration = this_time - last_time;
        ros::Duration elapsed(elapsed_duration.count());
        last_time = this_time;

        robot->copyJointsFromHardware();
        cm->update(ros::Time::now(), elapsed);
        robot->publishDriveFromController();
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    // Initialize ROS node.
    ros::init(argc, argv, "jackal_node");
    jackal_base::JackalHardware jackal;

    boost::asio::io_service io_service;
    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // Background thread for the controls callback.
    ros::NodeHandle controller_nh("");
    controller_manager::ControllerManager cm(&jackal, controller_nh);
    boost::thread(boost::bind(controlThread, ros::Rate(50), &jackal, &cm));

    // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
    ros::spin();

    return 0;
}

#endif
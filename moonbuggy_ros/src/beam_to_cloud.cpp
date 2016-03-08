#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

public:
    ros::NodeHandle& n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_notifier_;
    ros::Publisher scan_pub_;

    LaserScanToPointCloud(ros::NodeHandle& n) :
            n_(n),
            projector_(),
            listener_(),
            laser_sub_(n_, "/base_scan", 10)
            //laser_notifier_(laser_sub_,listener_, "map", 10)
    {
        laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, listener_, "laser_frame", 10);
        laser_notifier_->setTolerance(ros::Duration(3, 0));
        laser_notifier_->registerCallback(
                boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));

        laser_notifier_->registerFailureCallback(
                boost::bind(&LaserScanToPointCloud::failureCallback, this, _1, _2));

        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    void failureCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in, tf::filter_failure_reasons::FilterFailureReason reason){
        std::cout << "Fail reason:" << reason << std::endl;
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;
        try
        {
            projector_.transformLaserScanToPointCloud(
                    "laser_frame",*scan_in, cloud,listener_);
        }
        catch (tf::TransformException& e)
        {
            std::cout << e.what() << std::endl;
            return;
        }

        // Do something with cloud.

        scan_pub_.publish(cloud);

    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

    ros::spin();

    return 0;
}
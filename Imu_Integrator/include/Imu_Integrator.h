#ifndef IMU_INTEGRATOR_H
#define IMU_INTEGRATOR_H

// ROS includes.
// #include "ros/ros.h"
// #include "ros/time.h"
// #include <visualization_msgs/Marker.h>
// #include<sensor_msgs/Imu.h>
// #include <cmath>

#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include "spdlog/spdlog.h"
// #include </home/dygg/armlib/eigen339/include/eigen3/Eigen>

// Custom message includes. Auto-generated from msg/ directory.
/*
struct Orientation
{

};

struct Position
{
    double x, y, z;
};
*/
struct ImuMsg
{
    /*
    Orientation orien;
    Position pos;
    */
   std::chrono::steady_clock::time_point time;
//    double ax,ay,az,wx,wy,wz;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    // ros::Time time;
    std::chrono::steady_clock::time_point time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    // visualization_msgs::Marker path;
    std::vector<Eigen::Vector3d> path;
    // ros::Publisher line_pub;
    double deltaT;
    bool firstT;
public:
    //! Constructor.
    ImuIntegrator();
    //! Destructor.
    ~ImuIntegrator();

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    // void publishMessage();

    Eigen::Vector3d getCurrPose();
    std::vector<Eigen::Vector3d> getPath();


    //! Callback function for subscriber.
    void ImuCallback(const ImuMsg &msg);

    void setGravity(const Eigen::Vector3d &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const Eigen::Vector3d &msg);
    void calcOrientation(const Eigen::Vector3d &msg);
};

#endif // SR_NODE_EXAMPLE_CORE_H

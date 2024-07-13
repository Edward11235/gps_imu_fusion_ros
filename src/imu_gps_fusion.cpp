#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include "ekfNavINS.h"

#include <iostream>
#include <tuple>

using namespace std;

static int running = 0;
ekfNavINS ekf;

struct ImuData {
    float ax, ay, az;
    float p, q, r;
    float hx, hy, hz;
};

ImuData imuData;

struct GpsVelData {
    double vn, ve, vd;
};

GpsVelData gpsVelData;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imuData.ax = msg->linear_acceleration.x;
    imuData.ay = msg->linear_acceleration.y;
    imuData.az = msg->linear_acceleration.z;
    imuData.p = msg->angular_velocity.x;
    imuData.q = msg->angular_velocity.y;
    imuData.r = msg->angular_velocity.z;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    imuData.hx = msg->magnetic_field.x;
    imuData.hy = msg->magnetic_field.y;
    imuData.hz = msg->magnetic_field.z;
}

void gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    gpsVelData.vn = msg->twist.linear.x;
    gpsVelData.ve = msg->twist.linear.y;
    gpsVelData.vd = msg->twist.linear.z;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    // Update EKF with GPS and IMU data
    ekf.ekf_update(ros::Time::now().toNSec(), gpsVelData.vn, gpsVelData.ve, gpsVelData.vd, lat, lon, alt,
                   imuData.p, imuData.q, imuData.r,
                   imuData.ax, imuData.ay, imuData.az,
                   imuData.hx, imuData.hy, imuData.hz);

    // Prepare and publish the filtered state
    std_msgs::Float64MultiArray filtered_state;
    filtered_state.data.resize(15);

    filtered_state.data[0] = ekf.getLatitude_rad();
    filtered_state.data[1] = ekf.getLongitude_rad();
    filtered_state.data[2] = ekf.getAltitude_m();
    filtered_state.data[3] = ekf.getVelNorth_ms();
    filtered_state.data[4] = ekf.getVelEast_ms();
    filtered_state.data[5] = ekf.getVelDown_ms();
    filtered_state.data[6] = ekf.getRoll_rad();
    filtered_state.data[7] = ekf.getPitch_rad();
    filtered_state.data[8] = ekf.getHeading_rad();
    filtered_state.data[9] = ekf.getGyroBiasX_rads();
    filtered_state.data[10] = ekf.getGyroBiasY_rads();
    filtered_state.data[11] = ekf.getGyroBiasZ_rads();
    filtered_state.data[12] = ekf.getAccelBiasX_mss();
    filtered_state.data[13] = ekf.getAccelBiasY_mss();
    filtered_state.data[14] = ekf.getAccelBiasZ_mss();

    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/filtered_state", 1000);
    pub.publish(filtered_state);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ekf_nav_ins");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 1000, imuCallback);
    ros::Subscriber mag_sub = nh.subscribe("/imu/mag", 1000, magCallback);
    ros::Subscriber gps_sub = nh.subscribe("/fix", 1000, gpsCallback);
    ros::Subscriber gps_vel_sub = nh.subscribe("/vel", 1000, gpsVelCallback);

    // Set running flag
    running = 1;

    // Spin
    ros::Rate rate(10); // Adjust the rate as needed
    while (ros::ok() && running) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

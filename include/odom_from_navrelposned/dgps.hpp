//
// Created by mnowicki on 23.09.2019 in ROS1. transformed to ROS2 by Iman Esfandiyar 4/12/2023
//

#ifndef ODOM_FROM_NAVRELPOSNED_H
#define ODOM_FROM_NAVRELPOSNED_H


#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <odom_from_navrelposned/gps_common/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <fstream>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Needed for relposned
#include "ublox_msgs/msg/nav_relposned9.hpp"

namespace odom_from_navrelposned
{
    class DGPS : public rclcpp::Node
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum GPSStatus {
            FIX,
            FLOAT,
            FIXED,
            FIXwithHeading,
            FLOATwithHeading,
            FIXEDwithHeading
        };

    public:
        // Setups everything, reads parameters
        DGPS();
        ~DGPS();

        void readParams();

        // Callback for gps fix (lat and lon) and heading in moving base mode
        void callbackDGPSRelposned(const ublox_msgs::NavRELPOSNED9::Ptr &msg);
        void callbackDGPSFix(const sensor_msgs::NavSatFixConstPtr &fix);
        void callbackDGPSFixRover(const sensor_msgs::NavSatFixConstPtr &fix);

        // Method to compute global pose based on measurements from GPS1 and GPS2 & send it
        void publishGlobalPose(const std_msgs::Header &header);

        // Methods to perform visualization in shared CS
        void visualizeDGPS(const std_msgs::Header &header, const GPSStatus &status, const Eigen::Matrix4d &rearAxleInCharger);

    private:
        // Helper to convert to metric
        bool LLtoUTM(const sensor_msgs::NavSatFixConstPtr &fix, double &northing, double &easting, double &altitude);

        // Computes the inverse transformation
        static Eigen::Matrix4d inverse(const Eigen::Matrix4d &pose);


        // ros::NodeHandle nh;
        // ros::Subscriber subDGPSFix, subDGPSFixRover, subDGPSRelPosNed, subChargerFrontPose3, subChargerFrontPose6, subChargerBackPose, subXsens;
        rclcpp::Subscription<sensor_msgs::NavSatFixConstPtr>::SharedPtr subDGPSFix;  //subDGPSFix
        rclcpp::Subscription<sensor_msgs::NavSatFixConstPtr>::SharedPtr subDGPSFixRover; // subDGPSFixRover
        rclcpp::Subscription<ublox_msgs::NavRELPOSNED9>::SharedPtr subDGPSRelPosNed; // subDGPSRelPosNed

        rclcpp::Publisher<geometry_msgs::PoseWithCovarianceStamped>::SharedPtr pubPoseGPS;  //pubPoseGPS
        rclcpp::Publisher<visualization_msgs::Marker>::SharedPtr pubVisDGPS;  //pubVisDGPS
        rclcpp::Publisher<nav_msgs::Odometry>::SharedPtr pubBusRearAxisInMap;

        // ros::Publisher pubPoseGPS, pubPoseCam, pubPoseCamInChargerCam;
        // ros::Publisher pubVisCharger, pubVisCam, pubVisDGPS;

        // Charger location in the World
        double gps_parking_spot_latitude, gps_parking_spot_longitude, gps_parking_spot_altitude, gps_parking_spot_heading;

        // Place where out setup will be installed (Requires data from SOLARIS)
        Eigen::Vector3d tStandInFrontAxle, tStandInBackAxle;

        // AHRS quaternion while static
        Eigen::Vector4d frontAhrsQuat, backAhrsQuat;
        double front_assumed_tx, front_assumed_ty, front_assumed_tz, gpsCameraTimeShift;
        double front_assumed_yaw, front_assumed_pitch, front_assumed_roll, chargerYaw;

        // charger - location of charger in UTM (x, y, altitude, heading)
        Eigen::Vector4d charger;

        // lastDGPS - location of dgps in UTM
        Eigen::Vector3d lastDGPS, prevDGPS;
        Eigen::Vector3d lastDGPSRover, prevDGPSRover;
        char lastStatusDGPS;
        char lastStatusDGPSRover;
        std::string chargerZone;

        // last timestamp and heading from relposned
        unsigned int lastRelPosNedITow;
        double lastHeading;


        // Transformations needed to do
        // rearAxle -> stand -> gps2 -> charger -> rotCharger -> chargerVFO
        // rearAxle -> stand -> ahrs -> camera -> rotCharger -> chargerVFO
        Eigen::Matrix4d standInFrontAxle, standInBackAxle, gps2InStand, horCamInFrontStand, horCamInBackStand, rotChargerInCharger, chargerVFOinRotCharger, chargerVFOinCamCharger;
        Eigen::Matrix4d frontCamInHorFrontCam, backCamInHorBackCam;
    //    Eigen::Matrix4d frontAxleInRearAxle;
        Eigen::Matrix4d chargerCamInRotCharger;
        Eigen::Matrix4d chargerPointRotation;

        // Previous frontAxleInCharger
        bool systemInitialized;
        uint64_t prevTimestamp;
        Eigen::Matrix4d prevFrontAxleInCharger;

        // Visualization
        unsigned int visGpsSeq, visCamSeq, visChargerSeq;

        // 0 - no printing, 1 - info to the screen
        int verbose;

        // File to store measurements for analysis
        std::ofstream file;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        
    };
}
#endif //ODOM_FROM_NAVRELPOSNED_H

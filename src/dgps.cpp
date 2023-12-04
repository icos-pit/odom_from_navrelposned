#include "odom_from_navrelposned/dgps.hpp"

#include "rclcpp/rclcpp.hpp"


using namespace std;
using std::placeholders::_1;
namespace odom_from_navrelposned
{
    DGPS::DGPS() : Node("odom_from_navrelposned"),
    visGpsSeq(0), visChargerSeq(0), lastRelPosNedITow(0), lastStatusDGPS(-1), tfListener(tfBuffer)
    {
        

        lastDGPS = Eigen::Vector3d::Zero();
        prevDGPS = Eigen::Vector3d::Zero();
        lastDGPSRover = Eigen::Vector3d::Zero();
        prevDGPSRover = Eigen::Vector3d::Zero();
        charger = Eigen::Vector4d::Zero();

        // Computing charger location in UTM (X - north, Y - east, altitude - up, heading)
        chargerZone = "";
        gps_common::LLtoUTM(gps_parking_spot_latitude, gps_parking_spot_longitude, charger[0], charger[1], chargerZone);
        charger[2] = gps_parking_spot_altitude;
        // Changing to degrees and changing the direction to rotation along the Z axis of VFO (Z up) instead of Z axis of GPS (Z down)
        charger[3] = gps_parking_spot_heading * M_PI / 180.0;

        // Single point based on data from Solaris
        standInFrontAxle = Eigen::Matrix4d::Identity();
        standInFrontAxle.block<3, 1>(0, 3) = tStandInFrontAxle.transpose();

        standInBackAxle = Eigen::Matrix4d::Identity();
        standInBackAxle.block<3, 1>(0, 3) = tStandInBackAxle.transpose();

        // Measured location of DGPS antenna w.r.t. the origin of suction cup
        // Stand axes -> X-front, Y-left, Z-up
        // GPS axes -> X-front, Y-right, Z-down
        gps2InStand = Eigen::Matrix4d::Identity();
        gps2InStand(1, 1) = -1;
        gps2InStand(2, 2) = -1;
        gps2InStand(0, 3) = 0.0;
        gps2InStand(1, 3) = 0.0;
        gps2InStand(2, 3) = 0.0;

        // Based on the global orientation of the charger
        // charger - global GPS
        // rotCharger - GPS CS but rotated according to the heading of the charger
        rotChargerInCharger = Eigen::Matrix4d::Identity();
        rotChargerInCharger.block<3, 3>(0, 0) = Eigen::AngleAxisd(charger[3],
                                                                Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Changing the orientation of the axes
        // chargerVF0 -> X-front, Y-left, Z-up
        // RotCharger -> X-front, Y-right, Z-down
        chargerVFOinRotCharger = Eigen::Matrix4d::Identity();
        chargerVFOinRotCharger(1, 1) = -1; // Flipping y axis
        chargerVFOinRotCharger(2, 2) = -1; // Flipping z axis


        // Starting subscription --> fix
        auto dgpsFixTopic = this->declare_parameter<std::string>("dgps_fix_topic", "/ublox_moving_base/fix");
        subDGPSFix = this->create_subscription<sensor_msgs::msg::NavSatFix>(dgpsFixTopic, 10, std::bind(&DGPS::callbackDGPSFix, this, std::placeholders::_1));

        auto dgpsFixTopicRover = this->declare_parameter<std::string>("dgps_rover_fix_topic", "/ublox_rover/fix");
        subDGPSFixRover = this->create_subscription<sensor_msgs::msg::NavSatFix>(dgpsFixTopicRover, 10, std::bind(&DGPS::callbackDGPSFixRover, this, std::placeholders::_1));

        // Subscription for dgps_rover/navrelposned
        auto dgpsRelPosNedTopic = this->declare_parameter<std::string>("dgps_relposned_topic", "/navrelposned");
        subDGPSRelPosNed = this->create_subscription<ublox_msgs::msg::NavRELPOSNED9>(dgpsRelPosNedTopic, 10, std::bind(&DGPS::callbackDGPSRelposned, this, std::placeholders::_1));

        // Publisher for location_gps
        auto locationGpsTopic = this->declare_parameter<std::string>("location_gps_topic", "/gps_pose_estimator/location_gps");
        pubPoseGPS = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(locationGpsTopic, 10);
        
        // Visualization
        pubVisDGPS = this->create_publisher<visualization_msgs::msg::Marker>("/visualization/dgps", 1000);

        // Rear axis odometry
        pubBusRearAxisInMap = this->create_publisher<nav_msgs::msg::Odometry>("/gps_pose_estimator/rear_axis_pose", 5);


        systemInitialized = false;
        prevFrontAxleInCharger = Eigen::Matrix4d::Identity();

        

    }

    DGPS::~DGPS() {
        if (file.is_open())
            file.close();
    }   

    void DGPS::readParams() {

        // Charger location in the world! GPS lat, lon, alt, heading
        if (this->get_parameter("gps_parking_spot_latitude", gps_parking_spot_latitude))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set gps_parking_spot_latitude: " << gps_parking_spot_latitude);
        if (this->get_parameter("gps_parking_spot_longitude", gps_parking_spot_longitude))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set gps_parking_spot_longitude: " << gps_parking_spot_longitude);
        if (this->get_parameter("gps_parking_spot_altitude", gps_parking_spot_altitude))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set gps_parking_spot_altitude: " << gps_parking_spot_altitude);
        if (this->get_parameter("gps_parking_spot_heading", gps_parking_spot_heading))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set gps_parking_spot_heading: " << gps_parking_spot_heading);

        if (this->get_parameter("stand_in_frontAxle_x", tStandInFrontAxle[0]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_frontAxle_x: " << tStandInFrontAxle[0]);
        if (this->get_parameter("stand_in_frontAxle_y", tStandInFrontAxle[1]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_frontAxle_y: " << tStandInFrontAxle[1]);
        if (this->get_parameter("stand_in_frontAxle_z", tStandInFrontAxle[2]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_frontAxle_z: " << tStandInFrontAxle[2]);

        if (this->get_parameter("stand_in_backAxle_x", tStandInBackAxle[0]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_backAxle_x: " << tStandInBackAxle[0]);
        if (this->get_parameter("stand_in_backAxle_y", tStandInBackAxle[1]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_backAxle_y: " << tStandInBackAxle[1]);
        if (this->get_parameter("stand_in_backAxle_z", tStandInBackAxle[2]))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set stand_in_backAxle_z: " << tStandInBackAxle[2]);

        if (this->get_parameter("verbose", verbose))
            RCLCPP_INFO_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "Set verbose: " << verbose);
    }   


    // void DGPS::callbackDGPSRelposned(const ublox_msgs::msg::NavRELPOSNED9::SharedPtr &msg) 
    void DGPS::callbackDGPSRelposned(const std::shared_ptr<ublox_msgs::msg::NavRELPOSNED9> &msg) 
    {
        
        lastRelPosNedITow = msg->i_tow;
        // +180.0 -- navrelposned reports MB location in rover (and heading is taken from that). We need rover location in MB so inverse
        // lastHeading = msg->relPosHeading * 1e-5;
      
        double relNorth = msg->rel_pos_n + msg->rel_pos_hpn / 100.0;
        double relEast = msg->rel_pos_e + msg->rel_pos_hpe / 100.0;
        if (relNorth != 0 || relEast != 0)
            lastHeading = atan2(relEast, relNorth) * 180.0 / M_PI;
    }

    // void DGPS::callbackDGPSFix(const sensor_msgs::msg::NavSatFix::SharedPtr &fix) 
    void DGPS::callbackDGPSFix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &fix) 
    {
        // Save last measurement
        if (LLtoUTM(fix, lastDGPS[0], lastDGPS[1], lastDGPS[2])) {
            lastStatusDGPS = fix->status.status;

            // Let's update & send global pose
            // TODO: No timestamp checking is performed
            publishGlobalPose(fix->header);

            // Just save when GPS1 is not available
            prevDGPS = lastDGPS;
        };
    }

    void DGPS::callbackDGPSFixRover(const sensor_msgs::msg::NavSatFix::SharedPtr &fix) {

        // Save last measurement
        if (LLtoUTM(fix, lastDGPSRover[0], lastDGPSRover[1], lastDGPSRover[2])) {
            lastStatusDGPSRover = fix->status.status;

            // Just save when GPS1 is not available
            prevDGPSRover = lastDGPSRover;
        };
    }


    void DGPS::publishGlobalPose(const std_msgs::msg::Header &header) {

        // Heading of the bus
        double theta = 0.0;
        GPSStatus dataStatus;

        // Let's check if we have a valid measurement (FLOAT or FIXED) and relative pose information
        if (lastStatusDGPS > 1 && lastRelPosNedITow > 0) {

            // We have a relposned measurement
            theta = lastHeading * M_PI / 180.0;

            // Setting the status for visualization
            if (lastStatusDGPS == 0 || lastStatusDGPS == 1)
                dataStatus = GPSStatus::FIXwithHeading;
            else if (lastStatusDGPS == 2)
                dataStatus = GPSStatus::FLOATwithHeading;
            else if (lastStatusDGPS == 3)
                dataStatus = GPSStatus::FIXEDwithHeading;
        }
            // No theta measurement - assuming from motion
        else {
            // Pose of the gps2 in charger (not rotated!) from MOTION
            Eigen::Vector3d curInPrev = lastDGPS - prevDGPS;

            // We do it only when enough motion was recorded
            // if (curInPrev.norm() > 0.05)
            //     theta = atan2(curInPrev[1], curInPrev[0]);
            // else
            theta = charger[3];

            // We have a relposned measurement
            if (lastRelPosNedITow > 0) {
                theta = lastHeading * M_PI / 180.0;
            }

            // Setting the status for visualization
            if (lastStatusDGPS == 0 || lastStatusDGPS == 1)
                dataStatus = GPSStatus::FIX;
            else if (lastStatusDGPS == 2)
                dataStatus = GPSStatus::FLOAT;
            else if (lastStatusDGPS == 3)
                dataStatus = GPSStatus::FIXED;
        }

        // Pose of the DGPS + additional heading is available
        Eigen::Matrix4d gps2InCharger = Eigen::Matrix4d::Identity();
        gps2InCharger.block<3, 1>(0, 3) = lastDGPS;
        gps2InCharger.block<3, 3>(0, 0) = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Final pose from DGPS
        // For Upboard frontAxle means moving base position
        Eigen::Matrix4d chargerInFrontAxle =
                standInFrontAxle * gps2InStand * inverse(gps2InCharger) * rotChargerInCharger * chargerVFOinRotCharger;

        // Inverse of final: frontAxle in charger
        Eigen::Matrix4d frontAxleInCharger = inverse(chargerInFrontAxle);

        // Getting the theta angle (rotation by Z axis)
        double finalTheta = atan2(frontAxleInCharger(1, 0), frontAxleInCharger(0, 0));

        // Sending theta as quaternion
        Eigen::Quaterniond quatTheta(frontAxleInCharger.block<3, 3>(0, 0));

        // Preparing to publish
        geometry_msgs::msg::PoseWithCovarianceStamped poseOut;
        poseOut.header = header;
        poseOut.header.frame_id = "base_link";
        poseOut.pose.pose.position.x = frontAxleInCharger(0, 3);
        poseOut.pose.pose.position.y = frontAxleInCharger(1, 3);
        poseOut.pose.pose.position.z = frontAxleInCharger(2, 3);

        poseOut.pose.pose.orientation.x = quatTheta.x();
        poseOut.pose.pose.orientation.y = quatTheta.y();
        poseOut.pose.pose.orientation.z = quatTheta.z();
        poseOut.pose.pose.orientation.w = quatTheta.w();

        if (dataStatus == GPSStatus::FIXEDwithHeading) {
            pubPoseGPS->publish(poseOut);

            auto br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Rovers position (for visualization only)
            Eigen::Matrix4d gpsRoverInCharger = Eigen::Matrix4d::Identity();
            gpsRoverInCharger.block<3, 1>(0, 3) = lastDGPSRover;
            gpsRoverInCharger.block<3, 3>(0, 0) = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            
            Eigen::Matrix4d chargerInRover = standInFrontAxle  * gps2InStand * inverse(gpsRoverInCharger) * rotChargerInCharger * chargerVFOinRotCharger;
            Eigen::Matrix4d roverInCharger = inverse(chargerInRover);
            Eigen::Quaterniond quatRover(roverInCharger.block<3, 3>(0, 0));

            // Transform for GPS Rover based on rover/fix
            geometry_msgs::msg::TransformStamped transformStampedRover;
            transformStampedRover.header = header;
            transformStampedRover.header.frame_id = "map";
            transformStampedRover.child_frame_id = "gps_rover_fix";
            transformStampedRover.transform.translation.x = roverInCharger(0, 3);
            transformStampedRover.transform.translation.y = roverInCharger(1, 3);
            transformStampedRover.transform.translation.z = roverInCharger(2, 3);
            transformStampedRover.transform.rotation.x = quatRover.x();
            transformStampedRover.transform.rotation.y = quatRover.y();
            transformStampedRover.transform.rotation.z = quatRover.z();
            transformStampedRover.transform.rotation.w = quatRover.w();

            br->sendTransform(transformStampedRover);

            // Transform for moving base
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header = header;
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "gps_moving_base";
            transformStamped.transform.translation.x = frontAxleInCharger(0, 3);
            transformStamped.transform.translation.y = frontAxleInCharger(1, 3);
            transformStamped.transform.translation.z = frontAxleInCharger(2, 3);
            transformStamped.transform.rotation.x = quatTheta.x();
            transformStamped.transform.rotation.y = quatTheta.y();
            transformStamped.transform.rotation.z = quatTheta.z();
            transformStamped.transform.rotation.w = quatTheta.w();

            br->sendTransform(transformStamped);


            // Transform for bus rear axis based on mapping transform
            geometry_msgs::msg::TransformStamped busRearAxisInMovingBaseTS;
            try {
                busRearAxisInMovingBaseTS = tfBuffer.lookupTransform("gps_moving_base", "bus_rear_axis", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex) {
                std::cout << "Failed to read the transformation gps_moving_base -> bus_rear_axis" << std::endl;
                return;
            }
            // Calculate it as revers transform of GPS moving base -> bus_rear_axis definied in launch
            Eigen::Affine3d busRearAxiskInMovingBase  = tf2::transformToEigen(busRearAxisInMovingBaseTS);

            // // Publish also odometry
            nav_msgs::msg::Odometry rearAxisInMap;
            
            rearAxisInMap.header.stamp = header.stamp;
            rearAxisInMap.header.frame_id = "map";
            rearAxisInMap.child_frame_id = "bus_rear_axis";

            // Calculate transform as  map->aft_mapped->bus_rear_axis
            Eigen::Affine3d frontAxleInChargerAff;
            frontAxleInChargerAff.matrix() = frontAxleInCharger;
            Eigen::Affine3d rearAxisInMapAff =  frontAxleInChargerAff * busRearAxiskInMovingBase;
            Eigen::Quaterniond quat3(rearAxisInMapAff.rotation());

            rearAxisInMap.pose.pose.orientation.x = quat3.x();
            rearAxisInMap.pose.pose.orientation.y = quat3.y();
            rearAxisInMap.pose.pose.orientation.z = quat3.z();
            rearAxisInMap.pose.pose.orientation.w = quat3.w();
            rearAxisInMap.pose.pose.position.x = rearAxisInMapAff.translation()(0);
            rearAxisInMap.pose.pose.position.y = rearAxisInMapAff.translation()(1);
            rearAxisInMap.pose.pose.position.z = rearAxisInMapAff.translation()(2);

            pubBusRearAxisInMap->publish(rearAxisInMap);
        }


        if(verbose)
            RCLCPP_INFO_STREAM(rclcpp::get_logger("your_node_name"), "DGPS XYZ: [" << frontAxleInCharger(0,3) << " " << frontAxleInCharger(1,3) << " " << frontAxleInCharger(3,3) <<
                                        "] theta: " << finalTheta * 180.0 / M_PI << " deg");
        // std::cout << "theta: " << theta * 180.0 / M_PI << std::endl;

        // Visualization
        visualizeDGPS(header, dataStatus, frontAxleInCharger);
    }

    void DGPS::visualizeDGPS(const std_msgs::msg::Header &header, const GPSStatus &status, const Eigen::Matrix4d &rearAxleInCharger) 
    {

        Eigen::Quaterniond quat(rearAxleInCharger.block<3, 3>(0, 0));

        //
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = header.stamp;
        marker.id = visGpsSeq++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
        marker.pose.position.x = rearAxleInCharger(0, 3);
        marker.pose.position.y = rearAxleInCharger(1, 3);
        marker.pose.position.z = rearAxleInCharger(2, 3);
        marker.scale.x = 0.5;
        marker.scale.y = 1.0;
        marker.scale.z = 0.0;

        // Color based on status (RED - GPS FIX, GREEN - FLOAT, BLUE - FIXED)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // lifetime
        marker.lifetime = rclcpp::Duration::from_seconds(5.0);

        // Single receiver
        if (status == GPSStatus::FIX)
            marker.color.r = marker.color.g = 1.0; // yellow
        else if (status == GPSStatus::FLOAT)
            marker.color.g = marker.color.b = 1.0; // cyan
        else if (status == GPSStatus::FIXED)
            marker.color.r = marker.color.b = 1.0; // pink

            // Moving Base
        else if (status == GPSStatus::FIXwithHeading)
            marker.color.r = 1.0; // red
        else if (status == GPSStatus::FLOATwithHeading)
            marker.color.g = 1.0; // green
        else if (status == GPSStatus::FIXEDwithHeading)
            marker.color.b = 1.0; // blue

        marker.color.a = 1.0;
        marker.points.resize(2);
        marker.points[0].x = 0.0f;
        marker.points[0].y = 0.0f;
        marker.points[0].z = 0.0f;
        marker.points[1].x = 1.0f;
        marker.points[1].y = 0.0f;
        marker.points[1].z = 0.0f;
        pubVisDGPS->publish(marker);
    }

    // bool DGPS::LLtoUTM(const sensor_msgs::msg::NavSatFix::SharedPtr &fix, double &northing, double &easting, double &altitude) 
    bool DGPS::LLtoUTM(const std::shared_ptr<sensor_msgs::msg::NavSatFix> &fix, double &northing, double &easting, double &altitude) 
    {
        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
           
            auto& clk = *this->get_clock();

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), clk, 60, "No Fix.");
            return false;
        }

        if (fix->header.stamp == rclcpp::Time(0)) {
            return false;
        }

        std::string zone;
        gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

        if (chargerZone != "" && zone != chargerZone)
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("odom_from_navrelposned"), "We might have moved outside of UTM zone! chargerZone=" << chargerZone << " zone=" << zone);


        northing = northing - charger[0];
        easting = easting - charger[1];
        altitude = -(fix->altitude - charger[2]);

        return true;
    }

    Eigen::Matrix4d DGPS::inverse(const Eigen::Matrix4d &pose) {
        Eigen::Matrix4d invPose = Eigen::Matrix4d::Identity();
        invPose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
        invPose.block<3, 1>(0, 3) = -invPose.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
        return invPose;
    }
    


}



#ifndef _GPS_ODOM_HPP__
#define _GPS_ODOM_HPP__

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/geometry_msgs/msg/quaternion_stamped.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <memory>
#include <cmath>
#include <geodesy/utm.h>

namespace lazy{
    class GpsOdomNode : public rclcpp::Node
    {
        public:
            explicit GpsOdomNode(const rclcpp::NodeOptions & node_options);
        private:
            void dataCallback(
                const nmea_msgs::msg::Sentence::ConstSharedPtr input_msg);
            
            rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr sub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    };

        struct day
        {
            int32_t hour;
            int32_t minute;
            int32_t second;
            int32_t day;
            int32_t month;
            int32_t year;
        };
        struct gps_odom
        {
            day D;
            double lattitude;
            double lattitude_d;
            double NS;
            double longitude;
            double longitude_d;
            double EW;
            double direction;
            std::string status;
        };
        struct Localxy
        {
            double X;
            double Y;
        };
        Localxy localXY;
        gps_odom *prmc_info;
        
        double origin_x = 658357.000, origin_y = 3292682.000;
}; //namespace lazy
#endif
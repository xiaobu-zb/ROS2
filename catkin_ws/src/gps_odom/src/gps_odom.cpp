#include "gps_odom/gps_odom.hpp"

namespace lazy{
    GpsOdomNode::GpsOdomNode(const rclcpp::NodeOptions & node_options)
    : Node("gps_odom", node_options)
    {
        using std::placeholders::_1;
        sub_ = create_subscription<nmea_msgs::msg::Sentence>(
            "input/gps", rclcpp::QoS{1}, std::bind(&GpsOdomNode::dataCallback,this, _1));
        pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "output/gps_odom", rclcpp::QoS{1});
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    }

    static int GetComa(int num, const char *str)
    {
        int i, j = 0;
        for(i = 0; i < strlen(str); i++)
        {
            if(str[i] == ',')
                j ++;
            if(j == num)
                return i + 1;
        }
        return 0;
    }

    static double get_double_number(const char *str)
    {
        char buf[128];
        int i;
        double rev;
        i = GetComa(1, str);
        strncpy(buf, str, i);
        buf[i] = 0;
        rev = atof(buf);
        return rev;
    }

    double DegToRad(double deg)
    {
        return (deg* M_PI /180);
    }

    double RadToDeg(double rad)
    {
        return (rad* 180/ M_PI);
    }

    void GpsOdomNode::dataCallback(
        const nmea_msgs::msg::Sentence::ConstSharedPtr input_msg)
    {
        int tmp;
        char c, b;
        const char *buf = input_msg->sentence.c_str();
        c = buf[5];
        b = buf[2];
        if(b == 'P' && c == 'C')
        {
            prmc_info->D.hour = (buf[7]- '0')*10 + (buf[8]- '0');
            prmc_info->D.minute = (buf[9]- '0')*10 + (buf[10]- '0');
            prmc_info->D.second = (buf[11]- '0')*10 + (buf[12]- '0');
            tmp = GetComa(9, buf);
            prmc_info->D.day = (buf[tmp + 0]- '0')*10 + (buf[tmp+1]- '0');
            prmc_info->D.month = (buf[tmp + 2]- '0')*10 + (buf[tmp+3]- '0');
            prmc_info->D.year = (buf[tmp + 4]- '0')*10 + (buf[tmp+5]- '0')+ 2000;
            prmc_info->status = buf[GetComa(2, buf)];
            prmc_info->lattitude = get_double_number(&buf[GetComa(3, buf)]);
            prmc_info->NS = buf[GetComa(4, buf)];
            prmc_info->longitude = get_double_number(&buf[GetComa(5, buf)]);
            prmc_info->EW = buf[GetComa(6, buf)];

            int lat_d = prmc_info->lattitude/ 100;
            int long_d = prmc_info->longitude/ 100;
            double lat_fd = (prmc_info->lattitude -lat_d* 100)/ 60;
            double long_fd = (prmc_info->longitude - long_d*100)/ 60;
            prmc_info->lattitude_d = lat_d + lat_fd;
            prmc_info->longitude_d = long_d + long_fd;
            prmc_info->direction = get_double_number(&buf[GetComa(8, buf)]);

            geographic_msgs::msg::GeoPoint gp;
            gp.latitude = prmc_info->lattitude_d;
            gp.longitude = prmc_info->longitude_d;
            geodesy::UTMPoint pt(gp);
            localXY.X = pt.easting;
            localXY.Y = pt.northing;
            double yaw = DegToRad(-(prmc_info->direction - 90));
            tf2::Quaternion qua;
            qua.setRPY(0, 0, yaw);

            nav_msgs::msg::Odometry gps_odom;
            gps_odom.header.stamp = rclcpp::Clock().now();
            gps_odom.header.frame_id = "map";
            gps_odom.child_frame_id = "gps_odom";
            gps_odom.pose.pose.position.x = localXY.X - origin_x;
            gps_odom.pose.pose.position.y = localXY.Y - origin_y;
            gps_odom.pose.pose.orientation.x = qua.x();
            gps_odom.pose.pose.orientation.y = qua.y();
            gps_odom.pose.pose.orientation.z = qua.z();
            gps_odom.pose.pose.orientation.w = qua.w();
            pub_->publish(gps_odom);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = rclcpp::Clock().now();
            transform_stamped.header.frame_id = gps_odom.header.frame_id;
            transform_stamped.child_frame_id = gps_odom.child_frame_id;
            transform_stamped.transform.translation.x = localXY.X - origin_x;
            transform_stamped.transform.translation.y = localXY.Y - origin_y;
            transform_stamped.transform.translation.z = 0;
            transform_stamped.transform.rotation.x = qua.x();
            transform_stamped.transform.rotation.y = qua.y();
            transform_stamped.transform.rotation.z = qua.z();
            transform_stamped.transform.rotation.w = qua.w();

            tf_broadcaster_->sendTransform(transform_stamped);
        }
    }

};

#include<rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lazy::GpsOdomNode)
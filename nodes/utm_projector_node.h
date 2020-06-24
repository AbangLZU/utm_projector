#pragma once

#include "proj_api.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class UtmProjector
{
private:
    /* data */
    int utm_zone_ = 99999;
    bool init_proj_;

    ros::NodeHandle nh_, private_nh_;

    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::Buffer tf2_buffer_;

    ros::Subscriber navstatfix_sub_;

    projPJ wgs84pj_source_;
    projPJ utm_target_;

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & gps_msg);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::PoseStamped & pose_msg);

public:
    UtmProjector(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~UtmProjector();

    void run();
};



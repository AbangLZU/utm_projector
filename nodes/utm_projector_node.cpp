#include "utm_projector_node.h"

UtmProjector::UtmProjector(ros::NodeHandle &nh, ros::NodeHandle &private_nh):nh_(nh),
                            private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{
    ROS_INFO_STREAM("init utm projector");
    string navfix_topic = nh.param<string>("navfix_topic", "/apollo/ros/navsatfix");
    navstatfix_sub_ = nh.subscribe(navfix_topic, 100, &UtmProjector::gps_callback, this);

    init_proj_ = false;
}

UtmProjector::~UtmProjector()
{
}

void UtmProjector::gps_callback(const sensor_msgs::NavSatFix::ConstPtr & gps_msg){
    // convert degree to rad
    double x = gps_msg->longitude;
    double y = gps_msg->latitude;
    x = x * M_PI / 180.0;
    y = y * M_PI / 180.0;

    int zone = static_cast<int>((x * RAD_TO_DEG + 180) / 6) + 1;

    if((!init_proj_) || (utm_zone_!=zone)){
      // init the proj4 source and target
      string wgs84 = "+proj=latlong +ellps=WGS84";
      std::ostringstream proj4_text;
      proj4_text<<"+proj=utm +zone="<<zone<<" +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";

      ROS_INFO_STREAM("project source: "<<wgs84);
      ROS_INFO_STREAM("project target: "<<proj4_text.str());

      wgs84pj_source_ = pj_init_plus(wgs84.c_str());
      utm_target_ = pj_init_plus(proj4_text.str().c_str());
      utm_zone_ = zone;
      init_proj_ = true;
    }

    pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

    geometry_msgs::PoseStamped output_msg;
    output_msg.pose.position.x = x;
    output_msg.pose.position.y = y;
    output_msg.pose.position.z = gps_msg->altitude;
    output_msg.pose.orientation.w = 1.0;
    output_msg.pose.orientation.x = 0.0;
    output_msg.pose.orientation.y = 0.0;
    output_msg.pose.orientation.z = 0.0;
    output_msg.header = gps_msg->header;

    publish_tf("world", "gnss", output_msg);
}

void UtmProjector::run(){
}

void UtmProjector::publish_tf(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "utm_projector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    UtmProjector utm_projecter(nh, private_nh);
    utm_projecter.run();

    ros::spin();

    return 0;
}

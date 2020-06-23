#include "utm_projector_node.h"

UtmProjector::UtmProjector(ros::NodeHandle &nh, ros::NodeHandle &private_nh):nh_(nh),
                            private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{
    string wgs84 = "+proj=latlong +ellps=WGS84";
    string proj4_text = "+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
    wgs84pj_source_ = pj_init_plus(wgs84.c_str());
    utm_target_ = pj_init_plus(proj4_text.c_str());

    ROS_INFO_STREAM("init utm projector");
    string navfix_topic = nh.param<string>("navfix_topic", "/apollo/ros/navsatfix");
    navstatfix_sub_ = nh.subscribe(navfix_topic, 100, &UtmProjector::gps_callback, this);

}

UtmProjector::~UtmProjector()
{
}

void UtmProjector::gps_callback(const sensor_msgs::NavSatFix::ConstPtr & gps_msg){
    // auto utm_xy = latlon_to_utm_xy(gps_msg->longitude, gps_msg->latitude);

    // double lon_rad = -2.129343746458001;
    // double lat_rad = 0.6530018835651807;
    // // UTMCoor utm_xy;
    // // EXPECT_TRUE(FrameTransform::LatlonToUtmXY(lon_rad, lat_rad, &utm_xy));
    // // EXPECT_LT(std::fabs(utm_xy.x - 588278.9834174265), 1e-5);
    // // EXPECT_LT(std::fabs(utm_xy.y - 4141295.255870659), 1e-5);

    // auto utm_xy = latlon_to_utm_xy(lon_rad, lat_rad);


    double x = gps_msg->longitude;
    double y = gps_msg->latitude;
    x *= DEG_TO_RAD_LOCAL;
    y *= DEG_TO_RAD_LOCAL;

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

    publish_tf("world", "adam", output_msg);
}

void UtmProjector::run(){
}

pair<double, double> UtmProjector::latlon_to_utm_xy(double lon_rad, double lat_rad) {
  projPJ pj_latlon;
  projPJ pj_utm;
  int zone = 0;
  zone = static_cast<int>((lon_rad * RAD_TO_DEG + 180) / 6) + 1;

  ROS_INFO_STREAM("utm zone is: "<< zone);
  std::string latlon_src =
      "+proj=longlat +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::ostringstream utm_dst;
  utm_dst << "+proj=utm +zone=" << 50 << " +ellps=WGS84 +units=m +no_defs";
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return pair<double, double>(0.0, 0.0);
  }
  if (!(pj_utm = pj_init_plus(utm_dst.str().c_str()))) {
    return pair<double, double>(0.0, 0.0);
  }
  double longitude = lon_rad;
  double latitude = lat_rad;
  pj_transform(pj_latlon, pj_utm, 1, 1, &longitude, &latitude, nullptr);
  pair<double, double> utm_result;
  utm_result.first = longitude; // x
  utm_result.second = latitude; // y

  pj_free(pj_latlon);
  pj_free(pj_utm);
  return utm_result;
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

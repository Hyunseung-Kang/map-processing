#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include "visualization_msgs/Marker.h"


ros::Publisher pub_pose_;

geometry_msgs::Pose2D pose2d;

double pose_x;
double pose_y;
double ori_x;
double ori_y;
double ori_z;
double ori_w;


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_x= msg->pose.pose.position.x;
  pose_y = msg->pose.pose.position.y;
  ori_x = msg->pose.pose.orientation.x;
  ori_y = msg->pose.pose.orientation.y;
  ori_z = msg->pose.pose.orientation.z;
  ori_w = msg->pose.pose.orientation.w;
  // tf::Quaternion q(
  //   msg->pose.pose.orientation.x,
  //   msg->pose.pose.orientation.y,
  //   msg->pose.pose.orientation.z,
  //   msg->pose.pose.orientation.w
  // );
  // tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // pose_theta = yaw;
  // std::cout<<"yaw: "<<yaw<<"  pitch: "<<pitch<<"  roll: "<<roll<<std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/t265/odom/sample", 1000, poseCallback);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::SPHERE;


  visualization_msgs::Marker marker;
  marker.header.frame_id = "/t265_odom_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose_x;
  marker.pose.position.y = pose_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = ori_x;
  marker.pose.orientation.y = ori_y;
  marker.pose.orientation.z = ori_z;
  marker.pose.orientation.w = ori_w;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  
  ros::Rate loop_rate(0.5);
  while(nh.ok()){
    ros::spinOnce();
      marker.pose.position.x = pose_x;
  marker.pose.position.y = pose_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = ori_x;
  marker.pose.orientation.y = ori_y;
  marker.pose.orientation.z = ori_z;
  marker.pose.orientation.w = ori_w;
    marker_pub.publish(marker);

    loop_rate.sleep();
  }
}
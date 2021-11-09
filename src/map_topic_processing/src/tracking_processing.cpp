#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher pub_pose_;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose2D pose2d;
  pose2d.x = msg->pose.pose.position.x;
  pose2d.y = msg->pose.pose.position.y;
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose2d.theta = yaw;
  std::cout<<"yaw: "<<yaw<<"  pitch: "<<pitch<<"  roll: "<<roll<<std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/t265/odom/sample", 1000, poseCallback);

  ros::Rate loop_rate(0.5);
  while(nh.ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}
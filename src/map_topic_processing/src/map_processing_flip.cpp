#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<iostream>
#include<opencv2/opencv.hpp>
//#include<opencv2/highgui/highgui.hpp>
#include "nav_msgs/GetMap.h"
#include "image_transport/image_transport.h"
#include "grid_map_msgs/GridMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

using namespace cv;

cv::Mat input_map;
cv::Mat output_map;
ros::Publisher pub_pose_;
double cam_yaw;

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
  cam_yaw = yaw;
}


void mapConvert(const nav_msgs::OccupancyGrid& msg){
    try{
        std::cout<<"msg.MetaData: "<<msg.info<<std::endl;
        std::cout<<"msg.MetaData.width: "<<msg.info.width<<std::endl;
        std::cout<<"msg.MetaData.height: "<<msg.info.height<<std::endl;
        std::cout<<"msg.header: "<<msg.header<<std::endl;
        input_map = cv::Mat(msg.info.height, msg.info.width, CV_8UC1);

        for(int i=0; i<msg.info.height; i++){
            for(int j=0; j<msg.info.width; j++){
                int count = i*msg.info.width + j;
                if(msg.data[count] <= 0)
                    input_map.at<uchar>(i, j) = 255;
                else{
                    //std::cout<<"     valid data: "<<int(msg.data[count]);
                    input_map.at<uchar>(i, j) = msg.data[count];
                }
            }
        }
        //// flip 함수는 param이 양수이면 좌우반전, 0이면 상하반전, 음수이면 상하+좌우
        flip(input_map, input_map, 1);
        double angle = -90 - cam_yaw*180/CV_PI;
        std::cout<<"angle: "<<angle<<std::endl;
        Point2f center((input_map.cols-1)/2.0, (input_map.rows-1)/2.0);
        Mat rot = getRotationMatrix2D(center, angle, 1.0);
        Rect2f bbox = RotatedRect(Point2f(), input_map.size(), angle).boundingRect2f();
        rot.at<double>(0, 2) += bbox.width/2.0 - input_map.cols/2.0;
        rot.at<double>(1, 2) += bbox.height/2.0 - input_map.rows/2.0; 
        warpAffine(input_map, output_map, rot, bbox.size());
        //warpAffine(input_map, output_map, rot, Size(input_map.cols, input_map.rows));
    // 이 부분에 노이즈 제거를 포함시키고 바닥 정보를 제거할 수 있도록 하자
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from");
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "opencv_version_test");
    ros::NodeHandle nh;
    std::cout<<"OpenCV Version: "<<CV_VERSION<<std::endl;
    image_transport::ImageTransport it(nh);


    image_transport::Publisher pub1 = it.advertise("A/image", 1);
    image_transport::Publisher pub2 = it.advertise("B/image", 1);

    //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();

    ros::Subscriber sub = nh.subscribe("/grid_map_visualization/elevation_grid", 1000, mapConvert);
    //ros::spin();
    ros::Subscriber sub_tracking = nh.subscribe("/t265/odom/sample", 1000, poseCallback);

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);


    ros::Rate loop_rate(5);
    while(nh.ok()){
        ros::spinOnce();
        cv::equalizeHist(input_map, input_map);
        if(input_map.cols != 0 && input_map.rows!=0){
            imshow("view", input_map);
            imshow("out", output_map);
            waitKey(5);
        }
        
        cv_ptr->encoding = "mono8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "/AA";
        cv_ptr->image = input_map;


        cv_ptr2->encoding = "mono8";
        cv_ptr2->header.stamp = ros::Time::now();
        cv_ptr2->header.frame_id = "/BB";
        cv_ptr2->image = output_map;

        pub1.publish(cv_ptr->toImageMsg());
        pub2.publish(cv_ptr2->toImageMsg());
        
        // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();
        // pub.publish(img_msg);
        loop_rate.sleep();
    }
    return 0;
}
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

cv::Mat input_map;
using namespace cv;


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
    // 이 부분에 노이즈 제거를 포함시키고 바닥 정보를 제거할 수 있도록 하자
    }
    catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from");
  }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "opencv_version_test");
    ros::NodeHandle nh;
    std::cout<<"OpenCV Version: "<<CV_VERSION<<std::endl;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("AA/image", 1);

    //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();

    ros::Subscriber sub = nh.subscribe("/grid_map_visualization/elevation_grid", 1000, mapConvert);
    //ros::spin();

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    ros::Rate loop_rate(5);
    while(nh.ok()){
        ros::spinOnce();
        cv::equalizeHist(input_map, input_map);
        if(input_map.cols != 0 && input_map.rows!=0){
            imshow("view", input_map);
            waitKey(5);
        }
        cv_ptr->encoding = "mono8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "/AAAID";
        cv_ptr->image = input_map;
        pub.publish(cv_ptr->toImageMsg());
        
        // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();
        // pub.publish(img_msg);
        loop_rate.sleep();
    }
    return 0;
}
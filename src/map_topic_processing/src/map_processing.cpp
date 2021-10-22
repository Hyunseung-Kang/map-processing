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


void mapConvert(const nav_msgs::OccupancyGrid& msg){
    std::cout<<"msg.MetaData: "<<msg.info<<std::endl;
    std::cout<<"msg.MetaData.width: "<<msg.info.width<<std::endl;
    std::cout<<"msg.MetaData.height: "<<msg.info.height<<std::endl;
    std::cout<<"msg.header: "<<msg.header<<std::endl;
    input_map = cv::Mat(msg.info.height, msg.info.width, CV_8UC1);

    for(int i=0; i<msg.info.width; i++){
        for(int j=0; j<msg.info.height; j++){
            int count = i*j;
            if(msg.data[count] <= 0)
                input_map.at<uchar>(j, i) = 0;
            else{
                //std::cout<<"     valid data: "<<int(msg.data[count]);
                input_map.at<uchar>(j, i) = msg.data[count];
            }
        }
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "opencv_version_test");
    ros::NodeHandle nh;
    std::cout<<"OpenCV Version: "<<CV_VERSION<<std::endl;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("AA/image", 1);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();

    ros::Subscriber sub = nh.subscribe("/grid_map_visualization/elevation_grid", 1000, mapConvert);

    ros::Rate loop_rate(5);
    while(nh.ok()){
        ros::spinOnce();
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", input_map).toImageMsg();
        pub.publish(img_msg);
        loop_rate.sleep();
    }
    return 0;
}
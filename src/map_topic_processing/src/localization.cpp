#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>

#include <stdlib.h>

#include "nav_msgs/GetMap.h"
#include "image_transport/image_transport.h"
#include "grid_map_msgs/GridMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include <vector>


using namespace cv;
using namespace std;

#define FOV 160

geometry_msgs::Pose2D Initial_pose2d;   // 지정받는 초기 위치를 저장
geometry_msgs::Pose2D Robot_pose;       // 현재 로봇의 위치를 나타낼 좌표

geometry_msgs::Pose2D cam_prev_pose;
geometry_msgs::Pose2D cam_curr_pose;

geometry_msgs::PoseArray particles;     // 파티클의 정보를 표현하기 위한 변수

std::vector<std::vector<double> > samples;


ros::Subscriber initial_pose_sub_;      // 초기 위치의 메시지를 subscribe
ros::Subscriber map_sub;                // 센서로 취득되는 local map을 구독
ros::Subscriber pose_sub;
ros::Publisher marker_pub;
ros::Publisher pose_array_particles_pub;


void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
void initialize_particle_update(double x, double y, double theta, geometry_msgs::PoseArray* particles);
void get_marker_pose(visualization_msgs::Marker* marker);
void particle_weight_update(Mat* global_map, Mat* measured_map);
double map_matching(Mat* global_map, Mat* local_map);
void mapConvert(const nav_msgs::OccupancyGrid& msg);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
void particle_motion_update(geometry_msgs::Pose2D diff_pose);
void floor_remove(Mat* map, int floor_threshold);
std::vector<std::vector<double> > resampling(geometry_msgs::PoseArray* particles);


cv::Mat input_map;    // 센서로 취득되는 데이터
cv::Mat output_map;   // 취득 지도를 반전시키고 변형하여 출력시킬 데이터
cv::Mat global_map;


double initial_theta = 0.0;
double total_weight = 0.0;
int initial_pose_flag = 0;
double gaussian = sqrt((-2)*log(0.2*sqrt(2*CV_PI)));


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle nh;

  Robot_pose.x = 0.0;
  Robot_pose.y = 0.0;
  Robot_pose.theta = 0.0;
  global_map = cv::imread("aa.png");

  image_transport::ImageTransport it(nh);

  map_sub = nh.subscribe("/grid_map_visualization/elevation_grid", 1000, mapConvert);
  pose_sub = nh.subscribe("/t265/odom/sample", 1000, poseCallback);
  initial_pose_sub_ = nh.subscribe("initialpose", 2, initialPoseReceived);


  marker_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 1);
  pose_array_particles_pub = nh.advertise<geometry_msgs::PoseArray> ("particle_poses", 1);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  //uint32_t shape = visualization_msgs::Marker::SPHERE;
  uint32_t shape = visualization_msgs::Marker::ARROW;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/t265_odom_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  particles.header.stamp = ros::Time::now();
  particles.header.frame_id = "/t265_odom_frame";

  marker.scale.x = 1.0;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  double marker_ori_x, marker_ori_y, marker_ori_z, marker_ori_w;

  tf::Quaternion marker_quat;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  marker_quat = tf::createQuaternionFromYaw(Robot_pose.theta);
  marker.pose.orientation.x = marker_quat[0];
  marker.pose.orientation.y = marker_quat[1];
  marker.pose.orientation.z = marker_quat[2];
  marker.pose.orientation.w = marker_quat[3];
  
  ros::Rate loop_rate(0.5);
  while(nh.ok()){
    ros::spinOnce();
    // marker.pose.position.x = Robot_pose.x;
    // marker.pose.position.y = Robot_pose.y;

    get_marker_pose(&marker);
    marker_pub.publish(marker);
    pose_array_particles_pub.publish(particles);
    loop_rate.sleep();
  }
}


void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double temp_x, temp_y, temp_z, temp_w;

  if(initial_pose_flag == 0){
  //curr_pose_x = msg->pose.pose.position.x;
  //curr_pose_y = msg->pose.pose.position.y;
  cam_curr_pose.x = msg->pose.pose.position.x;
  cam_curr_pose.y = msg->pose.pose.position.y;
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cam_curr_pose.theta = yaw;
  }
}



















//// OccupancyGrid의 msg는 sensor data와 같다.
//// 즉, 센서 데이터가 취득되면 이에 대하여 particle을 업데이트하여
//// 재분포한다.
//// 원본 AMCL은 임계값 이상의 거리, 회전 발생 시 particle을 업데이트하지만
//// 나는 sensor data를 수신하면 particle을 업데이트 한다.
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
        double angle = -90 - cam_curr_pose.theta*180/CV_PI;
        std::cout<<"angle: "<<angle<<std::endl;
        Point2f center((input_map.cols-1)/2.0, (input_map.rows-1)/2.0);
        Mat rot = getRotationMatrix2D(center, angle, 1.0);
        Rect2f bbox = RotatedRect(Point2f(), input_map.size(), angle).boundingRect2f();
        rot.at<double>(0, 2) += bbox.width/2.0 - input_map.cols/2.0;
        rot.at<double>(1, 2) += bbox.height/2.0 - input_map.rows/2.0; 
        warpAffine(input_map, output_map, rot, bbox.size());
        //warpAffine(input_map, output_map, rot, Size(input_map.cols, input_map.rows));

        geometry_msgs::Pose2D diff_pose;        // n-1번째 데이터를 수신하고 현재 데이터를 수신하기까지
                                  // 변한 위치 및 방향 정보
        diff_pose.x = cam_curr_pose.x - cam_prev_pose.x;
        diff_pose.y = cam_curr_pose.y - cam_prev_pose.y;
        diff_pose.theta = cam_curr_pose.theta - cam_prev_pose.theta;
        if(abs(diff_pose.x) > 0.01 || abs(diff_pose.y) > 0.01){
          particle_motion_update(diff_pose);
          cam_prev_pose = cam_curr_pose;
        }
        /// weight를 업데이트하기 전, global map과 local ma에서 바닥정보를 없애주자.
        floor_remove(&global_map, 120);
        floor_remove(&output_map, 120);

        // global_map은 2.5차원 Grayscale의 전체 지도, output_map은 취득 데이터
        particle_weight_update(&global_map, &output_map);
        samples = resampling(&particles);

    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from");
    }
}

std::vector<std::vector<double> > resampling(geometry_msgs::PoseArray* particles){

  particles->poses.clear();
  geometry_msgs::Pose p;
  tf::Quaternion ori;
  double random_theta;

  std::vector<std::vector<double> > new_samples;

  for(int i=0; i<samples.size(); i++){
    double new_weight = (double)rand()*total_weight/(RAND_MAX);
    int start_pt = 0;
    double temp_weight = 0.0;
    while(new_weight >= temp_weight){
      temp_weight += samples[start_pt][3];
      start_pt += 1;
    }

    p.position.x = samples[start_pt][0] + (double)rand()/(RAND_MAX/gaussian)*pow(-1, rand());
    p.position.y = samples[start_pt][1] + (double)rand()/(RAND_MAX/gaussian)*pow(-1, rand());
    p.position.z = 0;
    random_theta = samples[start_pt][2] + ((CV_PI/3)*((double)rand()/RAND_MAX))*pow(-1, rand());
    ori = tf::createQuaternionFromYaw(random_theta);
    p.orientation.x = ori[0];
    p.orientation.y = ori[1];
    p.orientation.z = ori[2];
    p.orientation.w = ori[3];
    particles->poses.push_back(p);

    new_samples.push_back(samples[start_pt]);

    //new_samples.push_back(temp);
  }
  return new_samples;
}



















void floor_remove(Mat* map, int floor_threshold){
  int pixel_data;
  for(int i=0; i<map->cols; i++){
    for(int j=0; j<map->rows; j++){
      pixel_data = map->at<uchar>(j, i);
      if(pixel_data < floor_threshold)
        map->at<uchar>(j, i) = 0;
    }
  }
}


void particle_weight_update(Mat* global_map, Mat* measured_map){
  Mat segment_map = Mat::zeros(100, 50, CV_8UC1);
  double x, y;
  double theta;
  total_weight = 0;
  for(int i=0; i<samples.size(); i++){
    x = samples[i][0];
    y = samples[i][1];
    theta = samples[i][2];
    //// sample의 각도가 0이면 이는 3시방향을 바라보고 있는 것이므로 90도를 빼준다.
    Mat rot = cv::getRotationMatrix2D(cv::Point2f(x, y), theta-(CV_PI/2), 1.0);
    cv::Mat rotate_global_map;
    cv::warpAffine(*global_map, rotate_global_map, rot, Size(global_map->cols, global_map->rows));
    for(int segment_map_i=0; segment_map_i<segment_map.rows-1; segment_map_i++){
      for(int segment_map_j = 0; segment_map_j<segment_map.cols-1; segment_map_j++){
        int col_value = (segment_map.cols/2)-segment_map_j;
        if((atan((double)col_value / (segment_map.rows-i)) < (FOV/2)) && (atan((double)col_value/(segment_map.rows-i)) > -(FOV/2))){
          segment_map.at<uchar>(segment_map_i, segment_map_j) = rotate_global_map.at<uchar>(y-segment_map.rows+i, x-col_value);
        }
        else{
          segment_map.at<uchar>(segment_map_i, segment_map_j) = 0;
        }
      }
    }
    samples[i][3] = map_matching(&segment_map, measured_map);
    total_weight += samples[i][3];
  }
}



double map_matching(Mat* global_map, Mat* local_map){
  int width, height;
  if(global_map->cols > local_map->cols)
    width = global_map->cols;
  else
    width = local_map->cols;
  
  if(global_map->rows > local_map->rows)
    height = global_map->rows;
  else
    height = local_map->rows;

  //// template1에는 global 지도의 segment가 들어가고 template2에는 mewsured map이 들어간다.
  cv::Mat template1 = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat template2 = cv::Mat::zeros(height, width, CV_8UC1);
  for(int i=0; i<width; i++){
    for(int j=0; j<height; j++){
      if((i > global_map->cols) || (j<(height-global_map->rows)))
        template1.at<uchar>(j, i) = 0;
      else
        template1.at<uchar>(j, i) = global_map->at<uchar>(j-(height-global_map->rows), i);

      if((i > local_map->cols) || (j<(height-local_map->rows)))
        template2.at<uchar>(j, i) = 0;
      else
        template2.at<uchar>(j, i) = local_map->at<uchar>(j-(height-local_map->rows), i);
    }
  }

  int count = 0;
  int total_valid_pixel = 0;
  for(int i=0; i<width; i++){
    for(int j=0; j<height; j++){
      if(template2.at<uchar>(j, i) != 0){
        total_valid_pixel += 1;
        if(template1.at<uchar>(j, i) != 0){
          count += 1;
        }
      }
    }
  }
  double result = (double)count/total_valid_pixel;
  return result;
}






void particle_motion_update(geometry_msgs::Pose2D diff_pose){
  tf::Quaternion ori;
  for(int i=0; i<samples.size(); i++){
    double new_theta = samples[i][2] + diff_pose.theta;
    double new_x = samples[i][0] + diff_pose.x*cos(new_theta) - diff_pose.y*sin(new_theta);
    double new_y = samples[i][1] + diff_pose.x*sin(new_theta) + diff_pose.y*cos(new_theta);
    

    samples[i][0] = new_x;
    samples[i][1] = new_y;
    samples[i][2] = new_theta;

    ori = tf::createQuaternionFromYaw(new_theta);
    particles.poses[i].position.x = samples[i][0];
    particles.poses[i].position.y = samples[i][1];
    particles.poses[i].orientation.x = ori[0];
    particles.poses[i].orientation.y = ori[1];
    particles.poses[i].orientation.z = ori[2];
    particles.poses[i].orientation.w = ori[3];
  }
}



void get_marker_pose(visualization_msgs::Marker* marker){
  // marker->pose.position.x = Initial_pose2d.x + curr_pose_x;
  // marker->pose.position.y = Initial_pose2d.y + curr_pose_y;
  // double x = Initial_pose2d.x + (curr_pose_x * cos(initial_theta)) - (sin(initial_theta)*curr_pose_y);
  // double y = Initial_pose2d.x + (curr_pose_x * sin(initial_theta)) + (cos(initial_theta)* curr_pose_y);
  marker->pose.position.x = Robot_pose.x + cam_curr_pose.x*cos(initial_theta) - cam_curr_pose.y*sin(initial_theta);
  marker->pose.position.y = Robot_pose.y + cam_curr_pose.x*sin(initial_theta) + cam_curr_pose.y*cos(initial_theta);
  // marker->pose.position.x = x;
  // marker->pose.position.y = y;
  marker->pose.position.z = 0;
  tf::Quaternion marker_rot;
  marker_rot = tf::createQuaternionFromYaw(Robot_pose.theta + cam_curr_pose.theta);
  marker->pose.orientation.x = marker_rot[0];
  marker->pose.orientation.y = marker_rot[1];
  marker->pose.orientation.z = marker_rot[2];
  marker->pose.orientation.w = marker_rot[3];
}


void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  Initial_pose2d.x = msg->pose.pose.position.x;
  Initial_pose2d.y = msg->pose.pose.position.y;
  tf::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y, 
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  Initial_pose2d.theta = yaw;
  initialize_particle_update(Initial_pose2d.x, Initial_pose2d.y, Initial_pose2d.theta, &particles);
  // for(int i=0; i<samples.size(); i++){
  //   cout<<"samples["<<i<<"][0]: "<<samples[i][0]<<endl;
  // }
  Robot_pose.x = Initial_pose2d.x;
  Robot_pose.y = Initial_pose2d.y;
  Robot_pose.theta = Initial_pose2d.theta;
  cam_prev_pose = cam_curr_pose;
}



void initialize_particle_update(double x, double y, double theta, geometry_msgs::PoseArray* particles){

  geometry_msgs::Pose p;
  tf::Quaternion ori;
  
  double random_theta;

  particles->poses.clear();
  for(int i=0; i<20; i++){
    vector<double> sample_vector; //      [0]:x     [1]:y   [2]:theta   [3]:weight
    //srand(time(NULL));
    // p.position.x = x + (double)rand()/(RAND_MAX/0.5)*pow(-1, rand());
    // p.position.y = y + (double)rand()/(RAND_MAX/0.5)*pow(-1, rand());
    
    p.position.x = x + (double)rand()/(RAND_MAX/gaussian)*pow(-1, rand());
    p.position.y = y + (double)rand()/(RAND_MAX/gaussian)*pow(-1, rand());
    p.position.z = 0;
    random_theta = theta + ((CV_PI/2)*((double)rand()/RAND_MAX))*pow(-1, rand());
    //cout<<"theta: "<<theta<<"|\t"<<((CV_PI/8)*((double)rand()/RAND_MAX))*pow(-1, rand())<<endl;
    //random_theta = theta + CV_PI/2;

    // x, y, yaw, weight
    sample_vector.push_back(p.position.x);
    sample_vector.push_back(p.position.y);
    sample_vector.push_back(random_theta);
    sample_vector.push_back(0.0);
    samples.push_back(sample_vector);

    ori = tf::createQuaternionFromYaw(random_theta);
    //ori = tf::createQuaternionFromYaw(theta);
    //cout<<"particles theta: "<<theta<<endl;
    initial_theta = theta;
    p.orientation.x = ori[0];
    p.orientation.y = ori[1];
    p.orientation.z = ori[2];
    p.orientation.w = ori[3];
    particles->poses.push_back(p);
  }
}
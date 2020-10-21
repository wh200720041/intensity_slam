// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubCornerPointsSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_corner(new pcl::PointCloud<pcl::PointXYZI>());    
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_rest(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            //laserProcessing.preFiltering(pointcloud_in, pointcloud_filtered);
            laserProcessing.featureExtraction(pointcloud_in, pointcloud_corner, pointcloud_surf, pointcloud_rest);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            *pointcloud_filtered += *pointcloud_corner;
            *pointcloud_filtered += *pointcloud_surf;
            *pointcloud_filtered += *pointcloud_rest;
            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "/base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 cornerPointsMsg;
            pcl::toROSMsg(*pointcloud_corner, cornerPointsMsg);
            cornerPointsMsg.header.stamp = pointcloud_time;
            cornerPointsMsg.header.frame_id = "/base_link";
            pubCornerPointsSharp.publish(cornerPointsMsg);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_rest(new pcl::PointCloud<pcl::PointXYZI>());
            *pointcloud_surf_rest += *pointcloud_rest;
            *pointcloud_surf_rest += *pointcloud_surf;
            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf_rest, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "/base_link";
            pubSurfPointsFlat.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}
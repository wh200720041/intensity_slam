// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.h"

//points covariance class
class Double3d{
public:
	int id;
	double angle;
	double value;
	Double3d(int id_in, double angle_in, double value_in);
};


//这里所有的数据传递全部采用指针或引用的形式来提高传递效率
class LaserProcessingClass 
{
    public:
    	LaserProcessingClass();
		void init(lidar::Lidar lidar_param_in);
		void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_corner, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_rest);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double3d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_corner, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_rest);	
		void intensityCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in);
	private:
     	lidar::Lidar lidar_param;

     	pcl::CropBox<pcl::PointXYZI> closePointFilter;
     	pcl::CropBox<pcl::PointXYZI> farPointFilter;
     	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
};



#endif // _LASER_PROCESSING_CLASS_H_

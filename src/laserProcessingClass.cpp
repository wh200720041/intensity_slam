// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"
#include "ros/ros.h"
void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

    //init filter
    double x_min = -lidar_param.min_distance, y_min = -lidar_param.min_distance, z_min = -lidar_param.min_distance;
    double x_max = +lidar_param.min_distance, y_max = +lidar_param.min_distance, z_max = +lidar_param.min_distance;
    closePointFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    closePointFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    closePointFilter.setNegative(true);
    
    x_min = -lidar_param.max_distance;
    y_min = -lidar_param.max_distance;
    z_min = -lidar_param.max_distance;
    x_max = +lidar_param.max_distance;
    y_max = +lidar_param.max_distance;
    z_max = +lidar_param.max_distance;
    farPointFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    farPointFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    farPointFilter.setNegative(false);

}

void LaserProcessingClass::intensityCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in){
    for(int i=0;i<pc_in->points.size();i++){
        //laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        if(pc_in->points[i].z<-1.1){
            if(pc_in->points[i].intensity <0.05)
                pc_in->points[i].intensity =0.05;
            double distance_temp = pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y;
            double distance_ratio = sqrt(pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z);
            double ratio = fabs(sqrt(distance_temp)/pc_in->points[i].z);
            double angle = atan(ratio); 
            if (angle<M_PI/18) angle = M_PI/18;
            if (angle>M_PI/7.5) angle = M_PI/7.5;
            double new_intensity = pc_in->points[i].intensity * cos(angle)/cos(M_PI/7.5);
            pc_in->points[i].intensity = new_intensity;
            if(pc_in->points[i].intensity >1)
                pc_in->points[i].intensity =1;
            //pc_in->points[i].intensity =1.0;
        }
    }

    return;
}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_corner, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_rest){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    intensityCalibration(pc_in);

    int N_SCANS = lidar_param.num_lines;
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    std::vector<std::vector<Double3d>> laser_id;
    for(int i=0;i<N_SCANS*2;i++){
        //laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        std::vector<Double3d> id_arr_temp;
        laser_id.push_back(id_arr_temp);
    }
    
    int current_id=0;
    double last_angle =0.0;
    int disable_count =0;
    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        int scanID=0;
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
        double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
        double current_angle = atan2(pc_in->points[i].y, pc_in->points[i].x);
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.70){
                scanID = int((2 - angle) * 3 + 0.5);
            }else if(angle< -8.70){
                scanID = N_SCANS / 2 + int((-8.90 - angle) * 2 + 0.5);
            }
        }
        else
        {
            printf("wrong scan number\n");
        }

        if(disable_count>0)
            disable_count--;
        if(last_angle<0.0 && current_angle>=0.0 && disable_count==0){
            current_id++;
            disable_count=20;
            //printf("current_size %d, %d",current_id-1, (int)laser_id[current_id-1].size());
        }else if(last_angle>current_angle+0.2 && last_angle<current_angle+3.1 && current_id<N_SCANS-10){
            printf("error scan input %d,%f, %f, \r\n",current_id,last_angle,current_angle);
        }
        last_angle = current_angle;

        if(current_id>=N_SCANS*2){
            printf("error scan number please change");
            continue;
        }
        //point filtering
        if (angle > 2.33 || angle < -24.33 || scanID>N_SCANS || scanID<0)
            continue;
        //distance
        if(distance<lidar_param.min_distance)
            continue;
        if(distance>lidar_param.max_distance)
            continue;
        //kitti dataset has some error points
        if(pc_in->points[i].z < -5.0)
            continue;
        //for kitti datasaet only because other sensor is detected 
        if(distance < 3.0 && pc_in->points[i].z > -1.0)
            continue;

        Double3d point_id(i, current_angle, distance);
        laser_id[current_id].push_back(point_id);

    }

    for(int i = 0; i < N_SCANS*2; i++){
        if(laser_id[i].size()<251){
            continue;
        }


        for(int j =0; j<(int)laser_id[i].size(); j++){

            //laserCloudScans[i]->points[j].intensity = ((double)j)/laser_id[i].size();
            if( j<5 || j>=laser_id[i].size()-5 ){
                laser_id[i][j].value =-1.0;
            }else if(fabs(laser_id[i][j+5].angle - laser_id[i][j-5].angle) > M_PI*1.8/180){
                laser_id[i][j].value =-1.0;
            }else{
                double diffX = pc_in->points[laser_id[i][j-5].id].x + pc_in->points[laser_id[i][j-4].id].x + pc_in->points[laser_id[i][j-3].id].x + pc_in->points[laser_id[i][j-2].id].x + pc_in->points[laser_id[i][j-1].id].x - 10 * pc_in->points[laser_id[i][j].id].x + pc_in->points[laser_id[i][j+1].id].x + pc_in->points[laser_id[i][j+2].id].x + pc_in->points[laser_id[i][j+3].id].x + pc_in->points[laser_id[i][j+4].id].x + pc_in->points[laser_id[i][j+5].id].x;
                double diffY = pc_in->points[laser_id[i][j-5].id].y + pc_in->points[laser_id[i][j-4].id].y + pc_in->points[laser_id[i][j-3].id].y + pc_in->points[laser_id[i][j-2].id].y + pc_in->points[laser_id[i][j-1].id].y - 10 * pc_in->points[laser_id[i][j].id].y + pc_in->points[laser_id[i][j+1].id].y + pc_in->points[laser_id[i][j+2].id].y + pc_in->points[laser_id[i][j+3].id].y + pc_in->points[laser_id[i][j+4].id].y + pc_in->points[laser_id[i][j+5].id].y;
                double diffZ = pc_in->points[laser_id[i][j-5].id].z + pc_in->points[laser_id[i][j-4].id].z + pc_in->points[laser_id[i][j-3].id].z + pc_in->points[laser_id[i][j-2].id].z + pc_in->points[laser_id[i][j-1].id].z - 10 * pc_in->points[laser_id[i][j].id].z + pc_in->points[laser_id[i][j+1].id].z + pc_in->points[laser_id[i][j+2].id].z + pc_in->points[laser_id[i][j+3].id].z + pc_in->points[laser_id[i][j+4].id].z + pc_in->points[laser_id[i][j+5].id].z;
                laser_id[i][j].value =(diffX * diffX + diffY * diffY + diffZ * diffZ) / std::max(1.0,log10(laser_id[i][j].value)) ;
            }
            
        }

        for(int j=0;j<6;j++){
            int sector_length = (int)(laser_id[i].size()/6);
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = laser_id[i].size() - 1; 
            }
            std::vector<Double3d> cloudCurvature(laser_id[i].begin()+sector_start,laser_id[i].begin()+sector_end); 
            
            featureExtractionFromSector(pc_in, cloudCurvature, pc_out_corner, pc_out_surf, pc_out_rest);
            
        }
        

    }


}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double3d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_corner, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_rest){

    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double3d & a, const Double3d & b)
    { 
        return a.value < b.value; 
    });

    
    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.3 ){ //0.006343 0.000058
                break;
            }
            if(pc_in->points[ind].z<-0.9){
                bool is_ground = false;
                double dis1 = pc_in->points[ind].x * pc_in->points[ind].x + pc_in->points[ind].y * pc_in->points[ind].y + pc_in->points[ind].z * pc_in->points[ind].z;
                for(int k=-5;k<=5;k++){
                    if(k==0)
                        continue;
                    double dis2 = pc_in->points[ind+k].x * pc_in->points[ind+k].x + pc_in->points[ind+k].y * pc_in->points[ind+k].y + pc_in->points[ind+k].z * pc_in->points[ind+k].z;
                    if (dis2 < dis1 * 0.7){
                        is_ground = true;
                        break;
                    }
                }
                if(is_ground)
                    continue;
            }
                        
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if (largestPickedNum <= cloudCurvature.size()/13){
                pc_out_corner->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

        }
    }
    


    //find flat points
    point_info_count =0;
    int smallestPickedNum = 0;

    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if(cloudCurvature[i].value<0)
            continue;
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value > 0.1){
                //ROS_WARN("extracted feature not qualified, please check lidar");
                break;
            }
            smallestPickedNum++;
            picked_points.push_back(ind);
            
            if(smallestPickedNum <= cloudCurvature.size()/13){
                //find all points
                pc_out_surf->push_back(pc_in->points[ind]);
                point_info_count++;
            }
            else{
                break;
            }

        }
    }
    
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_out_rest->push_back(pc_in->points[ind]);
        }
    }
    


}
LaserProcessingClass::LaserProcessingClass(){
    
}


Double3d::Double3d(int id_in, double angle_in, double value_in){
    id = id_in;
    value =value_in;
    angle =angle_in;
}


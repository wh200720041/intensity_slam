// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

#define INTENSITY_THRESHOLD 0.50
void OdomEstimationClass::init(lidar::Lidar lidar_param){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudDisMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterSurf.setLeafSize(0.8, 0.8, 0.8);
    downSizeFilterDis.setLeafSize(0.4, 0.4, 0.4);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeDisMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    max_lidar_distance = lidar_param.max_distance * 0.85;
    optimization_count=12;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    *laserCloudDisMap += *edge_in;
    *laserCloudDisMap += *surf_in;
    optimization_count=12;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){

    if(optimization_count>4)
        optimization_count--;
    Eigen::Isometry3d odom_prediction = odom * last_odom.inverse() * odom;

    last_odom = odom;
    odom = odom_prediction;
    normalizeIsometry(odom);
    normalizeIsometry(last_odom);

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledDisCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud, downsampledDisCloud);

    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
        kdtreeDisMap->setInputCloud(laserCloudDisMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            addIntensityCostFactor(downsampledDisCloud,laserCloudDisMap,problem,loss_function);
            addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);

            ceres::Solver::Options options; 
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

        }
    }else{
        printf("not enough points in map to associate, map error");
    }
    q_w_curr.normalize();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);

}
void OdomEstimationClass::normalizeIsometry(Eigen::Isometry3d& trans){
    Eigen::Quaterniond q_temp(trans.linear());
    q_temp.normalize();
    trans.linear() = q_temp.toRotationMatrix();
}
void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;
                double weight = std::sqrt(pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z);
                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b,1.0);  
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {
            
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1.0 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                double weight = std::sqrt(pc_in->points[i].x*pc_in->points[i].x+pc_in->points[i].y*pc_in->points[i].y+pc_in->points[i].z*pc_in->points[i].z);
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm, 1.0);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        printf("not enough correct points");
    }

}

void OdomEstimationClass::addIntensityCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int dis_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {

        // if(pc_in->points[i].intensity<0.1)
        //     continue;
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        //kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
        if(kdtreeDisMap->radiusSearch(point_temp, 1.5, pointSearchInd, pointSearchSqDis)<3)//7
            continue;
        else if(pointSearchSqDis[0]<1.0){
            
            float intensity_derivative[8]={0,0,0,0,0,0,0,0};
            calculateDerivative(map_in->points[i],pointSearchInd,map_in,intensity_derivative);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            Eigen::Vector3d nearest_point(map_in->points[pointSearchInd[0]].x,map_in->points[pointSearchInd[0]].y,map_in->points[pointSearchInd[0]].z);
            ceres::CostFunction *cost_function = new IntensityAnalyticCostFunction(curr_point, nearest_point, pc_in->points[i].intensity - map_in->points[pointSearchInd[0]].intensity, intensity_derivative, 1.0);  
            problem.AddResidualBlock(cost_function, loss_function, parameters);
            dis_num++;
                     
        }
    }
    if(dis_num<10){
        printf("not enough correct points");
    }

}
void OdomEstimationClass::calculateDerivative(pcl::PointXYZI& point_in, std::vector<int>& pointSearchInd, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, float intensity_derivative[]){

    //calculate average 
    //calculate derivative
    int intensity_count[8]={0,0,0,0,0,0,0,0};
    for(int i=1;i<(int)pointSearchInd.size();i++){
        int position_id = 0;
        if(map_in->points[pointSearchInd[i]].x - map_in->points[pointSearchInd[0]].x >= 0){
            position_id+=0;
        }else{
            position_id+=4;
        }
        
        if(map_in->points[pointSearchInd[i]].y - map_in->points[pointSearchInd[0]].y >= 0){
            position_id+=0;
        }else{
            position_id+=2;
        }

        if(map_in->points[pointSearchInd[i]].z - map_in->points[pointSearchInd[0]].z >= 0){
            position_id+=0;
        }else{
            position_id+=1;
        }

        intensity_count[position_id] += 1;
        intensity_derivative[position_id]+= map_in->points[pointSearchInd[i]].intensity;
    }

    //assign to derivative
    for(int i=0;i<8;i++){
        if(intensity_count[i]!=0)
            intensity_derivative[i] = intensity_derivative[i]/intensity_count[i] - map_in->points[pointSearchInd[0]].intensity;
        else
            intensity_derivative[i] = -1;// - map_in->points[pointSearchInd[0]].intensity; 
    }
    
}
void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out, pcl::PointCloud<pcl::PointXYZI>::Ptr& dis_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpDis(new pcl::PointCloud<pcl::PointXYZI>());

    for(int i=0;i<edge_pc_in->points.size();i++){
        if(edge_pc_in->points[i].intensity>INTENSITY_THRESHOLD)
           tmpDis->push_back(edge_pc_in->points[i]);
    }
    for(int i=0;i<surf_pc_in->points.size();i++){
        if(surf_pc_in->points[i].intensity>INTENSITY_THRESHOLD)
           tmpDis->push_back(surf_pc_in->points[i]);
    }
    downSizeFilterDis.setInputCloud(tmpDis);
    downSizeFilterDis.filter(*dis_pc_out);    
    
}
void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){
    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
        if(point_temp.intensity>INTENSITY_THRESHOLD)
            laserCloudDisMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
        if(point_temp.intensity>INTENSITY_THRESHOLD)
            laserCloudDisMap->push_back(point_temp); 
    }
    
    double x_min = +odom.translation().x()-100;
    double y_min = +odom.translation().y()-100;
    double z_min = +odom.translation().z()-100;
    double x_max = +odom.translation().x()+100;
    double y_max = +odom.translation().y()+100;
    double z_max = +odom.translation().z()+100;
    
    //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpDis(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);
    cropBoxFilter.setInputCloud(laserCloudDisMap);
    cropBoxFilter.filter(*tmpDis);

    downSizeFilterEdge.setInputCloud(tmpSurf);
    downSizeFilterEdge.filter(*laserCloudSurfMap);
    downSizeFilterSurf.setInputCloud(tmpCorner);
    downSizeFilterSurf.filter(*laserCloudCornerMap);
    downSizeFilterDis.setInputCloud(tmpDis);
    downSizeFilterDis.filter(*laserCloudDisMap);


}

pcl::PointCloud<pcl::PointXYZI>::Ptr OdomEstimationClass::getMap(void){
    
    return laserCloudDisMap;
}

OdomEstimationClass::OdomEstimationClass(){

}
// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_OPTIMIZATION_ANALYTIC_H_
#define _LIDAR_OPTIMIZATION_ANALYTIC_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<3, 7> {
	public:

		EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double weight_);
		virtual ~EdgeAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;
		double weight;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_, double weight_);
		virtual ~SurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
		double weight;
};


class IntensityAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		IntensityAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d nearest_point_, float intensity_residual_, float intensity_derivative_[],  float weight);
		virtual ~IntensityAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
	
		Eigen::Vector3d curr_point;
		Eigen::Vector3d nearest_point;
		double intensity_residual;
		double intensity_derivative[8];
		double weight;
};

struct IntensityAutoDiffCostFunction
{

	IntensityAutoDiffCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d nearest_point_, float target_intensity_, float intensity_derivative_[]) 
                                                        : curr_point(curr_point_),nearest_point(nearest_point_),target_intensity(target_intensity_){
	    for(int i=0;i<8;i++){
	        intensity_derivative[i] = intensity_derivative_[i];
	    }
	}
	virtual ~IntensityAutoDiffCostFunction() {}
	
	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const{
	    Eigen::Quaternion<T> q_w_curr(q[3], q[0], q[1], q[2]);
	    Eigen::Matrix<T, 3, 1> t_w_curr(t[0], t[1], t[2]);
	    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
	    Eigen::Matrix<T, 3, 1> point_w = q_w_curr * cp + t_w_curr;
	    //f(xi) = I(p)- I(p')
	    //I(p) = xyz_radius * delta_intensity
	    T delta_x = point_w.x()-(T)nearest_point.x();
	    T delta_y = point_w.y()-(T)nearest_point.y();
	    T delta_z = point_w.z()-(T)nearest_point.z();
	    T xy_radius = ceres::sqrt(delta_x*delta_x+delta_y*delta_y);
	    T xyz_radius = ceres::sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
	    
	    T theta_1 = ceres::atan2(delta_z,xy_radius);
	    T theta_2 = ceres::atan2(delta_y,delta_x);
	    if(delta_x==0.0||delta_y==0.0||delta_z==0.0){
	    	xy_radius= (T)0.0;
	    	xyz_radius = (T)0.0;
	    	theta_1=(T)0.0;
	    	theta_2=(T)0.0;
	    }
	    T q1,q2,q3,q4,angle_shifts;
	    if(theta_2>=-0.75*M_PI &&theta_2<-0.25*M_PI){
	        q1 = (T)(intensity_derivative[2]+intensity_derivative[6])/2.0;
	        q2 = (T)(intensity_derivative[2]-intensity_derivative[6])/2.0;
	        q3 = (T)(intensity_derivative[3]+intensity_derivative[7])/2.0;
	        q4 = (T)(intensity_derivative[3]-intensity_derivative[7])/2.0;
	        angle_shifts = (T)0.5*M_PI;
	    }else if(theta_2>=-0.25*M_PI &&theta_2<0.25*M_PI){
	        q1 = (T)(intensity_derivative[0]+intensity_derivative[2])/2.0;
	        q2 = (T)(intensity_derivative[0]-intensity_derivative[2])/2.0;
	        q3 = (T)(intensity_derivative[1]+intensity_derivative[3])/2.0;
	        q4 = (T)(intensity_derivative[1]-intensity_derivative[3])/2.0;
	        angle_shifts = (T)0.0;
	    }else if(theta_2>=0.25*M_PI &&theta_2<0.75*M_PI){
	        // p1 = (intensity_derivative[4]+intensity_derivative[0])/2 +  (intensity_derivative[4]-intensity_derivative[0])/2 *(sin(2*(theta_2-0.5*M_PI)));
	        // p2 = (intensity_derivative[5]+intensity_derivative[1])/2 +  (intensity_derivative[5]-intensity_derivative[1])/2 *(sin(2*(theta_2-0.5*M_PI)));
	        q1 = (T)(intensity_derivative[4]+intensity_derivative[0])/2.0;
	        q2 = (T)(intensity_derivative[4]-intensity_derivative[0])/2.0;
	        q3 = (T)(intensity_derivative[5]+intensity_derivative[1])/2.0;
	        q4 = (T)(intensity_derivative[5]-intensity_derivative[1])/2.0;
	        angle_shifts = (T)-0.5*M_PI;
	    }else{
	        // p1 = (intensity_derivative[6]+intensity_derivative[4])/2 +  (intensity_derivative[6]-intensity_derivative[4])/2 *(sin(2*(theta_2)));
	        // p2 = (intensity_derivative[7]+intensity_derivative[5])/2 +  (intensity_derivative[7]-intensity_derivative[5])/2 *(sin(2*(theta_2)));
	        q1 = (T)(intensity_derivative[6]+intensity_derivative[4])/2.0;
	        q2 = (T)(intensity_derivative[6]-intensity_derivative[4])/2.0;
	        q3 = (T)(intensity_derivative[7]+intensity_derivative[5])/2.0;
	        q4 = (T)(intensity_derivative[7]-intensity_derivative[5])/2.0;
	        angle_shifts = (T)0.0;
	    }

	    T p1 = q1 + q2 *(ceres::sin(2.0*(theta_2 + angle_shifts)));
	    T p2 = q3 + q4 *(ceres::sin(2.0*(theta_2 + angle_shifts)));
	    T delta_intensity = (p1+p2)/2.0 + (p1-p2)/2.0 * ceres::sin(2.0*theta_1);
	    residual[0] = xyz_radius * delta_intensity  + xyz_radius;

	    return true;
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d nearest_point;
	double target_intensity;
	double intensity_derivative[8];
};


class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};



#endif // _LIDAR_OPTIMIZATION_ANALYTIC_H_


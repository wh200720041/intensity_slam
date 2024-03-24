// Author of INTENSITY-SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimization.h"

EdgeAnalyticCostFunction::EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double weight_)
        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), weight(weight_){

}

bool EdgeAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * curr_point + t_last_curr; //new point
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;

    residuals[0] = weight * nu.x() / de.norm();
    residuals[1] = weight * nu.y() / de.norm();
    residuals[2] = weight * nu.z() / de.norm();

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_lp;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            Eigen::Vector3d re = last_point_b - last_point_a;
            Eigen::Matrix3d skew_re = skew(re);

            J_se3.block<3,6>(0,0) = weight * skew_re * dp_by_so3/de.norm();
      
        }
    }

    return true;
 
}   

//surf norm cost

SurfNormAnalyticCostFunction::SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_, double weight_) 
                                                        : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_), weight(weight_){

}

bool SurfNormAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;

    residuals[0] = weight * (plane_unit_norm.dot(point_w) + negative_OA_dot_norm);

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);

            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = weight * plane_unit_norm.transpose() * dp_by_so3;
   
        }
    }
    return true;

}   

//intensity cost
IntensityAnalyticCostFunction::IntensityAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d nearest_point_, float intensity_residual_, float intensity_derivative_[], float weight_) 
                                                        : curr_point(curr_point_),nearest_point(nearest_point_),intensity_residual(intensity_residual_),weight(weight_){
    for(int i=0;i<8;i++){
        intensity_derivative[i] = intensity_derivative_[i];
    }
}

bool IntensityAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
    Eigen::Vector3d point_w = q_w_curr * curr_point + t_w_curr;
    //f(xi) = I(p)- I(p')
    //I(p) = xyz_radius * delta_intensity

    double delta_x = point_w.x()-nearest_point.x();
    double delta_y = point_w.y()-nearest_point.y();
    double delta_z = point_w.z()-nearest_point.z();
    double xy_radius_square = std::max(delta_x*delta_x+delta_y*delta_y,0.0000004);
    double xy_radius = std::sqrt(xy_radius_square);
    double xyz_radius_square = std::max(xy_radius_square + delta_z*delta_z,0.0000004);
    double xyz_radius = std::sqrt(xyz_radius_square);
    double theta_1 = std::atan2(delta_z,xy_radius);
    double theta_2 = std::atan2(delta_y,delta_x);

    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    double q4 = 0.0;
    double angle_shifts = 0.0;
    if(theta_2>=-0.75*M_PI &&theta_2<-0.25*M_PI){
        q1 = (intensity_derivative[2]+intensity_derivative[6])/2;
        q2 = (intensity_derivative[2]-intensity_derivative[6])/2;
        q3 = (intensity_derivative[3]+intensity_derivative[7])/2;
        q4 = (intensity_derivative[3]-intensity_derivative[7])/2;
        angle_shifts = 0.5*M_PI;
    }else if(theta_2>=-0.25*M_PI &&theta_2<0.25*M_PI){
        q1 = (intensity_derivative[0]+intensity_derivative[2])/2;
        q2 = (intensity_derivative[0]-intensity_derivative[2])/2;
        q3 = (intensity_derivative[1]+intensity_derivative[3])/2;
        q4 = (intensity_derivative[1]-intensity_derivative[3])/2;
        angle_shifts = 0.0;
    }else if(theta_2>=0.25*M_PI &&theta_2<0.75*M_PI){
        // p1 = (intensity_derivative[4]+intensity_derivative[0])/2 +  (intensity_derivative[4]-intensity_derivative[0])/2 *(sin(2*(theta_2-0.5*M_PI)));
        // p2 = (intensity_derivative[5]+intensity_derivative[1])/2 +  (intensity_derivative[5]-intensity_derivative[1])/2 *(sin(2*(theta_2-0.5*M_PI)));
        q1 = (intensity_derivative[4]+intensity_derivative[0])/2;
        q2 = (intensity_derivative[4]-intensity_derivative[0])/2;
        q3 = (intensity_derivative[5]+intensity_derivative[1])/2;
        q4 = (intensity_derivative[5]-intensity_derivative[1])/2;
        angle_shifts = -0.5*M_PI;
    }else{
        // p1 = (intensity_derivative[6]+intensity_derivative[4])/2 +  (intensity_derivative[6]-intensity_derivative[4])/2 *(sin(2*(theta_2)));
        // p2 = (intensity_derivative[7]+intensity_derivative[5])/2 +  (intensity_derivative[7]-intensity_derivative[5])/2 *(sin(2*(theta_2)));
        q1 = (intensity_derivative[6]+intensity_derivative[4])/2;
        q2 = (intensity_derivative[6]-intensity_derivative[4])/2;
        q3 = (intensity_derivative[7]+intensity_derivative[5])/2;
        q4 = (intensity_derivative[7]-intensity_derivative[5])/2;
        angle_shifts = 0.0;
    }

    double p1 = q1 + q2 *(sin(2*(theta_2 + angle_shifts)));
    double p2 = q3 + q4 *(sin(2*(theta_2 + angle_shifts)));
    double delta_intensity = (p1+p2)/2 + (p1-p2)/2 * sin(2*theta_1);
    residuals[0] = weight * (delta_intensity - intensity_residual);
    //residuals[0] = xyz_radius;
    //residuals[0] = xyz_radius * delta_intensity;
    Eigen::Matrix<double, 1, 2> dintensity_by_dtheta;
    dintensity_by_dtheta(0,0) = (p1 - p2) * cos(2*theta_1);
    dintensity_by_dtheta(0,1) = (q2 + q4 + (q2 - q4) * sin(2*theta_1)) * cos(2*(theta_2 + angle_shifts));

    Eigen::Matrix<double, 2, 3> dtheta_by_dp;
    dtheta_by_dp(0,0) = -delta_x*delta_z/(xy_radius*xyz_radius_square);
    dtheta_by_dp(0,1) = -delta_y*delta_z/(xy_radius*xyz_radius_square);
    dtheta_by_dp(0,2) = xy_radius/(xyz_radius_square);

    dtheta_by_dp(1,0) = -delta_y/xy_radius_square;
    dtheta_by_dp(1,1) = delta_x/xy_radius_square;
    dtheta_by_dp(1,2) = 0.0;

    Eigen::Matrix<double, 1, 3> dradius_by_dp;
    dradius_by_dp(0,0) = delta_x/xyz_radius;
    dradius_by_dp(0,1) = delta_y/xyz_radius;
    dradius_by_dp(0,2) = delta_z/xyz_radius;

    Eigen::Vector3d test_vector(delta_x/xyz_radius,delta_y/xyz_radius,delta_z/xyz_radius);
    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)
        {
            Eigen::Matrix3d skew_point_w = skew(point_w);

            Eigen::Matrix<double, 3, 6> dp_by_so3;
            dp_by_so3.block<3,3>(0,0) = -skew_point_w;
            (dp_by_so3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
            J_se3.setZero();
            //[f(x,y,z)g(x,y,z)]' = f'(x,y,z)g(x,y,z)+f(x,y,z)g'(x,y,z)
            // J_se3.block<1,6>(0,0) =  dradius_by_dp * dp_by_so3;
            J_se3.block<1,6>(0,0) =  weight * (/*dradius_by_dp * delta_intensity + */xyz_radius * dintensity_by_dtheta * dtheta_by_dp) * dp_by_so3;  //不加radius 效果好一点？？？？
        }
    }
    return true;

} 

#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)
bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;
    return true;
}

bool PoseSE3Parameterization::Minus(const double *x, const double *delta, double *x_minus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_minus(x_minus_delta);
    Eigen::Map<Eigen::Vector3d> trans_minus(x_minus_delta + 4);

    quater_minus = delta_q.inverse() * quater;
    trans_minus = delta_q.inverse() * trans - delta_t;
    return true;
}

bool PoseSE3Parameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();
    return true;
}

bool PoseSE3Parameterization::MinusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();
    return true;
}

#else

bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> trans(x + 4);

    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_t;
    getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
    Eigen::Map<const Eigen::Quaterniond> quater(x);
    Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

    quater_plus = delta_q * quater;
    trans_plus = delta_q * trans + delta_t;

    return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    (j.topRows(6)).setIdentity();
    (j.bottomRows(1)).setZero();

    return true;
}

#endif


void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t){
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if(theta<1e-10)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());


    Eigen::Matrix3d J;
    if (theta<1e-10)
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}

Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}

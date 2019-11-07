#include "pose_local_parameterization.h"

// delta是6维的, p + 旋转向量alpha
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
  // p = p + dp
  // q = q * dq 
  // xjw？？？奇怪一般是p = R * dp, q = q * dq啊
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3); 

    Eigen::Map<const Eigen::Vector3d> dp(delta);
    // d_alpha-> dq
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  //  the jacobian has already been calculated in Evaluate.
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

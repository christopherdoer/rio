#include <boost/math/distributions/chi_squared.hpp>

#include <ekf_yrio/ekf_yrio_filter.h>

using namespace rio;

bool EkfYRioFilter::updateYaw(const Real& yaw_m, const Real& sigma_yaw, const Real& outlier_rejection_thresh)
{
  Matrix H = Matrix::Zero(1, covariance_.cols());
  Vector r(1);
  Vector R_diag(1);

  H.block(0, error_idx_.sc_attitude + 2, 1, 1) = Matrix::Ones(1, 1);
  r[0]      = math_helper::wrapToCentered(radar_clone_state_.nav_sol.getEuler_n_b().yaw() - yaw_m, M_PI);
  R_diag[0] = sigma_yaw * sigma_yaw;
  ROS_DEBUG("yaw_filter: %0.2f, yaw_m: %0.2f", radar_clone_state_.nav_sol.getEuler_n_b().yaw(), yaw_m);
  ROS_DEBUG_STREAM("r=" << r[0] << " -- > " << angles::to_degrees(r[0]) << "deg");

  // outlier rejection
  if (outlier_rejection_thresh > 0.001)
  {
    const Real gamma = r.transpose() * (H * covariance_ * H.transpose() + Matrix(R_diag.asDiagonal())).inverse() * r;
    boost::math::chi_squared chiSquaredDist(1.0);
    const double gamma_thresh = boost::math::quantile(chiSquaredDist, 1 - outlier_rejection_thresh);

    ROS_DEBUG_STREAM("gamma: " << gamma << ", thresh: " << gamma_thresh);

    if (gamma < gamma_thresh)
    {
      kfUpdate(r, H, R_diag);
      ROS_DEBUG_STREAM(kStreamingPrefix << "Inlier RIO Yaw");
    }
    else
    {
      ROS_WARN_STREAM(kStreamingPrefix << "Outlier RIO Yaw");
      return false;
    }
  }
  else
  {
    kfUpdate(r, H, R_diag);
  }
  return true;
}

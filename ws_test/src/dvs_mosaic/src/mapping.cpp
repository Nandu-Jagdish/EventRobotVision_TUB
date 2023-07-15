#include <dvs_mosaic/mosaic.h>
#include <glog/logging.h>


namespace dvs_mosaic
{

/**
* \brief Process each event to refine the mosaic variables (mean and covariance)
*/
void Mosaic::processEventForMap(const dvs_msgs::Event& ev,
  const double t_ev, const double t_prev,
  const cv::Matx33d& Rot, const cv::Matx33d& Rot_prev)
{
  const double dt_ev = t_ev - t_prev;
  CHECK_GT(dt_ev,0) << "Non-positive dt_ev"; // Two events at same pixel with same timestamp

  // FILL in ... lots of gaps. This function is mostly empty

  // Get map point corresponding to current event
  // hint: call project_EquirectangularProjection
  VLOG(2) << "EVENT: start processing event";
  cv::Point3d rotated_bvec;
  cv::Point2f pm;
  const int idx = ev.y*sensor_width_ + ev.x;

  rotated_bvec = Rot * precomputed_bearing_vectors_.at(idx);
  project_EquirectangularProjection(rotated_bvec, pm);

  // Get map point corresponding to previous event at same pixel
  cv::Point3d rotated_bvec_prev;
  cv::Point2f pm_prev;
  rotated_bvec_prev = Rot_prev * precomputed_bearing_vectors_.at(idx);
  project_EquirectangularProjection(rotated_bvec_prev, pm_prev);

  // Get approx optical flow vector (vector v in the paper)
  cv::Point2f flow_vec;
  flow_vec = (pm - pm_prev)/dt_ev;

  // Extended Kalman Filter (EKF) for the intensity gradient map.
  // Get gradient and covariance at current map point pm
  cv::Matx21f gm;
  cv::Matx22f Pg;

// check if pm is not nan
  if (pm.x != pm.x || pm.y != pm.y)
  {
    VLOG(1) << "EVENT: pm is nan";
    // return;

}
  cv::Vec2f pGrad = grad_map_.at<cv::Vec2f>(pm);
  cv::Vec3f pGradCovar = grad_map_covar_.at<cv::Vec3f>(pm);

  gm = cv::Matx21f(pGrad[0], pGrad[1]);
  Pg = cv::Matx22f(pGradCovar[0], pGradCovar[1], pGradCovar[1], pGradCovar[2]); //

  // gm = grad_map_.at<cv::Vec2f>(pm);
  // Pg = grad_map_covar_.at<cv::Vec4f>(pm);
  // float C_intensity = gm.dot(flow_vec)*dt_ev; // predicted intensity change
  float C_intensity  = C_th_;
  
  // taking polarities into account
  if (ev.polarity)
  {
    C_intensity = C_intensity*1;
  }
  else
  {
    C_intensity = C_intensity*-1;
  }

  // define instantaneous event rate
  float z = 1.0/dt_ev;

  // measurement function
  float h = gm.dot(flow_vec)/C_intensity;


  // Compute innovation, measurement matrix and Kalman gain
  float nu_innovation;
  nu_innovation = z - h;
  cv::Matx12f H_measurement(flow_vec.x/C_intensity, flow_vec.y/C_intensity);
  float S_covar = cv::Mat(H_measurement*Pg*H_measurement.t()).at<float>(0,0) + var_R_;
  cv::Matx21f Kalman_gain;
  Kalman_gain = Pg*H_measurement.t()*(1/S_covar);


  // Update gradient (state) and covariance
  gm += Kalman_gain * nu_innovation;
  Pg -= Kalman_gain * S_covar * Kalman_gain.t();

  // Store updated values on corresponding pixel of grad_map_ and grad_map_covar_
  // convert gm to cv::Vec2f
  grad_map_.at<cv::Vec2f>(pm) = cv::Vec2f(gm(0), gm(1));
  // grad_map_.at<cv::Vec2f>(pm) = gm;
  // convert Pg to cv::Vec3f
  grad_map_covar_.at<cv::Vec3f>(pm) = cv::Vec3f(Pg(0,0), Pg(0,1), Pg(1,1));
  // grad_map_covar_.at<cv::Vec4f>(pm) = Pg;
  VLOG(2) << "EVENT: end processing event";

}



}

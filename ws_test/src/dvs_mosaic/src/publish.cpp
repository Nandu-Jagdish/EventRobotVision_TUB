#include <dvs_mosaic/mosaic.h>
#include <dvs_mosaic/image_util.h>
#include <geometry_msgs/PoseStamped.h>
#include <dvs_mosaic/reconstruction.h>
#include <glog/logging.h>


namespace dvs_mosaic
{

/**
* \brief Publish several variables related to the mapping (mosaicing) part
*/
void Mosaic::publishMap()
{
  // Publish the current map state
  VLOG(1) << "publishMap()";

  if ( time_map_pub_.getNumSubscribers() > 0 )
  {
    // Time map. Fill content in appropriate range [0,255] and publish
    // Happening at the camera's image plane
    cv_bridge::CvImage cv_image_time;
    cv_image_time.header.stamp = ros::Time::now();
    cv_image_time.encoding = "mono8";
    image_util::normalize(time_map_, cv_image_time.image, 15.);
    time_map_pub_.publish(cv_image_time.toImageMsg());
  }

  // Various mosaic-related topics
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  if ( mosaic_pub_.getNumSubscribers() > 0 )
  {
    // Brightness image. Fill content in appropriate range [0,255] and publish
    // Call Poisson solver and publish on mosaic_pub_
    // FILL IN ...
    // Hints: call image_util::normalize discarding 1% of pixels
    cv_bridge::CvImage cv_image_mosaic;
    cv_image_mosaic.header.stamp = ros::Time::now();
    cv_image_mosaic.encoding = "mono8";

    // Generate mosaic image
    // cv::Mat mosaic_image;
    // call poisson_solver::solve
    cv::Mat Recon;
    poisson::reconstructBrightnessFromGradientMap(grad_map_, &Recon);
    image_util::normalize(Recon, cv_image_mosaic.image, 1.0f);
    mosaic_pub_.publish(cv_image_mosaic.toImageMsg());



  }

  if ( mosaic_gradx_pub_.getNumSubscribers() > 0 ||
       mosaic_grady_pub_.getNumSubscribers() > 0 )
  {
    // Visualize gradient map (x and y components)
    // FILL IN ...
    // Hints: use cv::split to split a multi-channel array into its channels (images)
    //        call image_util::normalize
    // split grad_map_ into grad_map_x_ and grad_map_y_
    cv::Mat grad_map_x, grad_map_y;
    std::vector<cv::Mat> grad_map_xy;
    cv::split(grad_map_, grad_map_xy);
    
    // normalize grad_map_x_ and grad_map_y_
    image_util::normalize(grad_map_xy[0], grad_map_x, 1.0f);
    image_util::normalize(grad_map_xy[1], grad_map_y, 1.0f);

    // check for subscribers
    if (mosaic_gradx_pub_.getNumSubscribers() > 0)
    {
      // publish grad_map_x_
      cv_bridge::CvImage cv_image_gradx;
      cv_image_gradx.header.stamp = ros::Time::now();
      cv_image_gradx.encoding = "mono8";
      cv_image_gradx.image = grad_map_x;
      mosaic_gradx_pub_.publish(cv_image_gradx.toImageMsg());
    }

    if (mosaic_grady_pub_.getNumSubscribers() > 0)
    {
      // publish grad_map_y_
      cv_bridge::CvImage cv_image_grady;
      cv_image_grady.header.stamp = ros::Time::now();
      cv_image_grady.encoding = "mono8";
      cv_image_grady.image = grad_map_y;
      mosaic_grady_pub_.publish(cv_image_grady.toImageMsg());
    }


  }

  if ( mosaic_tracecov_pub_.getNumSubscribers() > 0 )
  {
    // Visualize confidence: trace of the covariance of the gradient map
    // FILL IN ...
    // Hints: use cv::split to split a multi-channel array into its channels (images)
    //        call image_util::normalize
// TODO Change this

    // split grad_map_covar
    cv::Mat grad_map_covar_channels[3];
    cv::split(grad_map_covar_, grad_map_covar_channels);
    // trace here is the sum of diagnal element hence sum of 1st and 3th emement ie 1st and 3rd channel //??
    cv::Mat trace_grad_map = grad_map_covar_channels[0] + grad_map_covar_channels[2];
    // normalize trace_grad_map
    image_util::normalize(trace_grad_map, trace_grad_map, 1.0f);
    // publish trace_grad_map
    cv_bridge::CvImage cv_image_tracecov;
    cv_image_tracecov.header.stamp = ros::Time::now();
    cv_image_tracecov.encoding = "mono8";
    cv_image_tracecov.image = trace_grad_map;
    mosaic_tracecov_pub_.publish(cv_image_tracecov.toImageMsg());

  }


  
}


/**
* \brief Publish pose once the tracker has estimated it
*/
void Mosaic::publishPose()
{
  if (pose_pub_.getNumSubscribers() <= 0)
    return;

  VLOG(1) << "publishPose()";
  geometry_msgs::PoseStamped pose_msg;
  // FILL IN ... when tracking part is implemented

  pose_pub_.publish(pose_msg);
}

}

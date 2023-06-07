#include "dvs_integrator/integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>

namespace dvs_integrator {

Integrator::Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of the method
  nh_private.param<double>("cut_off", alpha_cutoff_, 5.);

  // Get parameters of the Leaky integrator
  std::string integration_method;
  nh_private.param<std::string>("Integration_Method",integration_method, "INTEGRATION_METHOD_LEAKY");
  if (integration_method == "INTEGRATION_METHOD_LEAKY")
  {
    integration_method_ = INTEGRATION_METHOD_LEAKY;
  }
  else if (integration_method == "INTEGRATION_DIRECT")
  {
    integration_method_ = INTEGRATION_DIRECT;
  }
  else
  {
    ROS_ERROR("Integration method not recognized. Using Leaky integrator.");
    integration_method_ = INTEGRATION_METHOD_LEAKY;
  }

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 0, &Integrator::eventsCallback, this);//infiinte queue size
  // Set the queue size to infinity to avoid dropping messages.


  image_transport::ImageTransport it_(nh_);
  // FILL IN...
 time_map_pub_ = it_.advertise("time_map", 1); //Assuming timemap is image
 image_pub_ = it_.advertise("image_out", 1);

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Integrator::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_integrator::dvs_integratorConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // Contrast thresholds
  // Later we may use varying contrast sensitivities
  c_pos_ = 0.1;
  c_neg_ = 0.1;
}


Integrator::~Integrator()
{
  // Close the publishers
  // FILL IN...
  image_pub_.shutdown();
  time_map_pub_.shutdown();
}


/**
 * Process input event messages
 */
void Integrator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Need to update the state (time_map and brightness image) even if there are no subscribers on the output image
  //check monotonic time and count events

  //check if the new event is older than current event



  
  // init t_first to dummy value
  double t_first_ = 0.0;
  

  std::cout << "alpha_cutoff_ = " << alpha_cutoff_ << std::endl;
  if (!(state_time_map_.rows == msg->height && state_time_map_.cols == msg->width))
  {
    // Allocate memory for time map and for image out
    const double t_first_ = msg->events[0].ts.toSec();
    // Initialize the time map with the time of the first event
    
    state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));


    // and initialize the brightness image to zero.
    // state_time_map_ = cv::Mat::zeros(msg->height, msg->width, CV_64FC1) ;
    state_image_ = cv::Mat::zeros(msg->height, msg->width, CV_64FC1);
//    state_image_    = cv::Mat ...
  }

  // Process events in the message msg, one by one (in a loop)
  // FILL IN...
  
  for (int i = 0; i < msg->events.size(); i++)
  {
    // Get the event
    const dvs_msgs::Event & e = msg->events[i];

    // Get the event coordinates
    const int x = e.x;
    const int y = e.y;

    // Get the event time
    const double t = e.ts.toSec();

    // Get the event polarity
    const bool p = e.polarity;

    // Get the time of the last event at (x,y)
    const double t_last = state_time_map_.at<double>(y, x);

    // Compute the time interval from the last event at (x,y)
    const double dt = t - t_last;

    // ros info on dt
    // ROS_INFO("dt = %f\n current time =%f\n Previous Time =%f\n**", dt, t,t_last);
    // if time not monotonic
    if (dt < 0)
    {
      
      ROS_INFO("dt = %f\n current time =%f\n Previous Time =%f\n**", dt, t,t_last);
      //reset state_time_map_
      state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));

      return;
    }
    // ros info on current , previous time and dt


    
    

    // Update the time map
    state_time_map_.at<double>(y, x) = t;

    // Update the brightness image
    // Leaky or Direct integration
    if (integration_method_ == INTEGRATION_METHOD_LEAKY)
    {
      // Leaky integrator
      if (p)
      {
        state_image_.at<double>(y, x) = c_pos_ + exp(-dt * alpha_cutoff_)*state_image_.at<double>(y, x);
      }
      else
      {
        state_image_.at<double>(y, x) = -c_neg_ + exp(-dt * alpha_cutoff_)*state_image_.at<double>(y, x);
      }
    }
    else if (integration_method_ == INTEGRATION_DIRECT)
    {
      // Direct method
      // FILL IN...
      if (p)
      {
        state_image_.at<double>(y, x) += c_pos_;
      }
      else
      {
        state_image_.at<double>(y, x) -= c_neg_;
      }
    }
    else
    {
      ROS_ERROR("Unknown integration method");
    }



   
    
    // if (p)
    // {

    //   state_image_.at<double>(y, x) += c_pos_ + exp(-dt * alpha_cutoff_)*state_time_map_.at<double>(y, x);
    // }
    // else
    // {
    //   state_image_.at<double>(y, x) -= c_neg_ + exp(-dt * alpha_cutoff_);
    // }
  }
  



  // Exit if there are no subscribers
  if (image_pub_.getNumSubscribers() + time_map_pub_.getNumSubscribers() > 0)
  {
    publishState();
  }

  // if 
}


/**
 * Publish the time map and the brightness image at latest time (Output)
 */
void Integrator::publishState()
{
  // Publish the current state (time map and image)

  // Initialize, including header
  cv_bridge::CvImage cv_image, cv_image_time;
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = "mono8";
  cv_image_time.header.stamp = ros::Time::now();
  cv_image_time.encoding = "mono8";

  // Need to display all pixels at the same reference time (e.g., the time of
  // the last event). So do not forget to "decay" the brightness to the ref. time.
  // FILL IN...
  // cv_image_time.image = state_time_map_;
  // cv_image.image = state_image_;

  cv::Mat image_out;
  cv::Mat time_map_out;

  state_image_.copyTo(image_out);
  state_time_map_.copyTo(time_map_out);



  // Convert to appropriate range, [0,255]
  // Feel free to use minMaxLocRobust
  // FILL IN...
  double min, max;
  minMaxLocRobust(image_out, min, max, 2);
  image_out = (image_out - min) * 255.0 / (max - min);
  image_out.convertTo(cv_image.image, CV_8UC1);
  // cv_image_time.image = (cv_image_time.image - min) * 255.0 / (max - min);
  // cv_image_time.image.convertTo(cv_image_time.image, CV_8UC1);

  double min_time, max_time;
  minMaxLocRobust(time_map_out, min_time, max_time, 2);
  time_map_out = (time_map_out - min_time) * 255.0 / (max_time - min_time);
  time_map_out.convertTo(cv_image_time.image, CV_8UC1);


  


  // Publish the time map and the brightness image
  time_map_pub_.publish(cv_image_time.toImageMsg());
  image_pub_.publish(cv_image.toImageMsg());
}


/**
 * Interface with the parameters that can be changed online via dynamic reconfigure
 */
void Integrator::reconfigureCallback(dvs_integrator::dvs_integratorConfig &config, uint32_t level)
{
  alpha_cutoff_ = config.Cutoff_frequency;
  // integration_method_ = config.Integration_method;
}


/**
 * Compute the robust min and max pixel values of an image using percentiles,
 * e.g., 2th and 98th percentiles instead of min and max, respectively.
 *
 * Sort the pixel values of an image and discard a percentage of them,
 * from the top and the bottom, as potential outliers to compute
 * a robust version of the min and max values.
 */
void Integrator::minMaxLocRobust(const cv::Mat& image, double& rmin, double& rmax,
                                 const double& percentage_pixels_to_discard)
{
  cv::Mat image_as_row = image.reshape(0,1);
  cv::Mat image_as_row_sorted;
  cv::sort(image_as_row, image_as_row_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  image_as_row_sorted.convertTo(image_as_row_sorted, CV_64FC1);
  const int single_row_idx_min = (0.5*percentage_pixels_to_discard/100.)*image.total();
  const int single_row_idx_max = (1 - 0.5*percentage_pixels_to_discard/100.)*image.total();
  rmin = image_as_row_sorted.at<double>(single_row_idx_min);
  rmax = image_as_row_sorted.at<double>(single_row_idx_max);
}

} // namespace

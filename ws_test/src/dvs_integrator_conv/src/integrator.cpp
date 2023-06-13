#include "dvs_integrator_conv/integrator.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <cassert>

namespace dvs_integrator_conv {

Integrator::Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  // Get parameters of the method from the launch file
  nh_private.param<double>("cut_off", alpha_cutoff_, 5.);

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 0, &Integrator::eventsCallback_bak, this);
  // set queue_size to 0 to avoid discarding messages (for correctness).

  image_transport::ImageTransport it_(nh_);
  time_map_pub_ = it_.advertise("time_map", 1);
  image_pub_ = it_.advertise("image_out", 1);

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Integrator::reconfigureCallback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_integrator_conv::dvs_integrator_convConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // Contrast thresholds
  // Later we may use varying contrast sensitivities
  c_pos_ = 0.1;
  c_neg_ = 0.1;
  t_last_ = 0.;
}


Integrator::~Integrator()
{
  time_map_pub_.shutdown();
  image_pub_.shutdown();
}


/**
 * Process input event messages
 */
void Integrator::eventsCallback_bak(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // Need to update the state (time_map and brightness image) even if there are no subscribers on the output image
  const double t_first_ = msg->events[0].ts.toSec();
  const bool non_monotonic_time = (t_first_ < t_last_);
  if (!(state_time_map_.rows == msg->height && state_time_map_.cols == msg->width)
    || non_monotonic_time )
  {
    // Allocate memory for time map and for brightness image
    state_time_map_ = cv::Mat(msg->height, msg->width, CV_64FC1, cv::Scalar(t_first_));
    state_image_    = cv::Mat::zeros(msg->height, msg->width, CV_64FC1);
  }

  // Choose a kernel (outside the event loop)
  cv::Mat kernel;
  // sobel_x_.copyTo(kernel);
  // single_pix.copyTo(kernel);
  
  setKernel(kernel);
  // get kernel size
  int ksize = kernel.rows;
  // get half kernel size
  int hksize = (ksize-1)/2;

  // int ksize; // =... FILL IN...
  // int hksize; // =... FILL IN... half kernel size: 0 if 1x1 kernel, 1 if 3x3 kenel, 2 if 5x5 kernel, etc.


  // ksize = 3;
  // hksize = 1;
  double tk = 0;

  for (const dvs_msgs::Event& ev : msg->events)
  {
    tk = ev.ts.toSec();
    cv::Rect roi(ev.x-hksize, ev.y-hksize, ksize, ksize);
    
    // Update state
    if ( (hksize <= ev.x) && (ev.x <= msg->width-1-hksize)
      && (hksize <= ev.y) && (ev.y <= msg->height-1-hksize) )
    {
      // Border due to convolution mask

      // Current set of state pixels to be updated
      // cv::Rect ... FILL IN...
      // set replacement kernel
      // cv::Mat kernel_roi = kernel(roi);
      // Update state: time map and map of convolved accumulated polarities
      //replace state_image_ with 
      // kernel = [[0,0,0],[0,1,0],[0,0,0]];
     
      // kernel.copyTo(state_image_(roi));
      // //get t_last_ from state_time_map_ at current x,y
      // t_last_ = state_time_map_.at<double>(ev.y, ev.x);
      
      
     
      // iterate over all pixels in the roi
      // ROS_INFO("iterate over all pixels in the roi");
      for (int i = 0; i < ksize; i++)
      {
        for (int j = 0; j < ksize; j++)
        {
           /* 
          //get the polarity of the current pixel
          double p = state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize);

          // get the event polarity
          c_neg_ = -1*c_pos_;
          double p_ev = (ev.polarity) ? c_pos_ : c_neg_;
          //get the time of the last event
          double t_last = state_time_map_.at<double>(ev.y+i-hksize, ev.x+j-hksize);
          //get the time difference between the current event and the last event
          double dt = tk - t_last;
          //get the contrast sensitivity
          // double c = (p > 0) ? c_pos_ : c_neg_;
          //get the decay factor
          double alpha = exp(-dt*alpha_cutoff_);
          //update the polarity of the current pixel
          // ROS_INFO("apply the leaky integrator");
          state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize) = p_ev + alpha*state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize);
          // state_image_.at<double>(ev.y, ev.x) = c + alpha*state_image_.at<double>(ev.y, ev.x);
          // ROS_INFO("apply the leaky integrator at %d, %d", ev.y, ev.x);
          // state_image_.at<double>(ev.y, ev.x) += c ;
          
          // update the time map
          // state_time_map_.at<double>(ev.y, ev.x) = tk;

         */  
          // test

          const int x = ev.x;
          const int y = ev.y;
          const bool p = ev.polarity;
          const double t = ev.ts.toSec();
          double t_last = state_time_map_.at<double>(y, x);
          const double dt = t - t_last;
          state_time_map_.at<double>(y, x) = tk;
          if (p)
          {
            state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize) = c_pos_ + exp(-dt * alpha_cutoff_)*state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize);
          }
          else
          {
            state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize) = -c_neg_ + exp(-dt * alpha_cutoff_)*state_image_.at<double>(ev.y+i-hksize, ev.x+j-hksize);
          }








        }
      }

      // update time map
      state_time_map_(roi) = tk;
      
      // //decay state_image in the roi
      // cv::Mat state_image_roi = state_image_(roi);
      // cv::Mat state_time_map_roi = state_time_map_(roi);
      // cv::Mat state_image_roi_decay = state_image_roi * exp(-(tk - t_last_)/alpha_cutoff_);
      // cv::Mat state_time_map_roi_decay = state_time_map_roi * exp(-(tk - t_last_)/alpha_cutoff_);
      // state_image_roi_decay.copyTo(state_image_(roi));
      // state_time_map_roi_decay.copyTo(state_time_map_(roi));





 



      // Update time image using tk
      // ... FILL IN...

    }
    else
    {
      // Out of bounds. Just update the time map
      // out of bounds
      ROS_WARN_STREAM("Event outside of bounds: " << ev.x << ", " << ev.y);
      // state_time_map_(roi) = tk;
      // Update time map at y,x
      state_time_map_.at<double>(ev.y, ev.x) = tk;
      

     

    }
    
  }
  
  //t_last_ = ... FILL IN...

  // Publish if there are any subscribers
  if (image_pub_.getNumSubscribers() + time_map_pub_.getNumSubscribers() > 0)
  {
    // publishState();
    publishState_bak();
    // std::cout<<('x');
  }
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

  // Fill content in appropriate range [0,255]
  // Need to decay the image to the present time
  // cv::Mat image_out;
  // FILL IN...

  // Map from current range (or better, from robust min and max) to [0,255]
  normalize(state_time_map_, cv_image_time.image, 5.);
  normalize(state_image_, cv_image.image, 5.);

  // Publish the time map and the brightness image
  time_map_pub_.publish(cv_image_time.toImageMsg());
  image_pub_.publish(cv_image.toImageMsg());
}


void Integrator::setKernel(cv::Mat& ker)
{
  switch(convolution_mask_)
  {
    case 1:
      // sobel_x_.copyTo(ker);
      ker = cv::Mat::ones(1, 1, CV_64FC1);

      break;
    default:
      ker = cv::Mat::ones(1, 1, CV_64FC1);
      // sobel_x_.copyTo(ker);

      break;
    // FILL IN...
  }
}


/**
 * Interface with the parameters that can be changed online via dynamic reconfigure
 */
void Integrator::reconfigureCallback(dvs_integrator_conv::dvs_integrator_convConfig &config, uint32_t level)
{
  alpha_cutoff_ = config.Cutoff_frequency;
  convolution_mask_ = config.Convolution_mask;
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


/**
 * Normalize src to the range [0,255] using robust min and max values
 */
void Integrator::normalize(const cv::Mat& src, cv::Mat& dst, const double& percentage_pixels_to_discard)
{
  double rmin_val, rmax_val;
  minMaxLocRobust(src, rmin_val, rmax_val, percentage_pixels_to_discard);
  const double scale = ((rmax_val != rmin_val) ? 255. / (rmax_val - rmin_val) : 1.);
  cv::Mat state_image_normalized = scale * (src - rmin_val);
  state_image_normalized.convertTo(dst, CV_8UC1);
}



// test set
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
    // if (integration_method_ == INTEGRATION_METHOD_LEAKY)
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
    // else if (integration_method_ == INTEGRATION_DIRECT)
    // {
    //   // Direct method
    //   // FILL IN...
    //   if (p)
    //   {
    //     state_image_.at<double>(y, x) += c_pos_;
    //   }
    //   else
    //   {
    //     state_image_.at<double>(y, x) -= c_neg_;
    //   }
    // }
    // else
    // {
    //   ROS_ERROR("Unknown integration method");
    // }



   
    
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
    publishState_bak();
  }
}
  // if 
void Integrator::publishState_bak()
{
  // Publish the current state (time map and image)
  // ROS_INFO("Publishing state");
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





} // namespace

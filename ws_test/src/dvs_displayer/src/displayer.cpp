#include "dvs_displayer/displayer.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

namespace dvs_displayer {

Displayer::Displayer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{

  //get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  if (display_method_str == std::string("grayscale"))
  {
      display_method_ = GRAYSCALE;
  } else
  {
      display_method_ = RED_BLUE;
  }

  used_last_image_ = false;

  // Set up subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Displayer::eventsCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("event_image", 1);
  image_sub_ = it_.subscribe("image", 1, &Displayer::imageCallback, this);
}

void Displayer::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;

   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }

   // Convert to BGR image
   if (msg->encoding == "mono8")
   {
     cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);
   }

   if (!used_last_image_)
   {
     cv_bridge::CvImage cv_image;
     last_image_.copyTo(cv_image.image);
     cv_image.encoding = "bgr8";
     image_pub_.publish(cv_image.toImageMsg());
   }
   used_last_image_ = false;
 }



Displayer::~Displayer()
{
  image_pub_.shutdown();
}


void Displayer::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  std::cout << "Num events in msg = " << msg->events.size() << std::endl;

  // Loop over the events
  /*
  for (int i = 0; i < msg->events.size(); i++)
  {

    std::cout << "Event " << i << " x=" << msg->events[i].x
                               << " y=" << msg->events[i].y
                               << " t=" << msg->events[i].ts
                               << " pol=" << (msg->events[i].polarity ? "true": "false")
                               << std::endl;

    // Or using an alias:
    const dvs_msgs::Event& ev = msg->events[i]; // an alias
    std::cout << "Event " << i << " x =" << ev.x
                               << " y=" << ev.y
                               << " t=" << ev.ts
                               << " pol=" << (ev.polarity ? "true": "false")
                               << std::endl;
  }


  int iCount = 0;
  for(const dvs_msgs::Event& ev : msg->events)
  {
    std::cout << "Event " << iCount++ << " x =" << ev.x
                               << " y =" << ev.y
                               << " t=" << ev.ts
                               << " pol=" << (ev.polarity ? "true": "false")
                             << std::endl;
  }
  */
 

  // Exit if there are no subscribers
  if (image_pub_.getNumSubscribers() <= 0)
  {
    return;
  }

  // Create image if at least one subscriber
  // Create image if at least one subscriber
  cv_bridge::CvImage cv_image;
  if (msg->events.size() > 0)
  {
    cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
  }

  if (display_method_ == GRAYSCALE)
  {
    // Create image
    cv_image.encoding = "mono8";
    // cv_image.image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
    if (last_image_.rows == msg->height && last_image_.cols == msg->width)
     {
       // If DAVIS image is available, use it as canvas
       cv::cvtColor(last_image_, cv_image.image, CV_BGR2GRAY);
       used_last_image_ = true;
     }
     else
     {
       // Create image
       cv_image.image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
     }

    // Per-pixel event histograms
    cv::Mat on_events  = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    cv::Mat off_events = cv::Mat::zeros(msg->height, msg->width, CV_8U);
    for(const dvs_msgs::Event& ev : msg->events)
    {
      if (ev.polarity == true)
        on_events.at<uint8_t>(ev.y, ev.x)++;
      else
        off_events.at<uint8_t>(ev.y, ev.x)++;
    }

    // Scale image to use the full range [0,255]
    // (using the same scaling factor for ON and OFF events)
    double max_on, max_off, dummy;
    cv::minMaxLoc(on_events, &dummy, &max_on);
    cv::minMaxLoc(off_events, &dummy, &max_off);
    const double max_abs_val = std::max(max_on, max_off);
    const double scale = 127 / max_abs_val;
    cv_image.image += scale * on_events;
    cv_image.image -= scale * off_events;

  }
  else if (display_method_ == RED_BLUE)
  {
    // Create image
    cv_image.encoding = "bgr8";
    // cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3, cv::Scalar(255,255,255));
    if (last_image_.rows == msg->height && last_image_.cols == msg->width)
     {
       // If DAVIS image is available, use it as canvas
       cv::cvtColor(last_image_, cv_image.image, CV_BGR2GRAY);
       used_last_image_ = true;
     }
     else
     {
       // Create image
       cv_image.image = cv::Mat(msg->height, msg->width, CV_8U, cv::Scalar(128));
     }


    // Just red or blue over white background
    for(const dvs_msgs::Event& ev : msg->events)
    {
      cv_image.image.at<cv::Vec3b>(ev.y, ev.x) =
      (ev.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
    }

  }
  else
  {
    std::cout << "Non-implemented display method" << std::endl;
  }

  image_pub_.publish(cv_image.toImageMsg());
}


} // namespace

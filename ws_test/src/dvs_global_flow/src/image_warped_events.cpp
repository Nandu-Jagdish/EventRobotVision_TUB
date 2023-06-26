#include "dvs_global_flow/image_warped_events.h"
#include <opencv2/imgproc/imgproc.hpp>


void warpEvent(
  const cv::Point2d& vel,
  const dvs_msgs::Event& event,
  const double t_ref,
  cv::Point2d* warped_pt
)
{
  // Warp event according to flow model: displacement = velocity * time
  // FILL IN ...
  //disp = vel * time

  warped_pt->x = event.x - vel.x * (event.ts.toSec() - t_ref);
  warped_pt->y = event.y - vel.y * (event.ts.toSec() - t_ref);
}



void accumulateWarpedEventBilinear(
  const dvs_msgs::Event& event,
  const int img_width,
  const int img_height,
  const cv::Point2d& ev_warped_pt,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(event.polarity) - 1.f : 1.f;

  // Accumulate warped events, using bilinear voting (polarity or count)
  const int xx = ev_warped_pt.x,
            yy = ev_warped_pt.y;

  // calculate the fracational part
  double dx = ev_warped_pt.x - xx;
  double dy = ev_warped_pt.y - yy;


  // if warped point is within the image, accumulate polarity
  if (1 <= xx && xx < img_width-2 && 1 <= yy && yy < img_height-2)
  {
    // Accumulate warped events on the IWE
    // FILL IN ...  image_warped (4 pixels)
    image_warped->at<double>(yy,xx) += (1 - dx)*(1-dy)*polarity;   //TODO check this index order
    image_warped->at<double>(yy+1,xx) += dx*(1-dy)*polarity;
    image_warped->at<double>(yy,xx+1) += (1-dx)*dy*polarity;
    image_warped->at<double>(yy+1,xx+1) += dx*dy*polarity;

  }

  


  // // Compute integer pixel coordinates of warped event location
  // const int x = static_cast<int>(ev_warped_pt.x);
  // const int y = static_cast<int>(ev_warped_pt.y);

  // // Compute fractional part of pixel coordinates
  // const double dx = ev_warped_pt.x - x;
  // const double dy = ev_warped_pt.y - y;

  // // Compute four nearest pixel coordinates
  // const int x1 = std::max(0, std::min(img_width-1, x));
  // const int y1 = std::max(0, std::min(img_height-1, y));
  // const int x2 = std::max(0, std::min(img_width-1, x+1));
  // const int y2 = std::max(0, std::min(img_height-1, y+1));

  // // Compute pixel values at four nearest pixel coordinates
  // const double f11 = image_warped->at<double>(y1,x1);
  // const double f12 = image_warped->at<double>(y2,x1);
  // const double f21 = image_warped->at<double>(y1,x2);
  // const double f22 = image_warped->at<double>(y2,x2);

  // // Interpolate pixel value at warped event location using bilinear interpolation
  // const double f = (1-dx)*(1-dy)*f11 + dx*(1-dy)*f21 + (1-dx)*dy*f12 + dx*dy*f22;

  // Accumulate polarity or count of warped event at interpolated pixel value

}


void computeImageOfWarpedEvents(
  const cv::Point2d& vel,
  const std::vector<dvs_msgs::Event>& events_subset,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  // FILL IN ...
  // hint:  *image_warped = ...
  // init image_warped
  * image_warped = cv::Mat::zeros(width, height,CV_64FC1);

  // Loop through all events
  const double t_ref = events_subset.front().ts.toSec(); // warp wrt 1st event
  cv::Point2d ev_warped_pt;
  for (const dvs_msgs::Event& ev : events_subset)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    // FILL IN ...
    warpEvent(vel, ev, t_ref, &ev_warped_pt);
    accumulateWarpedEventBilinear(ev, width, height, ev_warped_pt, image_warped, optsWarp);
    // hint: Call warpEvent() and accumulateWarpedEvent()
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    // FILL IN ...
    // hint: cv::GaussianBlur()
    cv::GaussianBlur(*image_warped, *image_warped, cv::Size(0,0), optsWarp.blur_sigma_); //TODO change the blur size
  }
}

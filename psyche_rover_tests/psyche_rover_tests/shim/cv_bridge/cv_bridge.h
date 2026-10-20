#pragma once
#include <memory>
#include <stdexcept>
#include <string>
#include "opencv2/opencv.hpp"
#include "shim_msgs.hpp"
namespace cv_bridge {
class Exception : public std::runtime_error { public: using std::runtime_error::runtime_error; };
struct CvImage { cv::Mat image; };
using CvImageConstPtr = std::shared_ptr<const CvImage>;
inline size_t elem_size(const std::string & enc) {
  if (enc == "16UC1" || enc == "mono16") return 2;
  if (enc == "32FC1") return 4;
  if (enc == "8UC1" || enc == "mono8") return 1;
  if (enc == "bgr8" || enc == "rgb8") return 3;
  throw Exception("unsupported encoding '" + enc + "'");
}
template<class ImgPtr>
CvImageConstPtr toCvShare(const ImgPtr & msg, const std::string & enc) {
  auto out = std::make_shared<CvImage>();
  out->image.rows = static_cast<int>(msg->height);
  out->image.cols = static_cast<int>(msg->width);
  out->image.elem = elem_size(enc);
  out->image.data = msg->data.data();
  out->image.total_bytes = msg->data.size();
  if (msg->data.size() < static_cast<size_t>(msg->height) * msg->width * out->image.elem)
    throw Exception("image buffer smaller than height*width*elem");
  return out;
}
}  // namespace cv_bridge

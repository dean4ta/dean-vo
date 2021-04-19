#pragma once

#include <opencv2/core/core.hpp>

// data associated with one image
struct detector_data
{
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

struct image_frame
{
  double timestamp;
  std::vector<cv::Mat> images;
};

struct vo_config
{
  std::string vo_cam_type;
  bool using_imu;
  bool using_lidar;
  std::vector<cv::Mat> intrinsics;
  std::vector<cv::Mat> inverse_intrinsics;
  std::vector<cv::Mat> extrinsics;  // relative to camera at index 0
  std::string image_feed_type;
};
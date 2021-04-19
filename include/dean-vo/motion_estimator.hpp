#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

// Abstract Class
class MotionEstimator
{
public:
  MotionEstimator(std::vector<cv::Mat> intrinsics);

  virtual cv::Mat findTransform(std::vector<cv::Point2f> points_prev,
                                std::vector<cv::Point2f> points,
                                cv::Mat& R,
                                cv::Mat& t) = 0;
};

class FundMatMotionEstimator
{
};
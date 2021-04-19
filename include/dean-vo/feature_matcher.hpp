#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "dean-vo/feature_detector.hpp"

// Abstract Class
class FeatureMatcher
{
public:
  FeatureMatcher()
  {
  }

  virtual void runMatcher(const std::vector<cv::Mat> descriptors,
                          std::vector<cv::DMatch>& matches) = 0;

  virtual void filterMatches(const std::vector<cv::DMatch>& matches,
                             std::vector<cv::KeyPoint>& keypoints_prev,
                             std::vector<cv::KeyPoint>& keypoints,
                             std::vector<cv::Point2f>& points_prev,
                             std::vector<cv::Point2f>& points)
  {
  }
};

class BruteForceMatcher : public FeatureMatcher
{
public:
  BruteForceMatcher(double distance_thresh) : FeatureMatcher(), distance_thresh_(distance_thresh)
  {
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
  }

  void runMatcher(const std::vector<cv::Mat> descriptors, std::vector<cv::DMatch>& matches)
  {
    if (descriptors.size() != 2)
    {
      std::cout << "[ERROR]: 2 descriptor sets are required in feature matching\n";
      return;
    }
    matcher_->match(descriptors[0], descriptors[1], matches);
  }

  void filterMatches(const std::vector<cv::DMatch>& matches,
                     std::vector<cv::KeyPoint>& keypoints_prev,
                     std::vector<cv::KeyPoint>& keypoints,
                     std::vector<cv::Point2f>& points_prev,
                     std::vector<cv::Point2f>& points)
  {
    for (int i = 0; i < matches.size(); ++i)
    {
      points_prev.reserve(matches.size());
      points.reserve(matches.size());
      if (matches[i].distance <= distance_thresh_)
      {
        points_prev.push_back(keypoints_prev[matches[i].queryIdx].pt);
        points.push_back(keypoints[matches[i].queryIdx].pt);
      }
    }
  }

private:
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  const double distance_thresh_;
};
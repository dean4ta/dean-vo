#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "dean-vo/types.hpp"

// Abstract Class
class FeatureDetector
{
public:
  FeatureDetector()
  {
  }

  virtual void runDetector(const cv::Mat image, detector_data& detected_features) = 0;
};

class OrbFeatureDetector : public FeatureDetector
{
public:
  OrbFeatureDetector() : FeatureDetector()
  {
    detector_ = cv::ORB::create();
    descriptor_ = cv::ORB::create();
  }

  void runDetector(const cv::Mat image, detector_data& detected_features)
  {
    // Detect features
    detector_->detect(image, detected_features.keypoints);

    // Compute descriptors
    descriptor_->compute(image, detected_features.keypoints, detected_features.descriptors);
  }

private:
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_;
};
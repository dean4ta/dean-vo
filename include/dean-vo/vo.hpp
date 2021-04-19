#pragma once

#include <string>

#include <opencv2/core/core.hpp>

#include "dean-vo/image_server.hpp"
#include "dean-vo/feature_detector.hpp"
#include "dean-vo/feature_matcher.hpp"
#include "dean-vo/motion_estimator.hpp"

#include "dean-vo/YAMLHelper.hpp"
#include "dean-vo/vo_time.hpp"
#include "dean-vo/types.hpp"

class VO
{
public:
  /**
   * @brief Construct a Visual Odometry object that builds out the 1) Image Server 2) Feature
   * Detector 3) Feature Matcher 4) Motion Estimator 5) Optimizer
   *
   * @param path_to_config_dir  Path to the file containing VO configurations
   */
  VO(std::string path_to_config_file);

  /**
   * @brief Start the VO pipeline
   */
  void start();

  bool safeGetNextImage(image_frame& frame);

private:
  vo_config config_;

  // using polymorphism to use the inherited classes. Example link here:
  // https://www.tutorialspoint.com/cplusplus/cpp_polymorphism.htm
  std::unique_ptr<ImageServer> image_server_;
  std::unique_ptr<FeatureDetector> feature_detector_;
  std::unique_ptr<FeatureMatcher> feature_matcher_;
  std::unique_ptr<MotionEstimator> motion_estimator_;
  // VO_Time vo_time_;
};

VO::VO(std::string path_to_config_file)
{
  std::cout << "starting the dean-vo class\n";
  if (path_to_config_file == "test")
  {
    // initialize the config_
    // clang-format off
    float f = 9.842439e+02;  // assuming same camera intrinsics with both cameras
    float tmp_intrinsics[9] = { f, 0, 6.95e+2, 
                                0, f, 2.40e+2, 
                                0, 0, 1 };
    config_.intrinsics.push_back(cv::Mat(3, 3, CV_32F, tmp_intrinsics));
    config_.intrinsics.push_back(cv::Mat(3, 3, CV_32F, tmp_intrinsics));
    float baseline = 5.370000e-01;

    float tmp_inverse_intrinsics[6] = { 1 / f, 0,     -config_.intrinsics[0].at<float>(0, 2) / f,
                                        0,     1 / f, -config_.intrinsics[0].at<float>(1, 2) / f };
    cv::Mat inverse_intrinsics = cv::Mat(2, 3, CV_32F, tmp_inverse_intrinsics);
    config_.inverse_intrinsics.push_back(inverse_intrinsics);
    config_.inverse_intrinsics.push_back(inverse_intrinsics);

    float extrinsics0[12] = { 1, 0, 0, 0, 
                              0, 1, 0, 0, 
                              0, 0, 1, 0 };
    config_.extrinsics.push_back(cv::Mat(3, 4, CV_32F, extrinsics0));
    float extrinsics1[12] = { 1, 0, 0, 0, 
                              0, 1, 0, 0, 
                              0, 0, 1, -baseline };
    config_.extrinsics.push_back(cv::Mat(3, 4, CV_32F, extrinsics1));
    // clang-format on

    std::string kitti_data_src = "../data/2011_09_26_drive_0001_sync/";
    int number_of_images = 108;

    /* Build the VO Pipeline Components */
    image_server_.reset(new KittiImageServer(kitti_data_src, 108));
    feature_detector_.reset(new OrbFeatureDetector());
    feature_matcher_.reset(new BruteForceMatcher(15));
  }
  else
  {
    std::cout << "[ERROR]: not implemented\n";
  }
}

void VO::start()
{
  std::cout << "starting VO pipeline\n";
  bool first_frame = true;
  image_frame frame;
  image_frame frame_prev;
  // std::unique_ptr<image_frame> frame_p;
  // std::unique_ptr<image_frame> frame_prev_p;
  // image_frame* frame;
  // image_frame* frame_prev;
  detector_data detected_features;
  detector_data detected_features_prev;
  std::vector<cv::DMatch> temporal_matches;

  // get first frame
  // std::cout << "getting first frame\n";
  safeGetNextImage(frame_prev);
  feature_detector_->runDetector(frame_prev.images[0], detected_features_prev);
  for (int i = 0;; ++i)
  {
    /* 1. Get next frame */
    std::cout << "\tgetting " << i << " frame\n";
    if (!safeGetNextImage(frame))
      break;
    cv::imshow("img", frame.images[0]);
    cv::waitKey(0);

    /* 2. Detector */
    std::cout << "running detector\n";
    feature_detector_->runDetector(frame.images[0], detected_features);

    /* 3. Matcher */
    std::cout << "running matcher\n";
    std::vector<cv::Mat> descriptors(2);
    descriptors[0] = detected_features.descriptors;
    descriptors[1] = detected_features_prev.descriptors;
    feature_matcher_->runMatcher(descriptors, temporal_matches);

    // 3.1 extract points from matches
    std::cout << "extracting points from matcher\n";
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2f> points_prev;
    feature_matcher_->filterMatches(temporal_matches,
                                    detected_features_prev.keypoints,
                                    detected_features.keypoints,
                                    points_prev,
                                    points);
    std::cout << "Number of matches: " << points_prev.size() << "\n";

    // 4. Motion Estimation
    cv::Mat outlier_mask, R, t;
    // cv::Mat f_matrix = cv::findFundamentalMat(points_prev, points, outlier_mask);
    float f = 9.842439e+02;  // assuming same camera intrinsics with both cameras

    // clang-format off
    float tmp_intrinsics[9] = { f, 0, 6.95e+2, 
                                0, f, 2.40e+2, 
                                0, 0, 1 };

    // clang-format on
    cv::Mat intrinsics = cv::Mat(3, 3, CV_32F, tmp_intrinsics);
    cv::Mat E =
        cv::findEssentialMat(points_prev, points, intrinsics, cv::RANSAC, 0.99, 1.0, outlier_mask);
    // std::cout << "\t" << outlier_mask << "\n";
    cv::recoverPose(E, points_prev, points, intrinsics, R, t, outlier_mask);
    // std::cout << "\t" << outlier_mask << "\n";
    std::cout << "Rotation: " << R << "\nTranslation: " << t << "\n";

    /* prepare for the next iteration */
    frame_prev = std::move(frame);
    detected_features_prev = std::move(detected_features);
  }
  std::cout << "VO finished\n";
  return;
}

bool VO::safeGetNextImage(image_frame& frame)
{
  // std::cout << "do i even make it here?\n";
  try
  {
    // std::cout << "getting frame\n";
    image_server_->getNextImage(frame);
    if (frame.timestamp == -1)
      return false;
  }
  catch (const std::exception& e)
  {
    std::cout << "image read error\n";
    std::cerr << e.what() << '\n';
    return false;
  }
  return true;
}
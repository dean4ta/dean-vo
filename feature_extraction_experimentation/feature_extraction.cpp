#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "usage: feature_extraction img1 img2" << std::endl;
    return 1;
  }
  cv::Mat img_1 = cv::imread(argv[1]);
  cv::Mat img_2 = cv::imread(argv[2]);

  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat descriptors_1, descriptors_2;

  // Initialize the feature detection
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

  // Initialize the descriptor extraction
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

  // Initialize descriptor matching
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Detect features in both images
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  // Compute descriptors in both images
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  // Show the image with detected features
  cv::Mat outimg1;
  cv::drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  cv::imshow("Detected ORB features", outimg1);

  // Match the descriptors in the two images. Hamming distance is the default
  std::vector<cv::DMatch> matches;
  matcher->match(descriptors_1, descriptors_2, matches);

  // Show the matches
  cv::Mat img_match;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  imshow("All matches", img_match);

  // Filter the matches based on a distance threshold
  const double distance_thresh = 30;
  std::vector<cv::DMatch> good_matches;
  std::vector<float> disparities;
  std::vector<cv::Point2f> points_1;
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (matches[i].distance <= distance_thresh)
    {
      good_matches.push_back(matches[i]);
      points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
      // cameras' x axes are co-linear
      disparities.push_back(keypoints_1[matches[i].queryIdx].pt.x - keypoints_2[matches[i].trainIdx].pt.x);
    }
  }

  // Show the filtered match
  cv::Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  cv::imwrite("img_goodmatch.png", img_goodmatch);
  imshow("Good matches", img_goodmatch);

  // calculate X,Y,Z of point relative to camera 1 given disparity, camera intrinsics, and baseline of camera.
  //  Estimated camera intrinsics:
  float height = img_1.rows;
  float width = img_1.cols;
  float f = height;
  float tmp_intrinsics[9] = {
    f, 0, width/2,
    0, f, height/2,
    0, 0, 1
  };
  cv::Mat intrinsics = cv::Mat(3, 3, CV_32F, tmp_intrinsics);
  float baseline = 0.15;

  float tmp_inverse_intrinsics[6] = {
    1/f, 0, -width/(2*f),
    0, 1/f, -height/(2*f)
  };
  cv::Mat inverse_intrinsics = cv::Mat(2, 3, CV_32F, tmp_inverse_intrinsics);

  std::vector<cv::Point3f>  cam_T_point;
  for (int i = 0; i < good_matches.size(); i++)
  {
    float Z = intrinsics.at<float>(0,0)*baseline/disparities[i];
    cv::Mat XY = cv::Mat(2, 1, CV_32F);
    float tmp_point[3] = {points_1[i].x, points_1[i].y, 1};
    cv::Mat image_point = cv::Mat(3,1, CV_32F, tmp_point);
    XY = Z * (inverse_intrinsics * image_point);
    cv::Point3f point_3d;
    point_3d.x = XY.at<float>(0,0);
    point_3d.y = XY.at<float>(1,0);
    point_3d.z = Z;
    cam_T_point.push_back(point_3d);
  }

  for (int i = 0; i < good_matches.size(); i++)
  {
    std::cout << points_1[i] << ": " << disparities[i] << ": XYZ " << cam_T_point[i] << "\n";
    cv::putText(img_goodmatch, 
                std::to_string(cam_T_point[i].z).substr(0,4), 
                points_1[i],
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                cv::Scalar(255, 255, 255),
                2);
    cv::putText(img_goodmatch, 
                std::to_string(cam_T_point[i].z).substr(0,4), 
                points_1[i],
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                cv::Scalar(0, 0, 0),
                1);
  }
  imshow("Good matches with depth", img_goodmatch);

  cv::waitKey(0);
  return 0;
}

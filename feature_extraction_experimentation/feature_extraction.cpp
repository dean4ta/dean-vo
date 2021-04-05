#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "usage: feature_extraction img1 img2" << std::endl;
    return 1;
  }
  cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

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
  for (int i = 0; i < descriptors_1.rows; i++)
  {
    if (matches[i].distance <= distance_thresh)
    {
      good_matches.push_back(matches[i]);
    }
  }

  // Show the filtered match
  cv::Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  cv::imwrite("img_goodmatch.png", img_goodmatch);
  imshow("Good matches", img_goodmatch);

  cv::waitKey(0);
  return 0;
}

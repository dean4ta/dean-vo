#pragma once

#include <iomanip>
#include <sstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dean-vo/types.hpp"

// Abstract Class
class ImageServer
{
protected:
  std::string meta_data_;

public:
  ImageServer(std::string meta_data)
  {
    meta_data_ = meta_data;
  }

  virtual void getNextImage(image_frame& frame) = 0;
};

class KittiImageServer : public ImageServer
{
public:
  KittiImageServer(std::string meta_data, int number_of_images) : ImageServer(meta_data)
  {
    number_of_images_ = number_of_images;
  }

  void getNextImage(image_frame& frame)
  {
    if (frame_number_ > number_of_images_)
    {
      frame.timestamp = -1;
      return;
    }
    try
    {
      std::stringstream ss;
      ss << std::setw(10) << std::setfill('0') << frame_number_;
      std::string image_name = ss.str() + ".png";
      std::string img_l_path = std::string(meta_data_) + "image_00/data/" + image_name;
      std::string img_r_path = std::string(meta_data_) + "image_01/data/" + image_name;
      frame.images.resize(2);
      frame.images[0] = cv::imread(img_l_path, cv::IMREAD_GRAYSCALE);
      frame.images[1] = cv::imread(img_r_path, cv::IMREAD_GRAYSCALE);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    frame.timestamp = frame_number_ * 0.1;
    ++frame_number_;
  }

private:
  int frame_number_ = 0;
  int number_of_images_;
};
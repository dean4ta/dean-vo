// #include <iostream>
// #include <sstream>
// #include <iomanip>
// #include <opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc.hpp>

#include "dean-vo/vo.hpp"

int main(int argc, char **argv)
{
  VO vo(argv[1]);
  if (argc != 2)
  {
    std::cout << "usage: vo test\n";
    return -1;
  }
  image_frame frame;
  vo.start();
  return 0;
}
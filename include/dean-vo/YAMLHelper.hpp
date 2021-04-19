#pragma once

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>

// namespace YAML
// {
// /**
//  * @brief Conversion object to and from YAML::Node to ResolutionConfig. Allows you read YAML entry
//  * directly into the object like ResolutionConfig rc = yaml_node.as<ResolutionConfig>();
//  *
//  * @tparam
//  */
// template <>
// struct convert<ResolutionConfig>
// {
//   static Node encode(const ResolutionConfig& rhs)
//   {
//     Node node;
//     node["width"] = rhs.width;
//     node["height"] = rhs.height;
//     node["fps"] = rhs.fps;
//     return node;
//   }

//   static bool decode(const Node& node, ResolutionConfig& rhs)
//   {
//     if (!node.IsMap() || node.size() != 3)
//     {
//       return false;
//     }

//     rhs.width = node["width"].as<int>();
//     rhs.height = node["height"].as<int>();
//     rhs.fps = node["fps"].as<int>();
//     return true;
//   }
// };

// /**
//  * @brief Conversion object to and from YAML::Node to cv::Matx33d. Allows you read YAML entry
//  * directly into the object like cv::Matx33d h = yaml_node.as<cv::Matx33d>(). Can be converted to
//  * cv::Mat a shallow copy cv::Mat h2 = cv::Mat(h)
//  *
//  * @tparam
//  */
// template <>
// struct convert<cv::Matx33d>
// {
//   static Node encode(const cv::Matx33d& rhs)
//   {
//     Node node;

//     for (int r = 0; r < rhs.rows; ++r)
//     {
//       for (int c = 0; c < rhs.cols; ++c)
//       {
//         node.push_back(rhs(r, c));
//       }
//     }
//     return node;
//   }

//   static bool decode(const Node& node, cv::Matx33d& rhs)
//   {
//     if (!node.IsSequence() || node.size() != 9)
//     {
//       return false;
//     }

//     rhs = cv::Matx33d(node[0].as<double>(),
//                       node[1].as<double>(),
//                       node[2].as<double>(),
//                       node[3].as<double>(),
//                       node[4].as<double>(),
//                       node[5].as<double>(),
//                       node[6].as<double>(),
//                       node[7].as<double>(),
//                       node[8].as<double>());

//     return true;
//   }
// };

// }  // namespace YAML

// /**
//  * @brief Strucure to store the calibration parameters
//  *
//  */
// struct CalibrationParams
// {
//   int image_width;   //< Width the image used for calibration
//   int image_height;  //< Height of the image used for calibration

//   std::vector<uint64_t> serials;                           //< Vector of serials as uint64_t
//   std::unordered_map<uint64_t, cv::Mat> homographies_map;  //< Map of homographies with serials as
//                                                            // key

//   std::vector<int> target_center;  //< The image coordinates of the center of the calibration
//                                    // target on the output canvas in the order x, y

//   std::string timestamp;  //< The timestamp when the calibration was last performed

//   /**
//    * @brief Read the YAML config file into the system
//    *
//    * @param config_path Path to the config folder. Does not need the filename.
//    * @return true
//    * @return false
//    */
//   bool read(std::string config_path, std::string filename = "calibration.yaml")
//   {
//     serials.clear();
//     homographies_map.clear();

//     std::string config_file = config_path + filename;
//     try
//     {
//       auto node = YAML::LoadFile(config_file);
//       image_width = node["image_width"].as<int>();
//       image_height = node["image_height"].as<int>();

//       auto data = node["calibration_data"];
//       for (size_t idx = 0; idx < data.size(); ++idx)
//       {
//         auto serial = std::stoull(data[idx]["serial_id"].as<std::string>());
//         auto homography = cv::Mat(data[idx]["homography"].as<cv::Matx33d>());

//         serials.push_back(serial);
//         homographies_map[serial] = homography;
//       }

//       auto target_center_node = node["target_center"];
//       if (target_center_node.size() != 2)
//       {
//         return false;
//       }

//       target_center = target_center_node.as<std::vector<int>>();
//       timestamp = node["timestamp"].as<std::string>();

//       return true;
//     }
//     catch (YAML::ParserException& e)
//     {
//       std::cerr << "Error reading calibration config file: " << e.what() << std::endl;
//       return false;
//     }
//     catch (...)
//     {
//       std::cerr << "Error reading calibration config file" << std::endl;
//       return false;
//     }
//     return true;
//   }

//   /**
//    * @brief Write the parameters into a YAML file
//    *
//    * @param config_path Path of the configuration folder. Filename is not required.
//    * @return true
//    * @return false
//    */
//   bool write(std::string config_path, std::string filename = "calibration.yaml")
//   {
//     YAML::Node node;
//     node["image_width"] = image_width;
//     node["image_height"] = image_height;

//     node["target_center"] = target_center;
//     node["timestamp"] = timestamp;

//     YAML::Node calib_data_node;

//     for (auto& item : homographies_map)
//     {
//       YAML::Node calib_item_node;
//       calib_item_node["serial_id"] = item.first;
//       calib_item_node["homography"] = cv::Matx33d(item.second);
//       calib_data_node.push_back(calib_item_node);
//     }

//     node["calibration_data"] = calib_data_node;

//     std::ofstream calib_out_stream(config_path + filename);
//     calib_out_stream << node;

//     return true;
//   }
// };

// /**
//  * @brief Structure to store the camera parameters
//  *
//  */
// struct CameraParams
// {
//   ResolutionConfig camera_resolution;         //< Input resolution configuration of the cameras
//   ResolutionConfig output_canvas_resolution;  //< Internal ouput resolution
//   ResolutionConfig output_resolution;         //< Ouput resolution configuration

//   int number_of_cameras;
//   double zoom;          //< desired width of the calibration tool in the image as a ratio
//   double vertical_pan;  //< desired vertical pan of the
//   std::vector<int> center_mask_size;

//   /**
//    * @brief Read the camera configuration from the YAML file
//    *
//    * @param config_path Path of the configuration folder. Filename is not required.
//    * @return true
//    * @return false
//    */
//   bool read(std::string config_path, std::string filename = "cameras.yaml")
//   {
//     std::string config_file = config_path + filename;

//     auto node = YAML::LoadFile(config_file);
//     try
//     {
//       number_of_cameras = node["number_of_cameras"].as<int>();
//       zoom = node["zoom"].as<double>();
//       vertical_pan = node["vertical_pan"].as<double>();

//       output_canvas_resolution = node["output_canvas_resolution"].as<ResolutionConfig>();
//       output_resolution = node["output_resolution"].as<ResolutionConfig>();

//       auto camera_resolution_string = node["camera_resolution_config"].as<std::string>();
//       camera_resolution =
//           node["resolutions_config_definitions"][camera_resolution_string].as<ResolutionConfig>();

//       center_mask_size = node["center_mask_size"].as<std::vector<int>>();

//       return true;
//     }
//     catch (YAML::ParserException& e)
//     {
//       std::cerr << "Error reading camera config file: " << e.what() << std::endl;
//       return false;
//     }
//     catch (...)
//     {
//       std::cerr << "Error reading camera config file. " << std::endl;
//       return false;
//     }
//   }
// };
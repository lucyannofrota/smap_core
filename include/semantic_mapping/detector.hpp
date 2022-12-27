#ifndef SEMANTIC_MAPPING__DETECTOR_HPP_
#define SEMANTIC_MAPPING__DETECTOR_HPP_

#include "visibility_control.h"

#include "stdio.h"

#include "rclcpp/rclcpp.hpp"

namespace semantic_mapping
{

// Can be a scene or object detector

class Detector
{
private:
  rclcpp::Logger logger = rclcpp::get_logger("Concept");

public:
  Detector();

  virtual ~Detector();
};

}  // namespace semantic_mapping

#endif  // SEMANTIC_MAPPING__DETECTOR_HPP_

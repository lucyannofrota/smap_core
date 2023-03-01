#ifndef SEMANTIC_MAPPING__THING_HPP_
#define SEMANTIC_MAPPING__THING_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <iostream>

#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

#include "rclcpp/rclcpp.hpp"

namespace semantic_mapping
{

enum semantic_type {OBJECT, LOCATION};

// Can be a scene or object thing

class thing
{
public:
  // Attributes
  semantic_type type = semantic_type::LOCATION;


  // Methods
  thing();
  thing(semantic_type type);
  virtual ~thing();

  std::string get_label();

  std::pair<std::string, std::string> get_vertex_representation();

private:
  friend class boost::serialization::access;
  rclcpp::Logger logger = rclcpp::get_logger("thing");
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    ar & type;
  }
};

}  // namespace semantic_mapping

BOOST_CLASS_VERSION(semantic_mapping::thing, 0)

#endif  // SEMANTIC_MAPPING__THING_HPP_

#ifndef SEMANTIC_MAPPING__CONCEPT_HPP_
#define SEMANTIC_MAPPING__CONCEPT_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <iostream>

// #include <boost/serialization/serialization.hpp>
// #include <boost/container/list.hpp>
// #include <boost/serialization/list.hpp>
// #include <boost/serialization/
// #include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

namespace semantic_mapping
{

enum semantic_type {OBJECT, LOCATION};

// Can be a scene or object concept

class Concept
{
public:
  // Attributes
  semantic_type type = semantic_type::LOCATION;


  // Methods
  Concept();
  virtual ~Concept();


  std::string get_label();

  std::pair<std::string, std::string> get_vertex_representation();

private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    ar & type;
  }
};

}  // namespace semantic_mapping

BOOST_CLASS_VERSION(semantic_mapping::Concept, 0)

#endif  // SEMANTIC_MAPPING__concept_HPP_

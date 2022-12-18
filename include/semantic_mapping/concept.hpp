#ifndef SEMANTIC_MAPPING__CONCEPT_HPP_
#define SEMANTIC_MAPPING__CONCEPT_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <iostream>

namespace semantic_mapping
{

enum semantic_type{OBJECT,LOCATION};

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

  std::pair<std::string,std::string> get_vertex_representation();

};

}  // namespace semantic_mapping

#endif  // SEMANTIC_MAPPING__concept_HPP_

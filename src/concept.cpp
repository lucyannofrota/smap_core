#include "../include/semantic_mapping/concept.hpp"

namespace semantic_mapping
{

Concept::Concept()
{
}

Concept::~Concept()
{
}

std::string Concept::get_label(void){
  return std::string("Corridor");
}

}  // namespace semantic_mapping

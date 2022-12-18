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

std::pair<std::string,std::string> Concept::get_vertex_representation(){
  return std::pair<std::string,std::string>(std::string("Corridor"),std::string("red"));
}

}  // namespace semantic_mapping

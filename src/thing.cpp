#include "../include/semantic_mapping/thing.hpp"

namespace semantic_mapping
{

thing::thing()
{
}

thing::thing(semantic_type type){
  this->type = type;
}

thing::~thing()
{
}



std::string thing::get_label(void)
{
  return std::string("Corridor");
}

std::pair<std::string, std::string> thing::get_vertex_representation()
{
  return std::pair<std::string, std::string>(std::string("Corridor"), std::string("red"));
}

}  // namespace semantic_mapping

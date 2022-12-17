#ifndef SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_
#define SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/container/list.hpp>
#include <string>

#include "../include/semantic_mapping/concept.hpp"

namespace semantic_mapping
{

struct VertexData
{
  const long index = 0;
  Concept this_thing;
  boost::container::list<Concept> related_things;
  // boost::vertex_color_t color = boost::default_color_type::red_color;
};



struct EdgeData
{
  // The cost of the edge will be distance*modifier
  // 
  const double distance = 0; 
  double modifier = 1;
  double cost = 0;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexData, EdgeData> TopoMap;

class Conceptual_Map
{
public:
  TopoMap Semantic_Graph;

  Conceptual_Map();

  virtual ~Conceptual_Map();

  void export_ThingsGraph(const std::string &f_name);

  void export_TopoGraph(const std::string &f_name);
};

}  // namespace semantic_mapping

#endif  // SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_

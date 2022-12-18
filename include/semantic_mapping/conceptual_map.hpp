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
#include "../include/semantic_mapping/label_writers.hpp"


namespace semantic_mapping
{

struct VertexData
{
  const long index = 0;
  Concept this_thing;
  boost::container::list<Concept> related_things;
};

struct EdgeData
{
  // The cost of the edge will be distance*modifier
  // 
  const double distance = 0; 
  double modifier = 1;

  double get_cost(void){
    return round(distance*modifier*100)/100.0;
  }
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, 
                              boost::undirectedS,
                              VertexData,
                              EdgeData
                              > TopoMap;

class Conceptual_Map
{
public:
  VertexData *current_vertex = NULL;
  VertexData *previous_vertex = NULL;

  TopoMap Semantic_Graph;

  Conceptual_Map();

  virtual ~Conceptual_Map();

  void add_vertex();

  void export_ThingsGraph(const std::string &f_name);

  void export_TopoGraph(const std::string &f_name);

private:
  inline size_t _get_boost_index(VertexData* ptr);
};









}  // namespace semantic_mapping

#endif  // SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_

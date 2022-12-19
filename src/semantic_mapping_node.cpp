#include <cstdio>
#include "../include/semantic_mapping/detector.hpp"

#include "../include/semantic_mapping/conceptual_map.hpp"

#include "../include/semantic_mapping/concept.hpp"

#include "../include/semantic_mapping/macros.hpp"



#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

// geometry_msgs::msg::Point
// Node responsible to manage all the services


#include <fstream>
#include <iostream>

#include "../include/semantic_mapping/macros.hpp"



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  semantic_mapping::Conceptual_Map a,b;
  for(int i = 0; i < 5; i++){
    a.add_vertex();
  }

  a.export_TopoGraph("TopoGraph_bA");
  b.export_TopoGraph("TopoGraph_bB");
  semantic_mapping::Conceptual_Map::save_map(a);

  semantic_mapping::Conceptual_Map::load_map(b);

  

  a.export_TopoGraph("TopoGraph_aA");
  b.export_TopoGraph("TopoGraph_aB");





  printf("hello world semantic_mapping package\n");
  return 0;
}

#include <cstdio>
#include "../include/semantic_mapping/detector.hpp"

#include "../include/semantic_mapping/conceptual_map.hpp"

#include "../include/semantic_mapping/concept.hpp"

#include "../include/semantic_mapping/macros.hpp"



#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>



// Node responsible to manage all the services


#include <fstream>
#include <iostream>





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
  std::ofstream ofs(OUTPUT_PATH+"filename");
  boost::archive::text_oarchive oa(ofs);
  oa << a;


  ofs.close();

  std::ifstream ifs(OUTPUT_PATH+"filename");
  boost::archive::text_iarchive ia(ifs);

  ia >> b;

  ifs.close();
  

  a.export_TopoGraph("TopoGraph_aA");
  b.export_TopoGraph("TopoGraph_aB");





  printf("hello world semantic_mapping package\n");
  return 0;
}

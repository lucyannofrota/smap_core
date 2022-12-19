#include "../include/semantic_mapping/conceptual_map.hpp"

namespace semantic_mapping
{




Conceptual_Map::Conceptual_Map()
{
  // for(int i = 0; i < 5; i++){
  //   this->add_vertex();
  // }


  // this->export_TopoGraph("TopoGraph");

  // Serialization
}


Conceptual_Map::~Conceptual_Map()
{
}

void Conceptual_Map::add_vertex(void)  // Must receive current position as parameter
{
  long v_index;
  if(current_vertex == NULL) v_index = 0;
  else v_index = current_vertex->index+1;

  size_t idx = boost::add_vertex(
    {
      v_index,
      Concept(),
      std::list<Concept>()
    },
    Semantic_Graph
  );
  previous_vertex = current_vertex;
  current_vertex = &(Semantic_Graph[idx]);
  
  if(previous_vertex == NULL) return;


  //Add Edge
  // Calculate distance
    boost::add_edge(
      _get_boost_index(previous_vertex), idx,
      {
        random_double_in_range(0,6),
        random_double_in_range(1,3)
      },
      Semantic_Graph);
}

void Conceptual_Map::export_TopoGraph(const std::string & f_name)
{
  // const std::string path = "src/semantic_mapping/Outputs/";
  std::ofstream dotfile(OUTPUT_PATH + f_name + ".dot");

  write_graphviz(
    dotfile, Semantic_Graph,
    make_vertex_label_writer(boost::get(&VertexData::this_thing, Semantic_Graph)),
    make_cost_label_writer(boost::get(&EdgeData::distance, Semantic_Graph),boost::get(&EdgeData::modifier, Semantic_Graph))
  );

  if (std::system(("dot -Tpng " + OUTPUT_PATH + f_name + ".dot > " + OUTPUT_PATH + f_name + ".png").c_str()) ==
    0)
  {
    if (std::system(("rm " + OUTPUT_PATH + f_name + ".dot").c_str()) == 0) {
      std::cout << "[Export Complete] png file successfully exported to: \n\t" + OUTPUT_PATH + f_name +
          ".png" << std::endl;
    }
  }

}





















inline size_t Conceptual_Map::_get_boost_index(VertexData * ptr)
{
  auto v_pair = boost::vertices(Semantic_Graph);
  auto iter = v_pair.first;
  for (; iter != v_pair.second; iter++) {
    if (Semantic_Graph[*iter].index == ptr->index) break;
  }
  return *iter;
}

inline size_t Conceptual_Map::_get_boost_index(long idx)
{
  auto v_pair = boost::vertices(Semantic_Graph);
  auto iter = v_pair.first;
  for (; iter != v_pair.second; iter++) {
    if (Semantic_Graph[*iter].index == idx) break;
  }
  return *iter;
}

}  // namespace semantic_mapping

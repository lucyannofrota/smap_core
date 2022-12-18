#include "../include/semantic_mapping/conceptual_map.hpp"

namespace semantic_mapping
{




Conceptual_Map::Conceptual_Map()
{
  int i;
  for (i = 0; i < 6; i++) {
    boost::add_vertex(Semantic_Graph);
  }
  boost::vertices(Semantic_Graph);

  boost::add_edge(0, 1, {5, 1}, Semantic_Graph);
  boost::add_edge(2, 3, {2, 2}, Semantic_Graph);
  boost::add_edge(3, 4, {3, 0.5}, Semantic_Graph);
  boost::add_edge(4, 2, {10, 3}, Semantic_Graph);
  boost::add_edge(5, 0, {7, 0.1}, Semantic_Graph);

  this->add_vertex();
  this->add_vertex();

  this->export_TopoGraph("TopoGraph");
}


Conceptual_Map::~Conceptual_Map()
{
}

void Conceptual_Map::add_vertex(void)  // Must receive current position as parameter
{
  if (current_vertex == NULL) {
    size_t idx = boost::add_vertex(
      {
        0,
        Concept(),
        boost::container::list<Concept>()
      },
      Semantic_Graph
    );
    current_vertex = &(Semantic_Graph[idx]);
  } else {
    size_t idx = boost::add_vertex(
      {
        current_vertex->index + 1,
        Concept(),
        boost::container::list<Concept>()
      },
      Semantic_Graph
    );
    previous_vertex = current_vertex;
    current_vertex = &(Semantic_Graph[idx]);

    double v1 = 10 * rand() / (RAND_MAX * 1.0), v2 = rand() / (RAND_MAX * 1.0);
    boost::add_edge(
      _get_boost_index(previous_vertex), idx,
      {
        v1,
        v2
      },
      Semantic_Graph);
  }
}

void Conceptual_Map::export_TopoGraph(const std::string & f_name)
{
  const std::string path = "src/semantic_mapping/Outputs/";
  std::ofstream dotfile(path + f_name + ".dot");

  write_graphviz(
    dotfile, Semantic_Graph,
    make_vertex_label_writer(boost::get(&VertexData::this_thing, Semantic_Graph)),
    make_cost_label_writer(boost::get(&EdgeData::distance, Semantic_Graph),boost::get(&EdgeData::modifier, Semantic_Graph))
  );

  if (std::system(("dot -Tpng " + path + f_name + ".dot > " + path + f_name + ".png").c_str()) ==
    0)
  {
    if (std::system(("rm " + path + f_name + ".dot").c_str()) == 0) {
      std::cout << "[Export Complete] png file successfully exported to: \n\t" + path + f_name +
          ".png" << std::endl;
    }
  }

}


inline size_t Conceptual_Map::_get_boost_index(VertexData * ptr)
{
  auto v_pair = boost::vertices(Semantic_Graph);
  size_t idx = 0;
  for (auto iter = v_pair.first; iter != v_pair.second; iter++, idx++) {
    if (&(Semantic_Graph[*iter].index) == &(ptr->index)) {break;}
  }
  return idx;
}

}  // namespace semantic_mapping

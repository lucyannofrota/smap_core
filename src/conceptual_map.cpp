#include "../include/semantic_mapping/conceptual_map.hpp"

namespace semantic_mapping
{

Conceptual_Map::Conceptual_Map()
{
  int i;
  for (i = 0; i < 6; i++) {
    add_vertex(Semantic_Graph);
  }
  vertices(Semantic_Graph);

  add_edge(0, 1, {5, 1}, Semantic_Graph);
  add_edge(2, 3, {2, 2}, Semantic_Graph);
  add_edge(3, 4, {3, 0.5}, Semantic_Graph);
  add_edge(4, 2, {10, 3}, Semantic_Graph);
  add_edge(5, 0, {7, 0.1}, Semantic_Graph);

  std::vector<double> edge_cost;
  auto map_distances = get(&EdgeData::distance, Semantic_Graph);
  auto map_mod = get(&EdgeData::modifier, Semantic_Graph);
  auto map_cost = get(&EdgeData::cost, Semantic_Graph);
  auto epair = edges(Semantic_Graph);
  for (auto iter = epair.first; iter != epair.second; iter++) {
    put(map_cost, *iter, round(get(map_distances, *iter) * get(map_mod, *iter)*100)/100);
  }

  this->export_TopoGraph("TopoGraph");
}


Conceptual_Map::~Conceptual_Map()
{
}

void Conceptual_Map::export_TopoGraph(const std::string & f_name)
{
  int i;
  const std::string path = "src/semantic_mapping/Outputs/";
  std::ofstream dotfile(path + f_name + ".dot");


  // Vertex
  auto map_vertex = get(&VertexData::this_thing,Semantic_Graph);

  const int n_vertex = Semantic_Graph.m_vertices.size();

  std::string *vertex_name = new std::string[n_vertex];
  for(i = 0; i < n_vertex; i++){
    vertex_name[i] = map_vertex[i].get_label()+std::to_string(i);
  }

  write_graphviz(
    dotfile, Semantic_Graph,
    boost::make_label_writer(vertex_name),
    boost::make_label_writer(get(&EdgeData::cost, Semantic_Graph))
  );

  if(std::system(("dot -Tpng " + path + f_name + ".dot > " + path + f_name + ".png").c_str()) == 0){
    if(std::system(("rm " + path + f_name + ".dot").c_str()) == 0)
      std::cout << "[Export Complete] png file successfully exported to: "+path + f_name + ".png" << std::endl;
  }

}

}  // namespace semantic_mapping

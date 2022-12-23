#include "../include/semantic_mapping/conceptual_map.hpp"

namespace semantic_mapping
{


Conceptual_Map::Conceptual_Map()
{
}

Conceptual_Map::~Conceptual_Map()
{
  delete initial_point;
}


void Conceptual_Map::set_logger(rclcpp::Logger & logger)
{
  this->logger = &logger;
}

void Conceptual_Map::add_vertex(void)  // TODO Remove
{
  long v_index;
  if (current_vertex == NULL) {v_index = 0;} else {v_index = current_vertex->index + 1;}

  size_t idx = boost::add_vertex(
    {
      v_index,
      Concept(),
      geometry_msgs::msg::Point(),
      std::list<Concept>()
    },
    Semantic_Graph
  );
  previous_vertex = current_vertex;
  current_vertex = &(Semantic_Graph[idx]);

  if (previous_vertex == NULL) {return;}


  //Add Edge
  // Calculate distance
  boost::add_edge(
    _get_TopoMap_index(previous_vertex), idx,
    {
      random_double_in_range(0, 6),
      random_double_in_range(1, 3)
    },
    Semantic_Graph);
}

void Conceptual_Map::add_vertex(const geometry_msgs::msg::Point & pos)
{
  if (initial_point == NULL) {
    initial_point = new geometry_msgs::msg::Point;
    initial_point->x = pos.x;
    initial_point->y = pos.y;
    initial_point->z = pos.z;
    RCLCPP_DEBUG(*logger, "Init_point");
    return;
  }


  double distance;
  if (initialization) {
    RCLCPP_DEBUG(
      *logger, "Initialization ip(%4.1f,%4.1f,%4.1f) ap(%4.1f,%4.1f,%4.1f)",
      initial_point->x, initial_point->y, initial_point->z, pos.x, pos.y, pos.z);
    distance = sqrt(
      pow(initial_point->x - pos.x, 2) +
      pow(initial_point->y - pos.y, 2) +
      pow(initial_point->z - pos.z, 2)
    );

    if (distance < VERTEX_DISTANCE) {return;} else {initialization = false;}
  }


  long v_index = 0;
  size_t idx;
  if (current_vertex == NULL) {
    auto v_pair = boost::vertices(Semantic_Graph);
    RCLCPP_DEBUG(*logger, "current_vertex == NULL");
    for (auto iter = v_pair.first; iter != v_pair.second; iter++) {
      distance = sqrt(
        pow(Semantic_Graph[*iter].pos.x - pos.x, 2) +
        pow(Semantic_Graph[*iter].pos.y - pos.y, 2) +
        pow(Semantic_Graph[*iter].pos.z - pos.z, 2)
      );
      if (Semantic_Graph[*iter].index > v_index) {v_index = Semantic_Graph[*iter].index;}
      if (distance <= VERTEX_DISTANCE) {
        current_vertex = &(Semantic_Graph[*iter]);
        return;
      }
      v_index++;
    }
    idx = boost::add_vertex(
      {
        v_index,
        Concept(),
        pos,
        std::list<Concept>()
      },
      Semantic_Graph
    );

    RCLCPP_DEBUG(*logger, "First Vertex added");
    current_vertex = &(Semantic_Graph[idx]);
    return;
  }

  RCLCPP_DEBUG(*logger, "current_vertex != NULL");

  v_index = current_vertex->index + 1;

  distance = sqrt(
    pow(current_vertex->pos.x - pos.x, 2) +
    pow(current_vertex->pos.y - pos.y, 2) +
    pow(current_vertex->pos.z - pos.z, 2)
  );


  if (distance < VERTEX_DISTANCE) return;

  idx = boost::add_vertex(
    {
      v_index,
      Concept(),
      pos,
      std::list<Concept>()
    },
    Semantic_Graph
  );
  RCLCPP_DEBUG(*logger, "Vertex added");
  previous_vertex = current_vertex;
  current_vertex = &(Semantic_Graph[idx]);

  distance = sqrt(
    pow(current_vertex->pos.x - previous_vertex->pos.x, 2) +
    pow(current_vertex->pos.y - previous_vertex->pos.y, 2) +
    pow(current_vertex->pos.z - previous_vertex->pos.z, 2)
  );

  //Add Edge
  boost::add_edge(
    _get_TopoMap_index(previous_vertex), idx,
    {
      distance,
      1
    },
    Semantic_Graph);
}

void Conceptual_Map::export_TopoGraph(const std::string & f_name)
{
  std::ofstream dotfile(OUTPUT_PATH + f_name + ".dot");

  write_graphviz(
    dotfile, Semantic_Graph,
    make_vertex_label_writer(boost::get(&VertexData::this_thing, Semantic_Graph)),
    make_cost_label_writer(
      boost::get(&EdgeData::distance, Semantic_Graph),
      boost::get(&EdgeData::modifier, Semantic_Graph))
  );

  if (std::system(
      ("dot -Tpng " + OUTPUT_PATH + f_name + ".dot > " + OUTPUT_PATH + f_name +
      ".png").c_str()) ==
    0)
  {
    if (std::system(("rm " + OUTPUT_PATH + f_name + ".dot").c_str()) == 0) {
      // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
      RCLCPP_INFO(
        *logger,
        "\033[42m[Export Complete]\033[0m png file successfully exported to: %s.png", std::string(
          OUTPUT_PATH + f_name).c_str());
    }
  }

}

void Conceptual_Map::save_map(Conceptual_Map & obj)
{
  Conceptual_Map::save_map(obj, DEFAULT_FILE_NAME);
}

void Conceptual_Map::save_map(Conceptual_Map & obj, std::string file_name)
{
  std::ofstream ofs(SAVE_LOAD_PATH + file_name);
  boost::archive::text_oarchive oa(ofs);
  oa << obj;
  ofs.close();
}

void Conceptual_Map::load_map(Conceptual_Map & obj)
{
  Conceptual_Map::load_map(obj, DEFAULT_FILE_NAME);
}

void Conceptual_Map::load_map(Conceptual_Map & obj, std::string file_name)
{
  std::ifstream ifs(SAVE_LOAD_PATH + file_name);
  boost::archive::text_iarchive ia(ifs);
  ia >> obj;
  obj.previous_vertex = NULL;
  obj.current_vertex = NULL;
  ifs.close();
}


inline size_t Conceptual_Map::_get_TopoMap_index(VertexData * ptr)
{
  auto v_pair = boost::vertices(Semantic_Graph);
  auto iter = v_pair.first;
  for (; iter != v_pair.second; iter++) {
    if (Semantic_Graph[*iter].index == ptr->index) {break;}
  }
  return *iter;
}

inline size_t Conceptual_Map::_get_TopoMap_index(long idx)
{
  auto v_pair = boost::vertices(Semantic_Graph);
  auto iter = v_pair.first;
  for (; iter != v_pair.second; iter++) {
    if (Semantic_Graph[*iter].index == idx) {break;}
  }
  return *iter;
}

}  // namespace semantic_mapping

#include "../include/semantic_mapping/conceptual_map.hpp"

namespace semantic_mapping
{


Conceptual_Map::Conceptual_Map()
: Node("conceptual_map")
{
  RCLCPP_INFO(this->get_logger(), "Initializing conceptual_map");

  if (NEW_EDGE_FACTOR > 1) {
    RCLCPP_ERROR(this->get_logger(), "NEW_EDGE_FACTOR must be <= 1");
  }

  publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>(
    "semantic_mapper/conceptual_map/graph_nodes", 10);

  // Marker msg initialization
  // Vertex
  vertex_marker.header.frame_id = "/map";
  vertex_marker.ns = "graph_map_vertices";
  vertex_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  vertex_marker.action = visualization_msgs::msg::Marker::ADD;
  vertex_marker.scale.x = 0.075; vertex_marker.scale.y = 0.075; vertex_marker.scale.z = 0.075;
  vertex_marker.color.r = 102.0 / (102.0 + 51.0); vertex_marker.color.g = 51.0 / (102.0 + 51.0);
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;

  // Edge
  edge_marker.header.frame_id = "/map";
  edge_marker.ns = "graph_map_edges";
  edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.scale.x = 0.005; edge_marker.scale.y = 0.005; edge_marker.scale.z = 0.005;
  edge_marker.color.r = 1.0; edge_marker.color.g = 0.0; edge_marker.color.b = 0.0;
  edge_marker.pose.orientation.x = 0; edge_marker.pose.orientation.y = 0;
  edge_marker.pose.orientation.z = 0; edge_marker.pose.orientation.w = 1.0;
  edge_marker.color.a = 1.0;


  // timer = this->create_wall_timer(
  //   std::chrono::seconds(1),
  //   std::bind(
  //     &Conceptual_Map::timer_callback,
  //     this
  //   )
  // );
}

Conceptual_Map::~Conceptual_Map()
{
  // delete initial_point;
  this->export_TopoGraph("TopoGraph");
}

void Conceptual_Map::on_process(void)
{
  if (publish_vertex) {
    publisher_marker->publish(vertex_marker);
    RCLCPP_DEBUG(this->get_logger(), "Vertex Marker Published");
    publish_vertex = false;
  }
  if (publish_edge) {
    publisher_marker->publish(edge_marker);
    RCLCPP_DEBUG(this->get_logger(), "Edge Marker Published");
    publish_edge = false;
  }
}

void Conceptual_Map::add_vertex_(const geometry_msgs::msg::Point & pos)
{
  RCLCPP_WARN(this->get_logger(), "Call");
  // Initialization
  // static geometry_msgs::msg::Point initial_point;
  // static bool initialization = true;

  static double dist_current;
  VertexData * closest = _get_valid_close_vertex(pos, NEW_EDGE_FACTOR);
  size_t idx;
  static long v_index = 0;
  if (current_vertex == NULL) {
    RCLCPP_DEBUG(this->get_logger(), "current_vertex == NULL");
    if (closest == NULL) {
      idx = this->_add_vertex(v_index++, pos);
      current_vertex = &(Semantic_Graph[idx]);
      return;
    } else {
      current_vertex = closest;
    }
  }

  // current_vertex != NULL
  RCLCPP_DEBUG(this->get_logger(), "current_vertex != NULL");
  if ((closest != NULL) && (closest != current_vertex) /* && (closest != previous_vertex)*/) {
    RCLCPP_DEBUG(this->get_logger(), "(closest != current_vertex) && (closest != previous_vertex)");
    _add_edge(
      _get_TopoMap_index(current_vertex),
      _get_TopoMap_index(closest)
    );
    previous_vertex = current_vertex;
    current_vertex = closest;
    RCLCPP_DEBUG(
      this->get_logger(), "prev/cur [%i,%i]", previous_vertex->index, current_vertex->index);
    return;
  }

  dist_current = sqrt(
    pow(current_vertex->pos.x - pos.x, 2) +
    pow(current_vertex->pos.y - pos.y, 2) +
    pow(current_vertex->pos.z - pos.z, 2)
  );

  if (dist_current < VERTEX_DISTANCE) {return;}
  RCLCPP_DEBUG(this->get_logger(), "dist_current >= VERTEX_DISTANCE");
  // dist_current >= VERTEX_DISTANCE
  idx = _add_vertex(v_index++, pos);
  previous_vertex = current_vertex;
  current_vertex = &(Semantic_Graph[idx]);
  RCLCPP_DEBUG(
    this->get_logger(), "prev/cur [%i,%i]", previous_vertex->index,
    current_vertex->index);

  _add_edge(
    _get_TopoMap_index(previous_vertex),
    _get_TopoMap_index(current_vertex)
  );
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
        this->get_logger(),
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

}  // namespace semantic_mapping

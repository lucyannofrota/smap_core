#include "../include/smap_core/topological_map.hpp"


// #include "rcl/error_handling.h"
// #include <rmw/types.h>

namespace smap
{


void topological_map::on_process(void) // Pooling
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

void topological_map::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  // add vertex based on the current pose

  static double dist_current;
  VertexData * closest = _get_valid_close_vertex(pose->pose.position, NEW_EDGE_FACTOR);
  size_t idx;
  static long v_index = 0;
  if (current_vertex == NULL) {
    if (closest == NULL) {
      idx = this->_add_vertex(v_index++, pose->pose.position);
      current_vertex = &(Graph[idx]);
      return;
    } else {
      current_vertex = closest;
    }
  }

  if ((closest != NULL) && (closest != current_vertex)) {
    _add_edge(
      _get_TopoMap_index(current_vertex),
      _get_TopoMap_index(closest)
    );
    previous_vertex = current_vertex;
    current_vertex = closest;
    return;
  }

  dist_current = sqrt(
    pow(current_vertex->pos.x - pose->pose.position.x, 2) +
    pow(current_vertex->pos.y - pose->pose.position.y, 2) +
    pow(current_vertex->pos.z - pose->pose.position.z, 2)
  );

  if (dist_current < VERTEX_DISTANCE) {return;}
  idx = _add_vertex(v_index++, pose->pose.position);
  previous_vertex = current_vertex;
  current_vertex = &(Graph[idx]);

  _add_edge(
    _get_TopoMap_index(previous_vertex),
    _get_TopoMap_index(current_vertex)
  );
}

void topological_map::export_TopoGraph(const std::string & f_name)
{
  std::ofstream dotfile(OUTPUT_PATH + f_name + ".dot");

  write_graphviz(
    dotfile, Graph,
    make_vertex_label_writer(boost::get(&VertexData::this_thing, Graph)),
    make_cost_label_writer(
      boost::get(&EdgeData::distance, Graph),
      boost::get(&EdgeData::modifier, Graph))
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

void topological_map::save_map(topological_map & obj)
{
  topological_map::save_map(obj, DEFAULT_FILE_NAME);
}

void topological_map::save_map(topological_map & obj, std::string file_name)
{
  std::ofstream ofs(SAVE_LOAD_PATH + file_name);
  boost::archive::text_oarchive oa(ofs);
  oa << obj;
  ofs.close();
}

void topological_map::load_map(topological_map & obj)
{
  topological_map::load_map(obj, DEFAULT_FILE_NAME);
}

void topological_map::load_map(topological_map & obj, std::string file_name)
{
  std::ifstream ifs(SAVE_LOAD_PATH + file_name);
  boost::archive::text_iarchive ia(ifs);
  ia >> obj;
  obj.previous_vertex = NULL;
  obj.current_vertex = NULL;
  ifs.close();
}

}  // namespace smap

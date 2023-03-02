#ifndef SEMANTIC_MAPPING__TOPOLOGICAL_MAP_HPP_
#define SEMANTIC_MAPPING__TOPOLOGICAL_MAP_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <list>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/version.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "../include/semantic_mapping/macros.hpp"
#include "../include/semantic_mapping/thing.hpp"
#include "../include/semantic_mapping/label_writers.hpp"

//#include "semantic_mapping/msg/smap_data.hpp"

//#include "../include/semantic_mapping/msg/smap_data.hpp"

/* XXX current_vertex and previous_vertex can be a problem in the future!
       Check based on the location of the robot when loading
*/
/* TODO Parameter D_START
        Distance considered to be in the initialization. While in the range D_START from the first position
        acquired, vertex creation will gonna be blocked
*/

namespace semantic_mapping
{

struct VertexData
{
  long index = 0;
  geometry_msgs::msg::Point pos;
  thing this_thing;
  std::list<thing> related_things;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    ar & index;
    ar & pos.x; ar & pos.y; ar & pos.z;
    ar & this_thing;
    ar & related_things;
  }


};

struct EdgeData
{
  // The cost of the edge will be distance*modifier
  //
  double distance = 0;
  double modifier = 1;

  double get_cost(void)
  {
    return round(distance * modifier * 100) / 100.0;
  }

  friend class boost::serialization::access;
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    ar & distance;
    ar & modifier;
  }
};

typedef boost::adjacency_list<boost::vecS, boost::vecS,
    boost::undirectedS,
    VertexData,
    EdgeData
> TopoMap;

class topological_map : public rclcpp::Node
{

private:
  // Variables
  friend class boost::serialization::access;
  VertexData * current_vertex = NULL;
  VertexData * previous_vertex = NULL;

  TopoMap Semantic_Graph;
  // rclcpp::Logger * logger;

  // rclcpp::TimerBase::SharedPtr timer{nullptr};

  // Subscriptions
  // rclcpp::Subscription<semantic_mapping::msg::SmapData>::SharedPtr SmapData_sub;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;
  visualization_msgs::msg::Marker vertex_marker;
  visualization_msgs::msg::Marker edge_marker;

  bool publish_vertex = false;
  bool publish_edge = false;

private:
  // Internal Functions
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    ar & Semantic_Graph;
  }

  inline size_t _get_TopoMap_index(VertexData * ptr)
  {
    auto v_pair = boost::vertices(Semantic_Graph);
    auto iter = v_pair.first;
    for (; iter != v_pair.second; iter++) {
      if (Semantic_Graph[*iter].index == ptr->index) {break;}
    }
    return *iter;
  }

  inline size_t _get_TopoMap_index(long idx)
  {
    auto v_pair = boost::vertices(Semantic_Graph);
    auto iter = v_pair.first;
    for (; iter != v_pair.second; iter++) {
      if (Semantic_Graph[*iter].index == idx) {break;}
    }
    return *iter;
  }

  inline void _append_vertex_marker(const geometry_msgs::msg::Point & pos)
  {
    static bool init_flag = true;
    static int id = 0;
    // for(auto it = vertex_marker.points.begin(); it != vertex_marker.points.end(); it++){
    //   if((it->x == pos.x) && (it->y == pos.y) && (it->z == pos.z)) return;
    // }
    vertex_marker.header.stamp = this->get_clock().get()->now();
    vertex_marker.id = id; id++;
    vertex_marker.points.push_back(pos);
    if (init_flag) {
      init_flag = false;
      vertex_marker.action = visualization_msgs::msg::Marker::MODIFY;
    }
    publish_vertex = true;
  }

  inline void _append_edge_marker(
    const geometry_msgs::msg::Point & pos1,
    const geometry_msgs::msg::Point & pos2)
  {
    static bool init_flag = true;
    static int id = 0;
    edge_marker.header.stamp = this->get_clock().get()->now();
    edge_marker.id = id; id++;
    if (init_flag) {
      edge_marker.points.push_back(pos1);
      edge_marker.points.push_back(pos2);
      init_flag = false;
      edge_marker.action = visualization_msgs::msg::Marker::MODIFY;
    } else {edge_marker.points.push_back(pos2);}
    publish_edge = true;
  }

  // inline void timer_callback(void){
  //   if(publish_vertex){
  //     publisher_marker->publish(vertex_marker);
  //     RCLCPP_DEBUG(this->get_logger(),"Vertex Marker Published");
  //     publish_vertex = false;
  //   }
  //   if(publish_edge){
  //     publisher_marker->publish(edge_marker);
  //     RCLCPP_DEBUG(this->get_logger(),"Edge Marker Published");
  //     publish_edge = false;
  //   }
  // }

  inline size_t _add_vertex(
    long v_index, const geometry_msgs::msg::Point & pos)
  {
    // semantic_mapping::thing thing;
    // thing.type = semantic_type::LOCATION;
    size_t ret = boost::add_vertex(
      {
        v_index,
        pos,
        thing(semantic_type::LOCATION),
        std::list<thing>()
      },
      Semantic_Graph
    );
    publish_vertex = true;
    _append_vertex_marker(pos);
    RCLCPP_INFO(this->get_logger(), "Vertex added [%4.1f,%4.1f,%4.1f]", pos.x, pos.y, pos.z);
    return ret;
  }

  inline void _add_edge(size_t previous, size_t current)
  {
    double distance = sqrt(
      pow(Semantic_Graph[previous].pos.x - Semantic_Graph[current].pos.x, 2) +
      pow(Semantic_Graph[previous].pos.y - Semantic_Graph[current].pos.y, 2) +
      pow(Semantic_Graph[previous].pos.z - Semantic_Graph[current].pos.z, 2)
    );
    boost::add_edge(
      previous, current,
      {
        distance,
        1
      },
      Semantic_Graph);
    publish_edge = true;
    _append_edge_marker(Semantic_Graph[previous].pos, Semantic_Graph[current].pos);
    RCLCPP_DEBUG(
      this->get_logger(), "Edge added %i->%i [%4.1f,%4.1f,%4.1f]->[%4.1f,%4.1f,%4.1f]", previous, current,
      Semantic_Graph[previous].pos.x, Semantic_Graph[previous].pos.y, Semantic_Graph[previous].pos.z,
      Semantic_Graph[current].pos.x, Semantic_Graph[current].pos.y,
      Semantic_Graph[current].pos.z);
  }

  inline VertexData * _get_valid_close_vertex(const geometry_msgs::msg::Point & pos, double factor)
  {
    auto pair = boost::vertices(Semantic_Graph);
    VertexData * ret = NULL;
    double min = DBL_MAX;
    for (auto it = pair.first; it != pair.second; it++) {
      double distance = sqrt(
        pow(pos.x - Semantic_Graph[*it].pos.x, 2) +
        pow(pos.y - Semantic_Graph[*it].pos.y, 2) +
        pow(pos.z - Semantic_Graph[*it].pos.z, 2)
      );
      if (distance < VERTEX_DISTANCE * factor && distance < min) {
        min = distance;
        ret = &(Semantic_Graph[*it]);
      }
    }
    return ret;
  }

public:
  topological_map(void);

  virtual ~topological_map(void);

  void on_process(void); // Pooling

  void add_vertex(const geometry_msgs::msg::Point & pos);

  void export_ThingsGraph(const std::string & f_name);

  void export_TopoGraph(const std::string & f_name);

  static void save_map(topological_map & obj);

  static void save_map(topological_map & obj, std::string file_name);

  static void load_map(topological_map & obj);

  static void load_map(topological_map & obj, std::string file_name);
};


double random_double_in_range(double min, double max) // temp
{
  return min +
         (double)rand() / RAND_MAX *
         (max - min);
}

}  // namespace semantic_mapping

BOOST_CLASS_VERSION(semantic_mapping::VertexData, 0)
BOOST_CLASS_VERSION(semantic_mapping::EdgeData, 0)
BOOST_CLASS_VERSION(semantic_mapping::topological_map, 0)

#endif  // SEMANTIC_MAPPING__TOPOLOGICAL_MAP_HPP_

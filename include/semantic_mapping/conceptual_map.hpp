#ifndef SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_
#define SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_

#include "visibility_control.h"

#include "stdio.h"
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <string>

#include "../include/semantic_mapping/concept.hpp"
#include "../include/semantic_mapping/label_writers.hpp"


#include "../include/semantic_mapping/macros.hpp"


#include <list>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/version.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

/* XXX current_vertex and previous_vertex can be a problem in the future!
       Check based on the location of the robot when loading
*/
/* TODO Parameter D_START
        Distance considered to be in the initialization. While in the range D_START from the first position
        acquired, vertex creation will gonna be blocked
*/

/* TODO add_vertex() Check distance beteween listed vertices with the new point
*/

namespace semantic_mapping
{

struct VertexData
{
  long index = 0;
  geometry_msgs::msg::Point pos;
  Concept this_thing;
  std::list<Concept> related_things;

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

class Conceptual_Map : public rclcpp::Node
{

private:
  // Variables
  friend class boost::serialization::access;
  VertexData * current_vertex = NULL;
  VertexData * previous_vertex = NULL;

  TopoMap Semantic_Graph;
  rclcpp::Logger * logger;

  // rclcpp::TimerBase::SharedPtr timer{nullptr};

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;
  visualization_msgs::msg::Marker vertex_marker;
  visualization_msgs::msg::Marker edge_marker;

  bool publish_vertex = false;
  bool publish_edge = false;

  geometry_msgs::msg::Point * initial_point = NULL;
  bool initialization = true;

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

  inline void _append_vertex_marker(const geometry_msgs::msg::Point & pos){
    static bool init_flag = true;
    static int id = 0;
    vertex_marker.header.stamp = this->get_clock().get()->now();
    vertex_marker.id = id; id++;
    vertex_marker.points.push_back(pos);
    if(init_flag){
      init_flag = false;
      vertex_marker.action = visualization_msgs::msg::Marker::MODIFY;
    }
    publish_vertex = true;
  }

  inline void _append_edge_marker(const geometry_msgs::msg::Point & pos1, const geometry_msgs::msg::Point & pos2){
    static bool init_flag = true;
    static int id = 0;
    edge_marker.header.stamp = this->get_clock().get()->now();
    edge_marker.id = id; id++;
    if(init_flag){
      edge_marker.points.push_back(pos1);
      edge_marker.points.push_back(pos2);
      init_flag = false;
      edge_marker.action = visualization_msgs::msg::Marker::MODIFY;
    }
    else edge_marker.points.push_back(pos2);
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

  inline size_t _add_vertex(long v_index, const geometry_msgs::msg::Point & pos, Concept this_thing, std::list<Concept> related_things){
    size_t ret = boost::add_vertex(
      {
        v_index,
        pos,
        this_thing,
        related_things
      },
      Semantic_Graph
    );
    publish_vertex = true;
    _append_vertex_marker(pos);
    return ret;
  }

  inline void _add_edge(size_t previous, size_t current, double distance){
    boost::add_edge(
    previous, current,
    {
      distance,
      1
    },
    Semantic_Graph);
    publish_edge = true;
    _append_edge_marker(Semantic_Graph[previous].pos,Semantic_Graph[current].pos);
  }

public:
  Conceptual_Map(void);

  virtual ~Conceptual_Map(void);

  void on_process(void);

  void add_vertex(void);

  void add_vertex(const geometry_msgs::msg::Point & pos);

  void export_ThingsGraph(const std::string & f_name);

  void export_TopoGraph(const std::string & f_name);

  static void save_map(Conceptual_Map & obj);

  static void save_map(Conceptual_Map & obj, std::string file_name);

  static void load_map(Conceptual_Map & obj);

  static void load_map(Conceptual_Map & obj, std::string file_name);
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
BOOST_CLASS_VERSION(semantic_mapping::Conceptual_Map, 0)

#endif  // SEMANTIC_MAPPING__CONCEPTUAL_MAP_HPP_

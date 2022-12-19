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

/* XXX current_vertex and previous_vertex can be a problem in the future! 
       Check based on the location of the robot when loading
*/

namespace semantic_mapping
{

struct VertexData
{
  long index = 0;
  Concept this_thing;
  geometry_msgs::msg::Point pos;
  // boost::container::list<Concept> related_things;
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

class Conceptual_Map
{
public:
  VertexData * current_vertex = NULL;
  VertexData * previous_vertex = NULL;

  TopoMap Semantic_Graph;

  Conceptual_Map(void);

  virtual ~Conceptual_Map(void);

  void add_vertex(void);

  void add_vertex(geometry_msgs::msg::Point & pos);

  void export_ThingsGraph(const std::string & f_name);

  void export_TopoGraph(const std::string & f_name);

  static void save_map(Conceptual_Map &obj);

  static void save_map(Conceptual_Map &obj,std::string file_name);

  static void load_map(Conceptual_Map &obj);

  static void load_map(Conceptual_Map &obj,std::string file_name);

private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    (void) version;
    // ar & current_vertex;
    // ar & previous_vertex;
    ar & Semantic_Graph;
  }

  inline size_t _get_TopoMap_index(VertexData * ptr);

  inline size_t _get_TopoMap_index(long idx);
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

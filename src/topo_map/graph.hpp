#ifndef SMAP_CORE__GRAPH_HPP_
#define SMAP_CORE__GRAPH_HPP_

// STL
// #include <stdlib.h>

// BOOST
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>

// ROS
#include "geometry_msgs/msg/point.hpp"

// SMAP
#include "../thing/thing.hpp"

struct vertex_data_t

{
    size_t index = (size_t) -1;
    geometry_msgs::msg::Point pos;
    smap::thing this_thing;
    std::list< smap::thing > related_things;

    bool strong_vertex = false;

    friend class boost::serialization::access;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& index;
        ar& pos.x;
        ar& pos.y;
        ar& pos.z;
        ar& this_thing;
        ar& related_things;
    }
};

struct edge_data_t
{
    // The cost of the edge will be distance*modifier

    double distance = 0;
    double modifier = 1;

    double get_cost( void ) { return round( distance * modifier * 100 ) / 100.0; }

    friend class boost::serialization::access;

    template< class Archive >
    inline void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& distance;
        ar& modifier;
    }
};

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, vertex_data_t, edge_data_t > graph_t;

// traits
template<>
struct boost::graph::internal_vertex_name< vertex_data_t >
{
    // https://stackoverflow.com/questions/71488845/how-to-configure-boostgraph-to-use-my-own-stable-index-for-vertices
    struct type
    {
        using result_type = size_t;

        const result_type& operator()( const vertex_data_t& bundle ) const { return bundle.index; }
    };
};

template<>
struct boost::graph::internal_vertex_constructor< vertex_data_t >
{
    // https://stackoverflow.com/questions/71488845/how-to-configure-boostgraph-to-use-my-own-stable-index-for-vertices
    struct type
    {
      private:

        using extractor = typename internal_vertex_name< vertex_data_t >::type;
        using name_t    = std::decay_t< typename extractor::result_type >;

      public:

        using argument_type = name_t;
        using result_type   = vertex_data_t;

        result_type operator()( const name_t& index ) const
        {
            result_type ret;
            ret.index = index;
            return ret;
        }
    };
};

#endif  // SMAP_CORE__GRAPH_HPP_

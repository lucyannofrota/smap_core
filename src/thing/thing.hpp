#ifndef SMAP_CORE__THING_HPP_
#define SMAP_CORE__THING_HPP_

// STL

// BOOST
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// SMAP
#include "../include/smap_core/interface_templates.hpp"
#include "../include/smap_core/macros.hpp"
#include "../perception_server/detector_descriptor.hpp"
#include "observation_histogram.hpp"

namespace smap
{

enum semantic_type_t
{
    OBJECT,
    LOCATION
};

// Can be a location or object thing

class thing
{
  public:

    // Attributes
    semantic_type_t type = semantic_type_t::LOCATION;
    observation_histogram observations =
        observation_histogram( HISTOGRAM_BINS );                             // Polar histogram of the observations
    geometry_msgs::msg::Point pos;

    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb;  // min, max

    std::map< std::string, std::pair< int, int > >** reg_classes = nullptr;

    std::map< std::string, float > class_probabilities;

    // Methods
    thing( void ) {}

    thing( std::map< std::string, std::pair< int, int > >** class_map )
    {
        // this->observations = observation_histogram::observation_histogram( 36 );
        this->reg_classes = class_map;
    }

    thing( semantic_type_t type ) { this->type = type; }

    virtual ~thing() {}

    std::string get_label( void ) const;

    double get_confidence( void ) const;

    bool label_is_equal( uint8_t& module_id, uint8_t& obs_label );

    // std::string get_label( uint8_t module_id );

    std::pair< std::string, std::string > get_vertex_representation();

    void set(
        const semantic_type_t type, const std::vector< float >& probability_distribution,
        const geometry_msgs::msg::Point& point, std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb,
        const float& pos_confidence, const double& distance, const double& angle, const detector_t& detector );

    void update(
        const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
        std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
        double distance, double angle, const detector_t& detector );

  private:

    float pos_confidence;

    friend class boost::serialization::access;
    rclcpp::Logger logger = rclcpp::get_logger( "thing" );

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& type;
        // TODO: Implement
    }

    int _get_label( void );
};

}  // namespace smap

BOOST_CLASS_VERSION( smap::thing, 0 )

#endif  // SMAP_CORE__THING_HPP_

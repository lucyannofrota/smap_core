#ifndef SMAP_CORE__THING_HPP_
#define SMAP_CORE__THING_HPP_

// STL
#include <memory>
#include <utility>

// BOOST
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// SMAP
#include "../include/smap_core/aux_functions.hpp"
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
		std::shared_ptr<observation_histogram> observations; // Polar histogram of the observations
		// observation_histogram observations =
    //     observation_histogram( HISTOGRAM_BINS );                             
    geometry_msgs::msg::Point pos;

    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb;  // min, max

    std::shared_ptr< std::map< std::string, std::pair< int, int > > > reg_classes;

    std::map< std::string, float > class_probabilities;

    int id = -1;

    // Methods
    thing( void ) {}

    thing( std::shared_ptr< std::map< std::string, std::pair< int, int > > >& class_map, int id ) :
        observations(std::make_unique<observation_histogram>(HISTOGRAM_BINS)), reg_classes( class_map ), id( id )
				// observations(std::make_unique<observation_histogram>(HISTOGRAM_BINS))
				// o(std::make_unique<observation_histogram>(HISTOGRAM_BINS))
    {
			// observation_histogram observations =
      //   observation_histogram( HISTOGRAM_BINS );                             
        // this->observations = observation_histogram::observation_histogram( 36 );
        // this->reg_classes = class_map;
    }

    thing( semantic_type_t type, int id )
    {
        this->type = type;
        this->id   = id;
    }

    virtual ~thing() {}

    std::string get_label( void ) const;

    inline double get_class_confidence( void ) const
    {
        if( this->get_label() == UNDEFINED_LABEL ) return 0.0;
        return log_odds_inv( this->class_probabilities.at( this->get_label() ) );
    }

    inline double get_position_confidence( void ) const
    {
        if( this->get_label() == UNDEFINED_LABEL ) return 0.0;
        return log_odds_inv( this->pos_confidence );
    }

    inline double get_combined_confidence( void ) const
    {
        if( this->get_label() == UNDEFINED_LABEL ) return 0.0;

        return ( log_odds_inv( this->class_probabilities.at( this->get_label() ) ) / 3 )
             + ( log_odds_inv( this->pos_confidence ) / 3 ) + ( this->observations->get_histogram_ratio() / 3 );
    }

    bool label_is_equal( const uint8_t& module_id, const uint8_t& obs_label );

    // std::string get_label( uint8_t module_id );

    std::pair< std::string, std::string > get_vertex_representation();

    void set(
        const semantic_type_t type, const std::vector< float >& probability_distribution,
        const geometry_msgs::msg::Point& point, std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb,
        const float& pos_confidence, const double& distance, const double& angle, const detector_t& detector );

    geometry_msgs::msg::Point update(
        const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
        std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
        double distance, double angle, const detector_t& detector );

    inline void decay_confidence( const double& distance, const double& factor )
    {
        // current_likelihood - is a map containing a vector of probabilities that represents the probability of beeing
        // each class

        for( auto& class_likelihood: this->class_probabilities )
            class_likelihood.second -= log_odds( ( OBJECT_PROB_DECAY * ( 1 + factor ) ) / ( 1 + distance ) );
    }

    bool is_valid( void ) const;

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

    // int _get_label( void );
};

}  // namespace smap

BOOST_CLASS_VERSION( smap::thing, 0 )

#endif  // SMAP_CORE__THING_HPP_

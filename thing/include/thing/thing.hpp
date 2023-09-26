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
#include <rcpputils/asserts.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// SMAP
#include "macros.hpp"
#include "observation_histogram.hpp"
#include "smap_base/aux_functions.hpp"
#include "smap_base/detector_descriptor.hpp"
#include "smap_interfaces/interface_templates.hpp"

// #include "smap_base/macros.hpp"

namespace smap
{

enum semantic_type_t
{
    OBJECT,
    LOCATION
};

enum thing_state_t
{
    NONE,
    ABSENT,
    OCCLUDED,
    VISIBLE
};

// Can be a location or object thing

class thing
{
  public:

    // Attributes
    semantic_type_t type = semantic_type_t::LOCATION;
    std::shared_ptr< observation_histogram > observations;  // Polar histogram of the observations
                                                            // observation_histogram observations =
    // observation_histogram( HISTOGRAM_BINS );
    geometry_msgs::msg::Point pos;

    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb;  // min, max

    std::shared_ptr< std::map< std::string, std::pair< int, int > > > reg_classes;

    std::map< std::string, float > class_probabilities;

    int id              = -1;

    thing_state_t state = thing_state_t::NONE;

    // Methods
    thing( void ) : logger( rclcpp::get_logger( "thing_misc" ) ) {}

    thing(
        std::shared_ptr< std::map< std::string, std::pair< int, int > > >& class_map, int id,
        const rclcpp::Logger& _logger ) :
        observations( std::make_unique< observation_histogram >( HISTOGRAM_BINS ) ),
        reg_classes( class_map ), id( id ), logger( _logger )
    // observations(std::make_unique<observation_histogram>(HISTOGRAM_BINS))
    // o(std::make_unique<observation_histogram>(HISTOGRAM_BINS))
    {
        // observation_histogram observations =
        // observation_histogram( HISTOGRAM_BINS );
        // this->observations = observation_histogram::observation_histogram( 36 );
        // this->reg_classes = class_map;
    }

    thing( semantic_type_t type, int id, const rclcpp::Logger& _logger ) : logger( _logger )
    {
        this->type = type;
        this->id   = id;
    }

    virtual ~thing() {}

    std::pair< std::string, int > get_label( void ) const;

    // TODO: Transform inline

    double get_class_confidence( void ) const
    {
        if( this->get_label().first == UNDEFINED_LABEL ) return 0.0;
        return log_odds_inv( this->class_probabilities.at( this->get_label().first ) );
    }

    double get_position_confidence( void ) const
    {
        if( this->get_label().first == UNDEFINED_LABEL ) return 0.0;
        return log_odds_inv( this->pos_confidence );
    }

    double get_combined_confidence( const double confidence_threshold ) const
    {
        if( this->get_label().first == UNDEFINED_LABEL )
        {
            RCLCPP_WARN( this->logger, "undefined label" );
            return 0.0;
        }

        double t1 = ( log_odds_inv( this->class_probabilities.at( this->get_label().first ) ) * 4.0 / 6.0 );

        double t2 = ( log_odds_inv( this->pos_confidence ) / 6.0 );
        double t3 = ( this->observations->get_histogram_ratio() / 6.0 );

        if( !( ( t1 + t2 + t3 ) > confidence_threshold ) )
            RCLCPP_WARN( this->logger, "Combined conf| t1: %f, t2: %f, t3: %f", t1, t2, t3 );

        return t1 + t2 + t3;
    }

    bool label_is_equal( const uint8_t& module_id, const uint8_t& obs_label );

    std::pair< std::string, std::string > get_vertex_representation();

    void set(
        const semantic_type_t type, const std::vector< float >& probability_distribution,
        const geometry_msgs::msg::Point& point, std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb,
        const float& pos_confidence, const double& distance, const double& angle, const detector_t& detector );

    geometry_msgs::msg::Point update(
        const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
        std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
        double distance, double angle, const detector_t& detector );

    void decay_confidence( const double prob_decay_factor, const double& distance, const double& factor );

    void decay( const double& distance, const double& angle, const double& base_decay, const double& decay_factor );
    // {
    //     // current_likelihood - is a map containing a vector of probabilities that represents the probability of
    //     beeing
    //     // each class
    //     if( !( factor > 0 && factor < 1 ) )
    //     {
    //         RCLCPP_ERROR(
    //             this->logger, "Decay confidence error. Factor: %f. It should be ( factor > 0 && factor < 1 )" );
    //     }
    //     assert( factor < distance );
    //     float pre_sum = 0, sum = 0;
    //     for( auto& class_likelihood: this->class_probabilities )
    //     {
    //         float mod = ( ( 1 + factor * 3 ) / ( 1 + ( distance / 2 ) ) )
    //                   * ( 1 - ( this->observations->get_histogram_ratio() / 2 ) );
    //         // 0 <= mod <= 4
    //         float p_value = 0.5 - prob_decay_factor * ( mod / 8.0 );
    //         // 0 < pvalue <= 0.5
    //         // RCLCPP_DEBUG(
    //         //     this->logger, "p_value: %f, comp: %f, distance: %f, prob_decay_factor: %f",
    //         //     0.5 - prob_decay_factor * ( 1 / 4.0 ) * ( ( 1 + 3 * factor ) / ( 1 + distance ) ),
    //         //     abs( 0.5 - prob_decay_factor * 0.5 ), distance, prob_decay_factor );
    //         // assert( p_value <= abs( 0.5 - prob_decay_factor * 0.5 ) );
    //         pre_sum += log_odds_inv( class_likelihood.second );
    //         // class_likelihood.second += log_odds( ( prob_decay_factor * ( 1 + factor ) ) / ( 1 + distance ) );
    //         // // Clamping
    //         // if( class_likelihood.second < -LOG_ODDS_CLAMPING ) class_likelihood.second = -LOG_ODDS_CLAMPING;
    //         // if( class_likelihood.second > LOG_ODDS_CLAMPING ) class_likelihood.second = LOG_ODDS_CLAMPING;

    // assert( p_value <= 0.5 );

    // class_likelihood.second  = clamping_log_odds_sum< float >( class_likelihood.second, p_value );
    // sum                     += log_odds_inv( class_likelihood.second );
    // }
    // RCLCPP_INFO( this->logger, "sum: %f, pre_sum: %f", sum, pre_sum );
    // assert( sum <= pre_sum );
    // }

    bool is_valid( const double confidence_threshold ) const;

    inline bool class_prob_is_valid( void ) const
    {
        double sum = 0;
        std::string vals;
        for( const auto& c: this->class_probabilities )
        {
            sum  += log_odds_inv( c.second );
            vals += std::to_string( log_odds_inv( c.second ) ) + std::string( ", " );
        }
        // RCLCPP_WARN( this->logger, "class_prob sum: %f | %s| ", sum, vals.c_str() );
        assert( sum < 1.01 );
        return ( sum < 1.01 );
    }

  private:

    float pos_confidence;

    friend class boost::serialization::access;
    rclcpp::Logger logger;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar & type;
        // TODO: Implement
    }
};

}  // namespace smap

BOOST_CLASS_VERSION( smap::thing, 0 )

#endif  // SMAP_CORE__THING_HPP_

#ifndef SMAP_CORE__THING_HPP_
#define SMAP_CORE__THING_HPP_

// STL
#include "stdio.h"

#include <iostream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "visibility_control.h"

// BOOST
#include <boost/histogram/axis.hpp>
#include <boost/histogram/make_histogram.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/version.hpp>

namespace smap
{

#include <boost/histogram/axis.hpp>
#include <boost/histogram/make_histogram.hpp>
#include <iostream>

double log_odds( double prob ) { return log( prob / ( 1 - prob ) ); }

double log_odds_inv( double lodds ) { return 1 - 1 / ( 1 + exp( lodds ) ); }

struct observation_histogram
{
    using opts = decltype( boost::histogram::axis::option::circular );
    using axes_t =
        boost::histogram::axis::regular< double, boost::histogram::use_default, boost::histogram::use_default, opts >;
    using histogram_t = boost::histogram::histogram< std::tuple< axes_t > >;

    histogram_t histogram;
    size_t n_bins;
    size_t l            = 5;
    double weights[ 5 ] = { 0.5, 1.5, 6, 1.5, 0.5 };
    double factor       = 1.0 / 10;  // (1 / sum(weights) )
    double bin_width;

    observation_histogram( size_t n_bins )
    {
        this->histogram = boost::histogram::make_histogram(
            boost::histogram::axis::regular<
                double, boost::histogram::use_default, boost::histogram::use_default, opts >( n_bins, 0, 360 ) );
        this->bin_width = 360.0 / n_bins;
        this->n_bins    = n_bins;
    }

    void register_obs( double distance, double angle, bool positive )
    {
        const double prob_increase = 0.3;
        double add_value           = prob_increase / ( 1 + distance );

        for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
        {
            int idx = this->histogram.axis().index( angle + this->bin_width * i );
            // printf(
            //     "idx: %2i, factor: %8.4f, weight: %8.4f, add_value: %8.4f, prob_comp: %8.4f\n", idx, this->factor,
            //     this->weights[ j ], add_value, 0.5 + this->factor * this->weights[ j ] * add_value );
            if( positive ) this->histogram[ idx ] += log_odds( 0.5 + this->factor * this->weights[ j ] * add_value );
            else this->histogram[ idx ] -= log_odds( 0.5 + this->factor * this->weights[ j ] * add_value );
        }
    }

    bool object_is_valid( double current_angle )
    {
        const double upper_sat_threshold = 0.90, upper_threshold = 0.7, lower_threshold = 0.4;
        size_t pos_bins = 0, neg_bins = 0;
        int idx;
        double sum = 0, prob;
        for( size_t i = 0; i < this->histogram.size(); i++ )
        {
            prob  = log_odds_inv( this->histogram[ i ] );
            sum  += prob - 0.5;
            if( prob > upper_threshold ) pos_bins++;
            if( prob < lower_threshold ) neg_bins++;
        }
        // First condition
        // - Current active area greater than the upper saturation threshold
        for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
        {
            idx = this->histogram.axis().index( current_angle + this->bin_width * i );
            if( log_odds_inv( this->histogram[ idx ] ) > upper_sat_threshold ) return true;
        }
        // Second condition
        // - Max occlusion (1/5)*360
        if( neg_bins >= this->n_bins * ( 4 / 5 ) ) return false;
        // Third condition
        // - More than (4/5) of bins positive
        if( pos_bins >= this->n_bins * ( 4 / 5 ) ) return true;
        // Fourth condition
        // - (Number of positive bins greater than the number of negative ones) and (Sum of values greater than 0)
        if( ( pos_bins > neg_bins ) && ( sum > 0 ) ) return true;

        // Fifth condition
        // - Current active area greater than the upper threshold
        for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
        {
            idx = this->histogram.axis().index( current_angle + this->bin_width * i );
            if( log_odds_inv( this->histogram[ idx ] ) > upper_threshold ) return true;
        }

        return false;
    }

    void print( void )
    {
        for( size_t i = 0; i < this->histogram.size(); i++ )
        {
            printf(
                "Bin [%2i], bin center: %5.1f, value: %8.4f\n", (int) i, this->histogram.axis().bin( i ).center(),
                (double) log_odds_inv( this->histogram[ i ] ) );
        }
    }
};

enum semantic_type_t
{
    OBJECT,
    LOCATION
};

// Can be a scene or object thing

class thing
{
  public:

    // Attributes
    semantic_type_t type               = semantic_type_t::LOCATION;
    observation_histogram observations = observation_histogram( 36 );  // Polar histogram of the observations

                                                                       // Methods
    // thing( void ) { this->histogram = polar_histogram::polar_histogram(36); }
    thing( void ) {}

    thing( semantic_type_t type ) { this->type = type; }

    virtual ~thing() {}

    std::string get_label();

    std::pair< std::string, std::string > get_vertex_representation();

    void update( semantic_type_t type, double angle );

  private:

    friend class boost::serialization::access;
    rclcpp::Logger logger = rclcpp::get_logger( "thing" );

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        (void) version;
        ar& type;
    }
};

}  // namespace smap

BOOST_CLASS_VERSION( smap::thing, 0 )

#endif  // SMAP_CORE__THING_HPP_

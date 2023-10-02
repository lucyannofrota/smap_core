#ifndef SMAP_CORE__OBSERVATION_HISTOGRAM_HPP_
#define SMAP_CORE__OBSERVATION_HISTOGRAM_HPP_

// C Lib
#include <math.h>

// STL
#include <memory>
#include <tuple>

// BOOST
#include <boost/histogram/axis.hpp>
#include <boost/histogram/make_histogram.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>

namespace smap
{
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

    observation_histogram( size_t n_bins );

    void register_obs( double distance, double angle, bool positive );

    double get_histogram_ratio( void ) const;

    bool object_is_valid( void ) const;

    // bool object_is_valid( double current_angle ) const;

    void print( void ) const;

    friend class boost::serialization::access;

    template< class Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        ar & version;
        ar & histogram;
        ar & n_bins;
        ar & l;
        ar & weights;
        ar & factor;
        ar & bin_width;
    }
};

}  // namespace smap

#endif  // SMAP_CORE__OBSERVATION_HISTOGRAM_HPP_

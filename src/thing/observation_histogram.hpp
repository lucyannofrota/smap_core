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

    bool object_is_valid( double current_angle );

    void print( void );
};

}  // namespace smap

#endif  // SMAP_CORE__OBSERVATION_HISTOGRAM_HPP_

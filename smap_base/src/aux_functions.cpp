#include "../include/smap_base/aux_functions.hpp"

namespace smap
{
// TODO: convert all to inline
double rad2deg( double rad ) { return rad * ( 180 / M_PI ); }

double deg2rad( double deg ) { return deg * ( M_PI / 180 ); }

double log_odds( double prob )
{
    // assert( ( prob <= 1 ) && ( prob >= 0 ) );
    double ret = log( prob / ( 1 - prob ) );
    if( ret < -LOG_ODDS_CLAMPING ) ret = -LOG_ODDS_CLAMPING;
    if( ret > LOG_ODDS_CLAMPING ) ret = LOG_ODDS_CLAMPING;
    return ret;
}

double log_odds_inv( double lodds )
{
    double ret = ( 1 - 1 / ( 1 + exp( lodds ) ) );
    if( ret < 0 ) ret = 0;
    if( ret > 1 ) ret = 1;
    return ret;
}

bool compare_str( const std::string str1, const std::string str2 ) { return str1 == str2; }

}
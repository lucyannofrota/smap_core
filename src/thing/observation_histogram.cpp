#include "observation_histogram.hpp"

#include "../../include/smap_core/aux_functions.hpp"
#include "../../include/smap_core/macros.hpp"

namespace smap
{

observation_histogram::observation_histogram( size_t n_bins )
{
    this->histogram = boost::histogram::make_histogram(
        boost::histogram::axis::regular< double, boost::histogram::use_default, boost::histogram::use_default, opts >(
            n_bins, 0, 2 * M_PI ) );
    this->bin_width = ( 2 * M_PI ) / n_bins;
    this->n_bins    = n_bins;
    for( size_t i = 0; i < this->histogram.size(); i++ ) this->histogram[ i ] = log_odds( 0.5 );
}

void observation_histogram::register_obs( double distance, double angle, bool positive )
{
    // printf( "reg_obs\n" );

    // int c            = this->histogram.axis().size();
    // int b            = this->histogram.size();

    double add_value = HISTOGRAM_BIN_CHANGE_VALUE / ( 1 + distance );

    int idx;
    // printf( "----------------\n" );
    for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
    {
        // TODO: Check -= log_odds() -> This should be += log_odds()
        // 			Check if p_value < 0.5 in negative case and p_value > 0.5 in positive case.
        idx = this->histogram.axis().index( angle + this->bin_width * i );
        if( std::isnan( this->histogram[ idx ] ) || std::isinf( this->histogram[ idx ] ) )
            this->histogram[ idx ] = log_odds( 0.5 );
        double p_value = 0.5;
        if( positive ) p_value += this->factor * this->weights[ j ] * add_value;
        else p_value -= this->factor * this->weights[ j ] * add_value;
        assert( positive ? ( p_value >= 0.5 ) : ( p_value <= 0.5 ) );
        // Clamping
        if( p_value < 0 || p_value < log_odds_inv( -LOG_ODDS_CLAMPING ) ) this->histogram[ idx ] = -LOG_ODDS_CLAMPING;
        else
        {
            //
            if( p_value > 1 || p_value > log_odds_inv( LOG_ODDS_CLAMPING ) ) this->histogram[ idx ] = LOG_ODDS_CLAMPING;
            else this->histogram[ idx ] += log_odds( p_value );
        }
        // printf( "reg: [%i]: %6.2f\n", i, (double) this->histogram[ idx ] );
        assert( this->histogram[ idx ] >= -LOG_ODDS_CLAMPING && this->histogram[ idx ] <= LOG_ODDS_CLAMPING );
    }
}

double observation_histogram::get_histogram_ratio( void ) const
{
    const double upper_threshold = 0.7, lower_threshold = 0.4;
    size_t pos_bins = 0, neg_bins = 0, relevant_values = 0;
    double prob, ratio = 0;
    for( size_t i = 0; i < this->histogram.size(); i++ )
    {
        if( std::isnan( this->histogram[ i ] ) || std::isinf( this->histogram[ i ] )
            /*|| !( ( this->histogram[ i ] > -LOG_ODDS_CLAMPING ) && ( this->histogram[ i ] < LOG_ODDS_CLAMPING ) )*/ )
            continue;
        prob = log_odds_inv( this->histogram[ i ] );
        // sum  += prob - 0.5;
        if( prob > upper_threshold )
        {
            relevant_values++;
            pos_bins++;
        }
        if( prob < lower_threshold )
        {
            relevant_values++;
            neg_bins++;
        }
    }
    // TODO: Check values
    ratio = ( relevant_values > 0 ? ( ( (pos_bins) *1.0 ) / relevant_values ) : 0 );
    return ratio;
}

bool observation_histogram::object_is_valid( void ) const
{
    const double upper_threshold = 0.7, lower_threshold = 0.4;
    size_t pos_bins = 0, neg_bins = 0, relevant_values = 0;
    double sum = 0, prob;
    // bool ret   = false;
    for( size_t i = 0; i < this->histogram.size(); i++ )
    {
        if( std::isnan( this->histogram[ i ] ) || std::isinf( this->histogram[ i ] ) ) continue;
        prob  = log_odds_inv( this->histogram[ i ] );
        sum  += prob - 0.5;
        if( prob > upper_threshold )
        {
            relevant_values++;
            pos_bins++;
        }
        if( prob < lower_threshold )
        {
            relevant_values++;
            neg_bins++;
        }
    }
    // First condition
    // - Max occlusion (1/5)*(2*M_PI)
    if( neg_bins >= relevant_values * ( 3.5 / 5.0 ) ) return false;
    // Second condition
    // - More than (4/5) of bins positive
    if( pos_bins >= relevant_values * ( 3.5 / 5.0 ) ) return true;
    // Third condition
    // - (Number of positive bins greater than the number of negative ones) and (Sum of values greater than 0)
    if( ( pos_bins > neg_bins ) && ( sum > 0 ) ) return true;

    return false;
}

// bool observation_histogram::object_is_valid( double current_angle ) const
// {
//     // printf( "object_is_valid\n" );
//     const double upper_sat_threshold = 0.90, upper_threshold = 0.7, lower_threshold = 0.4;
//     size_t pos_bins = 0, neg_bins = 0;
//     int idx;
//     double sum = 0, prob;
//     for( size_t i = 0; i < this->histogram.size(); i++ )
//     {
//         prob  = log_odds_inv( this->histogram[ i ] );
//         sum  += prob - 0.5;
//         if( prob > upper_threshold ) pos_bins++;
//         if( prob < lower_threshold ) neg_bins++;
//     }
//     // First condition
//     // - Current active area greater than the upper saturation threshold
//     for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
//     {
//         idx = this->histogram.axis().index( current_angle + this->bin_width * i );
//         if( log_odds_inv( this->histogram[ idx ] ) > upper_sat_threshold ) return true;
//     }
//     // Second condition
//     // - Max occlusion (1/5)*(2*M_PI)
//     if( neg_bins >= this->n_bins * ( 4 / 5 ) ) return false;
//     // Third condition
//     // - More than (4/5) of bins positive
//     if( pos_bins >= this->n_bins * ( 4 / 5 ) ) return true;
//     // Fourth condition
//     // - (Number of positive bins greater than the number of negative ones) and (Sum of values greater than 0)
//     if( ( pos_bins > neg_bins ) && ( sum > 0 ) ) return true;

// // Fifth condition
// // - Current active area greater than the upper threshold
// for( int i = -int( floor( this->l / 2 ) ), j = 0; i <= int( floor( this->l / 2 ) ); i++, j++ )
// {
//     idx = this->histogram.axis().index( current_angle + this->bin_width * i );
//     if( log_odds_inv( this->histogram[ idx ] ) > upper_threshold ) return true;
// }

// return false;
// }

void observation_histogram::print( void ) const
{
    printf( "Bins       |" );
    for( size_t i = 0; i < this->histogram.size(); i++ ) printf( "%4i|", (int) i );
    printf( "\nBin center |" );
    for( size_t i = 0; i < this->histogram.size(); i++ )
        printf( "%4i|", (int) rad2deg( this->histogram.axis().bin( i ).center() ) );
    printf( "\nValue      |" );
    for( size_t i = 0; i < this->histogram.size(); i++ )
        printf( "%4.1f|", (double) log_odds_inv( this->histogram[ i ] ) );
    printf( "\n" );
}

}  // namespace smap

#include "thing.hpp"

#include "../include/smap_core/stacking_classification.hpp"

namespace smap
{

std::string thing::get_label( void ) const
{
    if( this->reg_classes == nullptr )
    {
        return UNDEFINED_LABEL;
        //
    }
    std::string ret = UNDEFINED_LABEL;
    float value     = 0;
    for( auto e: this->class_probabilities )
    {
        if( e.second > value )
        {
            value = e.second;
            ret   = e.first;
        }
    }

    return ret;
}

bool thing::label_is_equal( const uint8_t& module_id, const uint8_t& obs_label )
{
    (void) module_id;  // TODO: module_id verification
    if( this->reg_classes == nullptr ) return false;

    // Determine the string value of the current label
    // for( auto reg: ( **this->reg_classes ) )
    //     printf( "reg: %s|%i|%i\n", reg.first.c_str(), reg.second.first, reg.second.second );
    std::string current_label = this->get_label();
    if( this->reg_classes->find( current_label ) != this->reg_classes->end() )
    {
        // Compare it's value with the given one
        // 1. Get the pair of indexes
        std::pair< int, int > idxs = ( *this->reg_classes )[ current_label ];
        // printf( "Label: %s, id1: %i, id2: %i\n", current_label.c_str(), idxs.first, idxs.second );
        // 2. Check if the values detector value matches
        return idxs.second == obs_label;
    }
    return false;
}

// std::string thing::get_label( uint8_t module_id )
// {
//     (void) module_id;
//     if( this->reg_classes == nullptr ) return UNDEFINED_LABEL;
//     if( *( this->reg_classes ) == nullptr ) return UNDEFINED_LABEL;

// return UNDEFINED_LABEL;
// }

std::pair< std::string, std::string > thing::get_vertex_representation()
{
    return std::pair< std::string, std::string >( UNDEFINED_LABEL, std::string( "red" ) );
}

void thing::set(
    const semantic_type_t type, const std::vector< float >& probability_distribution,
    const geometry_msgs::msg::Point& point, std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb,
    const float& pos_confidence, const double& distance, const double& angle, const detector_t& detector )
{
    // set        = true;
    this->type = type;

    // 1. Histogram initialization
    // printf( "Histogram:\n" );
    // printf( "\tn_bins: %i\n", (int) this->observations.n_bins );
    // printf( "\tbin_width: %6.1f\n", rad2deg( this->observations.bin_width ) );
    this->observations->register_obs( distance, angle, true );
    // this->observations.print();

    // 2. Position initialization
    this->pos            = point;
    this->aabb           = aabb;
    this->pos_confidence = log_odds( pos_confidence );

    // 3. Probabilities vector initialization
    // probability_distribution
    int i   = 0;
    auto it = probability_distribution.begin();
    for( i = 0; it != probability_distribution.end(); ++it, i++ )
    {
        // printf( "i: %i| class: %s|value: %f\n", i, detector.classes.at( i ).c_str(), *it );
        this->class_probabilities[ detector.classes.at( i ) ] = log_odds( *it );
    }
    // for( auto c: **this->reg_classes ) this->class_probabilities[ c.first ] = log_odds( 0 );
    // // Create a map to store this information
    // // int u    = 0;
    // // double s = 0;
    // int i = 0, j = 0;
    // bool sw = false;
    // printf( "Probabilities [BEFORE]:\n" );
    // printf( "\tClass |" );
    // auto it = probability_distribution.begin();
    // while( it != probability_distribution.end() )
    // {
    //     if( sw )
    //     {
    //         printf( "%6.3f|", *it );
    //         it++;
    //         i--;
    //     }
    //     else
    //     {
    //         printf( "%6i|", i + j * 20 );
    //         i++;
    //     }
    //     if( sw && i == 0 )
    //     {
    //         sw = false;
    //         printf( "\n\tClass |" );
    //         j++;
    //         continue;
    //     }
    //     if( i % 20 == 0 )
    //     {
    //         sw = true;
    //         printf( "\n\tProbs |" );
    //         continue;
    //     }
    // }
    // // this->probabilities
    // for( auto c: **this->reg_classes )
    //     this->probabilities[ c.first ] = log_odds( probability_distribution[ c.second.second ] );

    // i  = 0;
    // j  = 0;
    // sw = false;
    // printf( "Probabilities [AFTER]:\n" );
    // printf( "\tClass |" );
    // auto itp = this->probabilities.begin();
    // while( itp != this->probabilities.end() )
    // {
    //     if( sw )
    //     {
    //         printf( "%6.3f|", log_odds_inv( ( *itp ).second ) );
    //         itp++;
    //         i--;
    //     }
    //     else
    //     {
    //         printf( "%6i|", i + j * 20 );
    //         i++;
    //     }
    //     if( sw && i == 0 )
    //     {
    //         sw = false;
    //         printf( "\n\tClass |" );
    //         j++;
    //         continue;
    //     }
    //     if( i % 20 == 0 )
    //     {
    //         sw = true;
    //         printf( "\n\tProbs |" );
    //         continue;
    //     }
    // }
}

geometry_msgs::msg::Point thing::update(
    const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
    double distance, double angle, const detector_t& detector )
{
    // 1. Histogram update
    this->observations->register_obs( distance, angle, true );

    // 2. Probabilities vector update
    if( this->class_probabilities.size() != this->reg_classes->size() )
    {
        // find
        for( auto c: ( *this->reg_classes ) )
        {
            // if class not found add it to the map
            if( this->class_probabilities.find( c.first ) == this->class_probabilities.end() )
                this->class_probabilities[ c.first ] = log_odds( 0 );
        }
    }

    // 3. Stack vectors
    stack_vectors( this->class_probabilities, probability_distribution, detector );

    // 4. Position update
    this->pos_confidence += log_odds( pos_confidence );
    // Saturation of p. Max value is MAX_POS_PROB
    double p = ( log_odds_inv( this->pos_confidence ) > MAX_POS_PROB )
                 ? MAX_POS_PROB
                 : ( log_odds_inv( this->pos_confidence ) > MAX_POS_PROB );
    // Low pass filter to update the position with p as coefficient
    this->pos         = this->pos * p + point * ( 1 - p );

    this->aabb.first  = this->aabb.first * p + aabb.first * ( 1 - p );
    this->aabb.second = this->aabb.second * p + aabb.second * ( 1 - p );
    return this->pos;
}

bool thing::is_valid( void ) const
{
    switch( this->type )
    {
    case semantic_type_t::OBJECT:
        return (
            this->observations->object_is_valid() && ( this->get_combined_confidence() > CONFIDENCE_OBJECT_VALID )
            && ( ( log_odds_inv( this->class_probabilities.at( this->get_label() ) ) > 0.5 )
                 && ( log_odds_inv( this->pos_confidence ) > 0.5 )
                 && ( this->observations->get_histogram_ratio() > 0.5 ) )
            && ( this->get_label() != UNDEFINED_LABEL ) );
        break;
    case semantic_type_t::LOCATION:
        return true;
        break;

    default:
        return false;
        break;
    }
}

// int thing::_get_label( void )
// {
//     // Return the numeric value of the class with higher probability
//     return 0;
// }
}  // namespace smap

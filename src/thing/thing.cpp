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
    this->observations->register_obs( distance, angle, true );

    // 2. Position initialization
    this->pos            = point;
    this->aabb           = aabb;
    this->pos_confidence = log_odds( pos_confidence );

    // 3. Probabilities vector initialization
    // probability_distribution
    int i   = 0;
    auto it = probability_distribution.begin();
    for( i = 0; it != probability_distribution.end(); ++it, i++ )
        this->class_probabilities[ detector.classes.at( i ) ] = log_odds( *it );
}

geometry_msgs::msg::Point thing::update(
    const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
    double distance, double angle, const detector_t& detector )
{
    // 1. Histogram update
    printf( "%s-id: %i\n", this->get_label().c_str(), this->id );
    this->observations->register_obs( distance, angle, true );

    // 2. Position update
    this->pos_confidence += log_odds( pos_confidence );
    // Clamping
    if( this->pos_confidence < -LOG_ODDS_CLAMPING ) this->pos_confidence = -LOG_ODDS_CLAMPING;
    if( this->pos_confidence > LOG_ODDS_CLAMPING ) this->pos_confidence = LOG_ODDS_CLAMPING;
    // Saturation of p. Max value is MAX_POS_PROB
    double p = ( log_odds_inv( this->pos_confidence ) > MAX_POS_PROB )
                 ? MAX_POS_PROB
                 : ( log_odds_inv( this->pos_confidence ) > MAX_POS_PROB );
    // Low pass filter to update the position with p as coefficient
    this->pos         = this->pos * p + point * ( 1 - p );

    this->aabb.first  = this->aabb.first * p + aabb.first * ( 1 - p );
    this->aabb.second = this->aabb.second * p + aabb.second * ( 1 - p );
    return this->pos;

    // 3. Probabilities vector update
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
    stack_vectors( this->class_probabilities, probability_distribution, detector );
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

}  // namespace smap

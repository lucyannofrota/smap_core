#include "include/thing/thing.hpp"

#include "smap_base/stacking_classification.hpp"

namespace smap
{

std::pair< std::string, int > thing::get_label( void ) const
{
    RCLCPP_DEBUG( this->logger, "get_label()" );
    if( this->reg_classes == nullptr )
    {
        return std::pair< std::string, int >( UNDEFINED_LABEL, -1 );
        //
    }
    std::pair< std::string, int > ret( UNDEFINED_LABEL, 0 );
    float value = 0;
    int i       = 0;
    for( const auto& e: this->class_probabilities )
    {
        if( log_odds_inv( e.second ) > value )
        {
            value      = log_odds_inv( e.second );
            ret.first  = e.first;
            ret.second = i;
        }
        i++;
    }

    // if( value < 0.35 ) return std::pair< std::string, int >( UNDEFINED_LABEL, -1 );
    // printf( "get_label: %s| value: %f\n", ret.c_str(), value );

    return ret;
}

bool thing::label_is_equal( const uint8_t& module_id, const uint8_t& obs_label )
{
    (void) module_id;  // TODO: module_id verification
    RCLCPP_DEBUG( this->logger, "thing::label_is_equal(), u_val: %i", (int) obs_label );
    if( this->reg_classes == nullptr ) return false;

    // Determine the string value of the current label
    // for( auto reg: ( **this->reg_classes ) )
    //     printf( "reg: %s|%i|%i\n", reg.first.c_str(), reg.second.first, reg.second.second );
    std::string current_label = this->get_label().first;
    if( current_label == UNDEFINED_LABEL ) return false;
    RCLCPP_DEBUG( this->logger, "current_label: --" );
    RCLCPP_DEBUG( this->logger, "current_label: %s", current_label.c_str() );

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
    {
        float p_value = ( ( ( *it ) - 0.5 ) * OBJECT_UPDATE_FACTOR ) + 0.5;  // Gain factor
        this->class_probabilities[ detector.classes.at( i ) ] = log_odds( p_value );
    }
    stack_normalization( this->class_probabilities );
    assert( this->class_prob_is_valid() );
    // assert( this->get_label().second == 75 || this->get_label().second == -1 );  // TODO: Remove
    // stack_vectors( this->class_probabilities, probability_distribution, detector );
}

geometry_msgs::msg::Point thing::update(
    const std::vector< float >& probability_distribution, geometry_msgs::msg::Point& point,
    std::pair< geometry_msgs::msg::Point, geometry_msgs::msg::Point > aabb, const float& pos_confidence,
    double distance, double angle, const detector_t& detector )
{
    // TODO: Debug update changing label!
    // 1. Histogram update
    // printf( "Update b l:%s |id:%i\n", this->get_label().first.c_str(), this->id );
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

    // 3. Probabilities vector update
    if( this->class_probabilities.size() != this->reg_classes->size() )
    {
        // find
        for( const auto& c: ( *this->reg_classes ) )
        {
            // if class not found add it to the map
            if( this->class_probabilities.find( c.first ) == this->class_probabilities.end() )
                this->class_probabilities[ c.first ] = 0;
        }
    }
    stack_vectors( this->class_probabilities, probability_distribution, detector, OBJECT_UPDATE_FACTOR );
    assert( this->class_prob_is_valid() );
    // printf( "Update e l:%s |id:%i\n", this->get_label().first.c_str(), this->id );
    // test_label( this->get_label().first, "tv" );
    // double sum = 0;
    // for( const auto& x: this->class_probabilities ) sum += log_odds_inv( x.second );
    // if( this->get_label().second != 75 && this->get_label().second != -1 )
    //     printf( "\n\n\n----------------------------------------\n----------------------------------------\nthing::"
    //             "update()\n\t label != tv\n"
    //             "----------------------------------------\n----------------------------------------\n\n\n\n" );
    return this->pos;
}

void thing::decay_confidence( const double prob_decay_factor, const double& distance, const double& factor )
{
    // current_likelihood - is a map containing a vector of probabilities that represents the probability of beeing
    // each class
    if( !( factor > 0 && factor < 1 ) )
        RCLCPP_ERROR( this->logger, "Decay confidence error. Factor: %f. It should be ( factor > 0 && factor < 1 )" );
    assert( factor < distance );
    float pre_sum = 0, sum = 0;
    for( auto& class_likelihood: this->class_probabilities )
    {
        float mod =
            ( ( 1 + factor ) / ( 1 + ( distance / 4 ) ) ) * ( 1 - ( this->observations->get_histogram_ratio() / 4 ) );
        // 0 <= mod <= 2
        float p_value = 0.5 - prob_decay_factor * ( mod / 4.0 );
        // 0 < pvalue <= 0.5
        pre_sum += log_odds_inv( class_likelihood.second );

        assert( p_value <= 0.5 );

        class_likelihood.second  = clamping_log_odds_sum< float >( class_likelihood.second, p_value );
        sum                     += log_odds_inv( class_likelihood.second );
    }
    RCLCPP_INFO( this->logger, "sum: %f, pre_sum: %f", sum, pre_sum );
    assert( sum <= pre_sum );
}

void thing::decay( const double& distance, const double& angle, const double& base_decay, const double& decay_factor )
{
    double factor = ( decay_factor < 1 ) ? decay_factor : 1;
    // Decay observation histogram
    this->observations->register_obs( distance, angle, false );
    // Decay confidence
    this->decay_confidence( base_decay, distance, factor );
}

bool thing::is_valid( const double confidence_threshold ) const
{
    switch( this->type )
    {
    case semantic_type_t::OBJECT:
        // if( !( this->observations->object_is_valid() && this->class_prob_is_valid()
        //        && ( this->get_combined_confidence() > CONFIDENCE_OBJECT_VALID )
        //        && ( ( log_odds_inv( this->class_probabilities.at( this->get_label().first ) ) > 0.7 )
        //             && ( log_odds_inv( this->pos_confidence ) > 0.5 )
        //             && ( this->observations->get_histogram_ratio() > 0.5 ) )
        //        && ( this->get_label().first != UNDEFINED_LABEL ) ) )
        // {
        //     printf( " " );
        //     printf( " " );
        // }
        // Test
        // if( !this->observations->object_is_valid() )
        // {  // Cond 1
        //     RCLCPP_WARN( this->logger, "observations->object_is_valid invalid\n" );
        //     return false;
        // }
        if( !( this->get_combined_confidence( confidence_threshold ) > confidence_threshold ) )
        {  // Cond 3

            RCLCPP_WARN(
                this->logger,
                "( this->get_combined_confidence(confidence_threshold) > confidence_threshold ) invalid\n" );
            return false;
        }
        // if( !( log_odds_inv( this->pos_confidence ) > 0.3 ) )
        // {  // Cond 4
        //     RCLCPP_WARN( this->logger, "( log_odds_inv( this->pos_confidence ) > 0.5 ) invalid\n" );
        //     return false;
        // }
        // if( !( this->observations->get_histogram_ratio() >= 0.5 ) )
        // {  // Cond 5
        //     RCLCPP_WARN(
        //         this->logger, "( this->observations->get_histogram_ratio() [%f] > 0.5 ) invalid\n",
        //         this->observations->get_histogram_ratio() );
        //     return false;
        // }
        if( !( this->get_label().first != UNDEFINED_LABEL ) )
        {  // Cond 6
            RCLCPP_WARN( this->logger, "( this->get_label().first != UNDEFINED_LABEL ) invalid\n" );
            return false;
        }
        RCLCPP_WARN( this->logger, "Cond Pass\n" );
        return true;
        // return (
        //     this->observations->object_is_valid() && this->class_prob_is_valid()
        //     && ( this->get_combined_confidence() > CONFIDENCE_OBJECT_VALID )
        //     && ( ( log_odds_inv( this->class_probabilities.at( this->get_label().first ) ) > 0.7 )
        //          && ( log_odds_inv( this->pos_confidence ) > 0.5 )
        //          && ( this->observations->get_histogram_ratio() > 0.5 ) )
        //     && ( this->get_label().first != UNDEFINED_LABEL ) );
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

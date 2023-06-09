#include "../include/smap_core/thing.hpp"

namespace smap
{

std::string thing::get_label( void )
{
    if( this->reg_classes == nullptr ) return UNDEFINED_LABEL;
    if( *( this->reg_classes ) == nullptr ) return UNDEFINED_LABEL;

    // for( auto e: ( **( this->reg_classes ) ) )
    //     if( e.second.first == this->_get_label() ) return e.first;
    std::string ret = UNDEFINED_LABEL;
    float value     = 0;
    for( auto e: this->probabilities )
    {
        if( e.second > value )
        {
            value = e.second;
            ret   = e.first;
        }
    }

    return ret;
}

bool thing::label_is_equal( uint8_t& module_id, uint8_t& obs_label )
{
    (void) module_id;  // TODO: module_id verification
    if( this->reg_classes == nullptr ) return false;
    if( *( this->reg_classes ) == nullptr ) return false;

    // Determine the string value of the current label
    std::string current_label = this->get_label();
    if( ( **this->reg_classes ).find( current_label ) == ( **this->reg_classes ).end() )
    {
        // Compare it's value with the given one
        // 1. Get the pair of indexes
        std::pair< int, int > idxs = ( **this->reg_classes )[ current_label ];
        // 2. Check if the values detector value matches
        return idxs.second == obs_label;
    }
    return false;
}

// std::string thing::get_label( uint8_t module_id )
// {
//     // TODO: Class equivalent to a given detector
//     (void) module_id;
//     if( this->reg_classes == nullptr ) return UNDEFINED_LABEL;
//     if( *( this->reg_classes ) == nullptr ) return UNDEFINED_LABEL;

// return UNDEFINED_LABEL;
// }

std::pair< std::string, std::string > thing::get_vertex_representation()
{
    return std::pair< std::string, std::string >( UNDEFINED_LABEL, std::string( "red" ) );
}

void thing::update(
    const semantic_type_t type, const smap_interfaces::msg::SmapObject& obj, double distance, double angle,
    detector_t& detector )
{
    (void) angle;
    (void) obj;
    static bool set = false;
    if( !set )
    {
        // set        = true;
        this->type = type;
        // TODO: Define initialization

        // 1. Histogram initialization
        printf( "Histogram:\n" );
        printf( "\tn_bins: %i\n", (int) this->observations.n_bins );
        printf( "\tbin_width: %6.1f\n", rad2deg( this->observations.bin_width ) );
        this->observations.register_obs( distance, angle, true );
        this->observations.print();

        // 2. Position initialization

        // 3. Probabilities vector initialization
        // Create a map to store this information
        // int u    = 0;
        // double s = 0;
        int i = 0, j = 0;
        bool sw = false;
        printf( "Probabilities [BEFORE]:\n" );
        printf( "\tClass |" );
        auto it = obj.probability_distribution.begin();
        while( it != obj.probability_distribution.end() )
        {
            if( sw )
            {
                printf( "%6.3f|", *it );
                it++;
                i--;
            }
            else
            {
                printf( "%6i|", i + j * 20 );
                i++;
            }
            if( sw && i == 0 )
            {
                sw = false;
                printf( "\n\tClass |" );
                j++;
                continue;
            }
            if( i % 20 == 0 )
            {
                sw = true;
                printf( "\n\tProbs |" );
                continue;
            }
        }
        // this->probabilities
        for( auto c: **this->reg_classes )
            this->probabilities[ c.first ] = log_odds( obj.probability_distribution[ c.second.second ] );

        i  = 0;
        j  = 0;
        sw = false;
        printf( "Probabilities [AFTER]:\n" );
        printf( "\tClass |" );
        auto itp = this->probabilities.begin();
        while( itp != this->probabilities.end() )
        {
            if( sw )
            {
                printf( "%6.3f|", log_odds_inv( ( *itp ).second ) );
                itp++;
                i--;
            }
            else
            {
                printf( "%6i|", i + j * 20 );
                i++;
            }
            if( sw && i == 0 )
            {
                sw = false;
                printf( "\n\tClass |" );
                j++;
                continue;
            }
            if( i % 20 == 0 )
            {
                sw = true;
                printf( "\n\tProbs |" );
                continue;
            }
        }
    }
    else
    {
        // TODO: Define
        // 1. Histogram update
        // TODO: Test
        this->observations.register_obs( distance, angle, true );

        // 2. Probabilities vector update
        // TODO: Test
        if( this->probabilities.size() != ( *this->reg_classes )->size() )
        {
            // find
            for( auto c: **this->reg_classes )
            {
                // if class not found add it to the map
                if( this->probabilities.find( c.first ) == this->probabilities.end() )
                    this->probabilities[ c.first ] = log_odds( 0 );
            }
        }

        stack_vectors( this->probabilities, obj.probability_distribution, detector );

        // 3. Position update
        // TODO: Test
        // Saturation of p. Max value is MAX_POS_PROB
        float p = ( this->probabilities[ this->get_label() ] > MAX_POS_PROB )
                    ? MAX_POS_PROB
                    : ( this->probabilities[ this->get_label() ] > MAX_POS_PROB );
        // Low pass filter to update the position with p as coefficient
        this->pos.x = this->pos.x * p + obj.pose.pose.position.x * ( 1 - p );
        this->pos.y = this->pos.y * p + obj.pose.pose.position.y * ( 1 - p );
        this->pos.z = this->pos.z * p + obj.pose.pose.position.z * ( 1 - p );
    }
}

int thing::_get_label( void )
{
    // Return the numeric value of the class with higher probability
    return 0;
}
}  // namespace smap

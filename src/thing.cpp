#include "../include/smap_core/thing.hpp"

namespace smap
{

std::string thing::get_label( void )
{
    if( this->reg_classes == nullptr ) return UNDEFINED_LABEL;
    if( *( this->reg_classes ) == nullptr ) return UNDEFINED_LABEL;

    for( auto e: ( **( this->reg_classes ) ) )
        if( e.second.first == this->_get_label() ) return e.first;

    return UNDEFINED_LABEL;
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

void thing::update( const semantic_type_t type, const smap_interfaces::msg::SmapObject& obj, double angle )
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
        printf( "\tbin_width: %f\n", this->observations.bin_width );
        int u    = 0;
        double s = 0;

        // 2. Position initialization

        // 3. Probabilities vector initialization
        // Create a map to store this information
        printf( "Probabilities\n" );
        for( auto e: obj.probability_distribution )
        {
            u++;
            s += e;
            printf( "\tProb: %6.2f\n", e );
        }

        // this->
        // this->histogram
        // this->
    }
    else
    {
        // TODO: Define
        // 1. Histogram update

        // 2. Position update

        // 3. Probabilities vector update

        // Must combine the received probabilities vector with the previous
        // x_hat[k] = x[k] + x[k-1]
        // Verify the best combination (PROBABLY NOT A SUM [+])
    }
}

int thing::_get_label( void )
{
    // Return the numeric value of the class with higher probability
    return 0;
}
}  // namespace smap
